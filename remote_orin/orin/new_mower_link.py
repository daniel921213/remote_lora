#!/usr/bin/env python3
"""115200 8N1 ST-LINK VCP client.

Read-only monitor:
    python3 mower_link.py --port /dev/ttyACM0 monitor

Finite RPM command:
    python3 mower_link.py --port /dev/ttyACM0 command \
        --left 10 --right 10 --seconds 5
"""

import argparse
import binascii
import json
import math
import os
import struct
import time


def crc16(data):
    return binascii.crc_hqx(data, 0xffff)


def command_packet(sequence, lease,
                   left=0, right=0,
                   hydraulic=0, flags=0):

    if not all(
        math.isfinite(v) and abs(v) <= 300
        for v in (left, right)
    ):
        raise ValueError(
            "RPM must be finite and within +/-300; "
            "MCU configured limit is normally lower."
        )

    if hydraulic not in (-1, 0, 1):
        raise ValueError("hydraulic must be -1, 0 or +1")

    if flags & ~15:
        raise ValueError(
            "flags: bit0 ESTOP, bit1 ENGINE_KILL, bit2 CLEAR_ESTOP, bit3 ARM_AUTO"
        )
    if not flags & 3 and flags & 12:
        if flags not in (4, 8) or left != 0 or right != 0 or hydraulic != 0:
            raise ValueError("CLEAR_ESTOP/ARM_AUTO must be separate zero/hold commands")

    p = bytearray(32)

    struct.pack_into(
        "<BBBBIIhhbB",
        p,
        0,
        0xa5,
        0x5a,
        1,
        1,
        sequence & 0xffffffff,
        lease & 0xffffffff,
        round(left * 100),
        round(right * 100),
        hydraulic,
        flags
    )

    struct.pack_into(
        "<H",
        p,
        30,
        crc16(p[:30])
    )

    return bytes(p)


def decode_packet(p):

    if len(p) != 32:
        return None

    if p[:3] != b"\xa5\x5a\x01":
        return None

    if crc16(p[:30]) != struct.unpack_from(
        "<H", p, 30
    )[0]:
        return None

    if p[3] == 0x81:

        seq, lease, left, right, lpwm, rpwm = \
            struct.unpack_from(
                "<IIhhHH",
                p,
                4
            )

        return {
            "type": "status",
            "sequence": seq,
            "lease": lease,
            "rpm": [
                left / 100.0,
                right / 100.0
            ],
            "pwm": [
                lpwm,
                rpwm
            ],
            "state": p[20],
            "health": p[21],
            "clear_ready": bool(p[21] & 1),
            "run_request": bool(p[21] & 32),
            "arm_ready": bool(p[21] & 64),
            "explicit_arm_supported": bool(p[21] & 128),
            "faults": struct.unpack_from(
                "<H", p, 22
            )[0],
            "sbus_flags": p[24],
            "fc_pwm": list(
                struct.unpack_from(
                    "<HH", p, 25
                )
            ),
            "fc_pwm_valid": p[29],
        }

    if p[3] == 0x82:

        bits = int.from_bytes(
            p[5:27],
            "little"
        )

        return {
            "type": "sbus",
            "channels": [
                (bits >> (11 * c)) & 2047
                for c in range(16)
            ],
            "flags": p[27],
            "fresh": bool(p[29]),
        }

    if p[3] == 0x83:
        length = min(p[11], 18)
        data = bytes(p[12:12 + length])
        return {
            "type": "uart5",
            "rx_count": struct.unpack_from("<I", p, 4)[0],
            "errors": struct.unpack_from("<H", p, 8)[0],
            "tx_enabled": bool(p[10] & 1),
            "dropped": bool(p[10] & 2),
            "hex": data.hex(" "),
            "ascii": "".join(chr(b) if 32 <= b < 127 else "." for b in data),
        }

    if p[3] == 0x84:
        bits = int.from_bytes(p[4:26], "little")
        return {
            "type": "fc_sbus",
            "channels": [
                (bits >> (11 * c)) & 2047
                for c in range(16)
            ],
            "flags": p[26],
            "fresh": bool(p[27]),
            "frame_count": struct.unpack_from("<H", p, 28)[0],
        }

    return None


class PacketParser:

    def __init__(self):
        self.buffer = bytearray()

    def feed(self, data):

        self.buffer.extend(data)

        result = []

        while len(self.buffer) >= 32:

            p = decode_packet(
                self.buffer[:32]
            )

            if p:
                result.append(p)
                del self.buffer[:32]

            else:
                del self.buffer[0]

        return result


class MowerLink:

    def __init__(self, port, log=None):

        import serial

        self.serial = serial.Serial(
            None,
            115200,
            timeout=0,
            write_timeout=0.1
        )
        self.serial.dtr = False
        self.serial.rts = False
        if os.name == 'posix':
            self.serial.exclusive = True
        self.serial.port = port
        self.serial.open()

        self.parser = PacketParser()

        self.status = None
        self.status_at = 0.0
        self.sequence = None

        self.log = log
        self.session = time.time_ns()

    def record(self, packet, direction):

        event = dict(
            packet,
            direction=direction,
            session=self.session,
            host_monotonic_ns=time.monotonic_ns(),
            host_time_ns=time.time_ns()
        )

        if self.log:

            self.log.write(
                json.dumps(
                    event,
                    ensure_ascii=False
                ) + "\n"
            )

            self.log.flush()

        return event

    def poll(self):

        data = self.serial.read(4096)

        if not data:
            return []

        packets = self.parser.feed(data)

        events = []

        for p in packets:

            if p["type"] == "status":

                self.status = p
                self.status_at = time.monotonic()

                if self.sequence is None:

                    self.sequence = (
                        p["sequence"] + 1
                    ) & 0xffffffff

            events.append(
                self.record(
                    p,
                    "rx"
                )
            )

        return events

    def command(self,
                left=0,
                right=0,
                hydraulic=0,
                estop=False,
                engine_kill=False,
                clear_estop=False,
                arm_auto=False):

        #
        # IMPORTANT:
        # Return events consumed here so callers such
        # as pid_live.py do not lose telemetry.
        #
        events = self.poll()

        flags = (
            int(estop)
            | (int(engine_kill) << 1)
            | (int(clear_estop) << 2)
            | (int(arm_auto) << 3)
        )

        if (
            not (flags & 3)
            and (
                not self.status
                or (
                    time.monotonic()
                    - self.status_at
                ) > 0.15
            )
        ):
            raise ConnectionError(
                "No fresh MCU telemetry; "
                "motion command not sent."
            )

        if self.sequence is None:
            self.sequence = 0

        lease = (
            self.status["lease"]
            if self.status
            else 0
        )

        packet = command_packet(
            self.sequence,
            lease,
            left,
            right,
            hydraulic,
            flags
        )

        written = self.serial.write(
            packet
        )

        if written != len(packet):

            raise ConnectionError(
                "Incomplete serial command write."
            )

        self.record(
            {
                "type": "command",
                "sequence": self.sequence,
                "lease": lease,
                "rpm": [
                    round(left * 100) / 100,
                    round(right * 100) / 100
                ],
                "hydraulic": hydraulic,
                "flags": flags
            },
            "tx"
        )

        self.sequence = (
            self.sequence + 1
        ) & 0xffffffff

        return events

    def _authority_status(self):
        if not self.status or not self.status.get("explicit_arm_supported", False):
            raise RuntimeError("Firmware does not advertise explicit ARM support; rebuild/flash first.")

    def prepare_auto(self, timeout=10.0, on_events=None):
        """Explicit operator-initiated arm. Never clears ESTOP or retries ARM.

        Send zero until STM32 reports its post-entry 500 ms zero hold complete,
        send one ARM packet, then confirm ACTIVE while keeping zero alive.
        Call once per deliberate start, never from an automatic reconnect loop.
        """
        self._authority_status()
        def deliver(events):
            if on_events: on_events(events)
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            deliver(self.command())
            if self.status["state"] == 4 or self.status["faults"]:
                raise RuntimeError("ESTOP/fault present; explicit clear-estop or local recovery required.")
            if self.status["state"] == 3 and self.status.get("arm_ready"):
                break
            time.sleep(0.05)
        else:
            raise RuntimeError("AUTO arm readiness timeout; check neutral RC, encoders and AUTO mode.")
        deliver(self.command(arm_auto=True))
        deadline = time.monotonic() + 1.0
        while time.monotonic() < deadline:
            time.sleep(0.05)
            deliver(self.command())
            if self.status["state"] != 3 or self.status["faults"]:
                raise RuntimeError("AUTO changed/faulted during ARM; request not retried.")
            if self.status.get("run_request"):
                return
        raise RuntimeError("ARM not acknowledged; request not retried.")

    def request_clear_estop(self, timeout=5.0):
        """One explicit reset request after STM32 reports one-second safe hold.
        Does not arm, does not retry reset and never sends a nonzero target.
        """
        self._authority_status()
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            self.command()
            if self.status["state"] != 4:
                raise RuntimeError("MCU is not in ESTOP; no clear request sent.")
            if self.status.get("clear_ready"):
                break
            time.sleep(0.05)
        else:
            raise RuntimeError("Clear not ready: fault source, neutral RC, engine HOLD or encoders unsafe.")
        self.command(clear_estop=True)
        deadline = time.monotonic() + 1.0
        while time.monotonic() < deadline:
            self.poll()
            if self.status["state"] == 3 and self.status["faults"] == 0:
                return
            time.sleep(0.01)
        raise RuntimeError("Clear not acknowledged; request not retried.")

    def close(self):
        self.serial.close()


def main():

    ap = argparse.ArgumentParser(
        description=__doc__
    )

    ap.add_argument(
        "--port",
        required=True
    )

    ap.add_argument(
        "--log",
        help="Optional JSONL diagnostic recording"
    )

    sub = ap.add_subparsers(
        dest="action",
        required=True
    )

    monitor = sub.add_parser(
        "monitor",
        help="Read-only monitor"
    )

    monitor.add_argument(
        "--interval",
        type=float,
        default=1.0,
        help=(
            "Display interval seconds; "
            "0 prints every packet"
        )
    )

    sub.add_parser(
        "estop",
        help="Request latched stop"
    )
    sub.add_parser("clear-estop", help="Explicitly clear safe ESTOP into AUTO standby; does not arm")

    cmd = sub.add_parser(
        "command",
        help="Finite RPM test"
    )

    cmd.add_argument(
        "--left",
        type=float,
        required=True
    )

    cmd.add_argument(
        "--right",
        type=float,
        required=True
    )

    cmd.add_argument(
        "--hydraulic",
        type=int,
        choices=(-1, 0, 1),
        default=0
    )

    cmd.add_argument(
        "--seconds",
        type=float,
        default=5
    )

    args = ap.parse_args()

    if args.action == "monitor":

        if (
            not math.isfinite(args.interval)
            or args.interval < 0
        ):
            ap.error(
                "--interval must be finite and >= 0"
            )

    if args.action == "command":

        if (
            not math.isfinite(args.seconds)
            or not 0 < args.seconds <= 30
        ):
            ap.error(
                "--seconds must be >0 and <=30"
            )

        command_packet(
            0,
            0,
            args.left,
            args.right,
            args.hydraulic
        )

    log = (
        open(
            args.log,
            "a",
            encoding="utf-8"
        )
        if args.log
        else None
    )

    link = None

    try:

        link = MowerLink(
            args.port,
            log=log
        )

        if args.action == "estop":

            for _ in range(10):

                link.command(
                    estop=True
                )

                time.sleep(0.05)

            print(
                "Stop requests sent."
            )

            return

        if args.action in ("command", "clear-estop"):

            deadline = (
                time.monotonic() + 2.0
            )

            while (
                link.status is None
                and time.monotonic()
                < deadline
            ):

                link.poll()
                time.sleep(0.01)

            if link.status is None:

                raise RuntimeError(
                    "No MCU telemetry."
                )
            if args.action == "clear-estop":
                link.request_clear_estop()
                print("ESTOP cleared: AUTO standby, motion is not armed.")
                return

            print(
                "Preparing AUTO: safe boot enters standby; if in MANUAL toggle A. "
                "Zero hold then one explicit ARM request."
            )

            link.prepare_auto()

            if (
                not link.status
                or link.status["state"] != 3
            ):

                raise RuntimeError(
                    "MCU is not in AUTO; "
                    "test aborted."
                )

            end = (
                time.monotonic()
                + args.seconds
            )

            while (
                time.monotonic()
                < end
            ):

                link.command(
                    args.left,
                    args.right,
                    args.hydraulic
                )

                if (
                    link.status["state"]
                    != 3
                ):
                    raise RuntimeError(
                        "AUTO authority lost; "
                        "test aborted."
                    )

                time.sleep(0.05)

        else:

            pending = {}

            next_display = (
                time.monotonic()
                + args.interval
            )

            while True:

                for p in link.poll():

                    if args.interval == 0:

                        print(
                            json.dumps(
                                p,
                                ensure_ascii=False
                            )
                        )

                    else:

                        pending[
                            p["type"]
                        ] = p

                now = time.monotonic()

                if (
                    args.interval > 0
                    and now >= next_display
                ):

                    for p in pending.values():

                        print(
                            json.dumps(
                                p,
                                ensure_ascii=False
                            )
                        )

                    pending.clear()

                    next_display = (
                        now
                        + args.interval
                    )

                time.sleep(0.01)

    except KeyboardInterrupt:
        pass

    finally:

        if (
            link is not None
            and args.action == "command"
        ):

            for _ in range(3):

                try:

                    link.command(
                        estop=True
                    )

                except Exception:
                    pass

                time.sleep(0.05)

        if link is not None:
            link.close()

        if log:
            log.close()


if __name__ == "__main__":
    main()
