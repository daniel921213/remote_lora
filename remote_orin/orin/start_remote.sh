#!/usr/bin/env bash
# 一個指令開好遙控整條路：main_control → stm_bridge → lora_ros_node（--fake 時先開假 STM32）。
#
#   ./start_remote.sh            真車：STM32 /dev/stm32（沒有就用 /dev/ttyACM0）、LoRa /dev/lora_gs
#   ./start_remote.sh --fake     不接車：假 STM32、ROS_DOMAIN_ID=77、TCP 8790，碰不到正在用的系統
#   其他選項：--lora-port 路徑  --stm-port 路徑  --max-rpm N  --auto-rpm N
#
# 用 /dev/lora_gs 時會先停掉組員的 fucking-mower-lora-gs 服務（要 sudo），結束時自動開回來。
# 所有輸出合在這個畫面，前面標示是誰印的，也存在 ~/remote_lora/logs/。按一次 Ctrl+C 全部關掉。
# 組員的程式只執行、不修改（python3 -B 不會留下快取檔）。

set -m  # 每支程式自己一個 process group：Ctrl+C 由這支接，再一支一支叫它們正常結束

HERE=$(cd "$(dirname "$0")" && pwd)
MOWER=${MOWER:-$HOME/fucking_mower}
BRIDGE_DIR=$MOWER/rover2_ws/src/campusrover_base/ros2_stm_connect
GS_SERVICE=fucking-mower-lora-gs.service
FAKE_LINK=/tmp/fake-stm32

FAKE=0
LORA=/dev/lora_gs
STM=
NODE_ARGS=()
while [ $# -gt 0 ]; do
    case $1 in
        --fake) FAKE=1 ;;
        --lora-port) LORA=$2; shift ;;
        --stm-port) STM=$2; shift ;;
        --max-rpm|--auto-rpm) NODE_ARGS+=("$1" "$2"); shift ;;
        -h|--help) sed -n '2,10s/^# \{0,1\}//p' "$0"; exit 0 ;;
        *) echo "不認得的選項：$1（-h 看用法）"; exit 2 ;;
    esac
    shift
done

# ROS 環境：跟組員一樣用 CycloneDDS
if [ -z "$ROS_DISTRO" ]; then
    source /opt/ros/humble/setup.bash
fi
export RMW_IMPLEMENTATION=${RMW_IMPLEMENTATION:-rmw_cyclonedds_cpp}
if [ -z "$CYCLONEDDS_URI" ] && [ -f "$HOME/CycloneDDS.xml" ]; then
    export CYCLONEDDS_URI=file://$HOME/CycloneDDS.xml
fi
export PYTHONDONTWRITEBYTECODE=1
if [ $FAKE = 1 ]; then
    export ROS_DOMAIN_ID=77
    TCP=8790
    STM=$FAKE_LINK
else
    export ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-50}
    TCP=8772
    if [ -z "$STM" ]; then
        if [ -e /dev/stm32 ]; then STM=/dev/stm32; else STM=/dev/ttyACM0; fi
    fi
fi

PIDS=()
NAMES=()
GS_STOPPED=0
CLEANING=

stop_one() {
    local pid=$1 name=$2
    kill -0 "$pid" 2>/dev/null || return
    kill -INT "$pid"
    for _ in $(seq 50); do
        kill -0 "$pid" 2>/dev/null || return
        sleep 0.1
    done
    echo "$name 5 秒沒結束，強制關閉"
    kill -KILL "$pid"
}

cleanup() {
    [ -n "$CLEANING" ] && return
    CLEANING=1
    if [ ${#PIDS[@]} -gt 0 ]; then
        echo
        echo "關閉中…"
        # 反過來關：node 先送 0 → bridge → main 送最後的 0 給 STM32 → 假 STM32
        for (( i=${#PIDS[@]}-1; i>=0; i-- )); do
            stop_one "${PIDS[$i]}" "${NAMES[$i]}"
        done
    fi
    if [ $GS_STOPPED = 1 ]; then
        echo "把組員的 $GS_SERVICE 開回來…"
        if sudo systemctl start $GS_SERVICE; then
            echo "✓ 已開回來"
        else
            echo "✗ 開不回來，請手動執行：sudo systemctl start $GS_SERVICE"
        fi
    fi
    [ -n "$LOG_DIR" ] && echo "紀錄在 $LOG_DIR"
}
trap cleanup EXIT
trap 'exit 130' INT TERM HUP

fail() {
    echo "✗ $*"
    exit 1
}

holders() {  # 誰開著這個裝置（PID 和指令）
    local pids
    pids=$(fuser "$(readlink -f "$1")" 2>/dev/null | xargs)
    [ -n "$pids" ] && ps -o pid=,args= -p "${pids// /,}"
}

port_busy() {
    ss -ltn "( sport = :$1 )" | grep -q LISTEN
}

wait_until() {  # 秒數 PID 條件…：等到條件成立；程式先死掉就失敗
    local seconds=$1 pid=$2
    shift 2
    for _ in $(seq $((seconds * 10))); do
        "$@" && return 0
        kill -0 "$pid" 2>/dev/null || return 1
        sleep 0.1
    done
    return 1
}

start() {  # 名稱 資料夾 指令…：輸出加上 [名稱] 印在畫面，也寫進紀錄檔
    local name=$1 dir=$2
    shift 2
    # 背景程式一碰終端機就會被系統暫停，所以輸入接 /dev/null（假 STM32 的鍵盤功能在這裡不能用）
    (cd "$dir" && exec "$@") < /dev/null > >(trap '' INT; tee -a "$LOG_DIR/$name.log" | sed -u "s/^/[$name] /") 2>&1 &
    PIDS+=($!)
    NAMES+=("$name")
}

# ---- 開之前先檢查 ----
[ -d "$MOWER/main_control" ] || fail "找不到組員的 main_control：$MOWER/main_control"
[ -d "$BRIDGE_DIR" ] || fail "找不到 stm_bridge：$BRIDGE_DIR"
[ -e "$LORA" ] || fail "找不到 LoRa：$LORA（USB 有插嗎？）"
pgrep -f lora_ros_node.py > /dev/null && fail "lora_ros_node 已經在跑，先關掉"
port_busy $TCP && fail "TCP $TCP 已經有人在用：main_control 是不是已經在跑？"
if [ $FAKE = 1 ]; then
    pgrep -f pty_fake_stm32.py > /dev/null && fail "假 STM32 已經在跑，先關掉"
    [ -L "$FAKE_LINK" ] && rm -f "$FAKE_LINK"  # 上次沒關乾淨留下的
else
    [ -e "$STM" ] || fail "找不到 STM32：$STM（USB 有插嗎？）"
    owner=$(holders "$STM") && fail "STM32 $STM 被佔用，先關掉：
$owner"
    pgrep -f stm_bridge > /dev/null && fail "stm_bridge 已經在跑（組員的 mower_stm_bridge？），先關掉"
fi

# lora_gs 平常被組員的地面站服務佔著：先停，結束時開回來
if [ "$(readlink -f "$LORA")" = "$(readlink -f /dev/lora_gs 2>/dev/null)" ] \
        && systemctl is-active --quiet $GS_SERVICE; then
    echo "停掉組員的 $GS_SERVICE（結束時會開回來），可能要輸入 sudo 密碼："
    sudo systemctl stop $GS_SERVICE || fail "停不掉 $GS_SERVICE"
    GS_STOPPED=1
fi
owner=$(holders "$LORA") && fail "LoRa $LORA 被佔用，先關掉：
$owner"

LOG_DIR=$HOME/remote_lora/logs/$(date +%Y%m%d-%H%M%S)$([ $FAKE = 1 ] && echo -fake)
mkdir -p "$LOG_DIR"
echo "模式 $([ $FAKE = 1 ] && echo '假 STM32' || echo '真車')｜STM32 $STM｜LoRa $LORA｜ROS_DOMAIN_ID $ROS_DOMAIN_ID｜TCP $TCP"
echo "紀錄：$LOG_DIR"

# ---- 依序開 ----
if [ $FAKE = 1 ]; then
    start fake-stm32 "$HERE" python3 -B -u pty_fake_stm32.py --auto --link "$FAKE_LINK"
    wait_until 5 "${PIDS[-1]}" test -L "$FAKE_LINK" || fail "假 STM32 沒開起來，看 $LOG_DIR/fake-stm32.log"
fi
start main "$MOWER" python3 -B -u -m main_control --stm-port "$STM" --no-lora --no-auto-map --tcp-port $TCP
wait_until 15 "${PIDS[-1]}" port_busy $TCP || fail "main_control 沒開起來，看 $LOG_DIR/main.log"
start bridge "$BRIDGE_DIR" python3 -B -u -m ros2_stm_connect.stm_bridge --ros-args -p tcp_port:=$TCP
sleep 1
start node "$HERE" python3 -B -u lora_ros_node.py --lora-port "$LORA" "${NODE_ARGS[@]}"
echo "全部開好了。按 Ctrl+C 全部關掉。"

# 任何一支自己結束，就全部關掉（不留半套在跑）
wait -n
for i in "${!PIDS[@]}"; do
    kill -0 "${PIDS[$i]}" 2>/dev/null || echo "✗ ${NAMES[$i]} 自己結束了，全部關閉（看 $LOG_DIR/${NAMES[$i]}.log）"
done
exit 1
