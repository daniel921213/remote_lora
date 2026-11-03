#!/usr/bin/env bash
# 一個指令開好遙控整條路：分流程式 → main_control → stm_bridge → lora_ros_node（--fake 時先開假 STM32）。
#
#   ./start_remote.sh            真車：STM32 /dev/stm32（沒有就用 /dev/ttyACM0），LoRa 跟地面站共用
#   ./start_remote.sh --fake     不接車：假 STM32、ROS_DOMAIN_ID=77、TCP 8790，碰不到正在用的系統
#   ./start_remote.sh --stop     上次沒關乾淨：關掉留下來的程式，把組員的 gs 服務開回來
#   ./start_remote.sh --lora-port /dev/lora_gs   不共用：停掉組員的 gs 服務，遙控自己開 LoRa（結束時開回來）
#   其他選項：--stm-port 路徑  --max-rpm N  --auto-rpm N
#
# 共用（預設）：組員的 gs 服務照常開著 LoRa；分流程式（8787～8789）把遙控的句子給 node，地面站的封包給 main_control。
# 所有輸出合在這個畫面，前面標示是誰印的，也存在 ~/remote_lora/logs/。按一次 Ctrl+C 全部關掉。
# 組員的程式只執行、不修改（python3 -B 不會留下快取檔）。

set -m  # 每支程式自己一個 process group：Ctrl+C 由這支接，再一支一支叫它們正常結束

HERE=$(cd "$(dirname "$0")" && pwd)
COMMON=$(cd "$HERE/../common" && pwd)
MOWER=${MOWER:-$HOME/fucking_mower}
BRIDGE_DIR=$MOWER/rover2_ws/src/campusrover_base/ros2_stm_connect
GS_SERVICE=fucking-mower-lora-gs.service
GS_API_PORT=8767   # 組員 gs 服務的 API；資料埠是 8768
GS_DATA_PORT=8768
MUX_API_PORT=8787  # 分流程式：給 main_control 的 API，資料埠是 8788
MUX_REMOTE_PORT=8789
FAKE_LINK=/tmp/fake-stm32

FAKE=0
STOP=0
LORA=  # 空的：跟地面站共用
STM=
NODE_ARGS=()
while [ $# -gt 0 ]; do
    case $1 in
        --fake) FAKE=1 ;;
        --lora-port) LORA=$2; shift ;;
        --stm-port) STM=$2; shift ;;
        --max-rpm|--auto-rpm) NODE_ARGS+=("$1" "$2"); shift ;;
        --stop) STOP=1 ;;
        -h|--help) sed -n '2,12s/^# \{0,1\}//p' "$0"; exit 0 ;;
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

start_gs() {  # 剛輸入過 sudo 密碼就不用再問（終端機斷了也開得回來）
    sudo -n systemctl start $GS_SERVICE 2> /dev/null || sudo systemctl start $GS_SERVICE
}

cleanup() {
    [ -n "$CLEANING" ] && return
    CLEANING=1
    trap '' INT TERM HUP  # 關到一半再按 Ctrl+C 或斷線，也要關完
    if [ ${#PIDS[@]} -gt 0 ]; then
        echo
        echo "關閉中…（不用再按，等它關完）"
        # 反過來關：node 先送 0 → bridge → main 送最後的 0 給 STM32 → 分流程式 → 假 STM32
        for (( i=${#PIDS[@]}-1; i>=0; i-- )); do
            stop_one "${PIDS[$i]}" "${NAMES[$i]}"
        done
    fi
    if [ $GS_STOPPED = 1 ]; then
        echo "把組員的 $GS_SERVICE 開回來…"
        if start_gs; then
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

mux_connected() {  # 分流程式已經連上組員的 LoRa 程式
    python3 -c "import json, sys, urllib.request
s = json.load(urllib.request.urlopen('http://127.0.0.1:$MUX_API_PORT/status', timeout=1))
sys.exit(0 if s['mux']['upstream_connected'] else 1)" 2>/dev/null
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

# 這支開的程式，指令要從頭完全對上（編輯器、grep、組員平常不加 -B -u 開的都不算），照關閉順序排
OURS=('^python3 -B -u lora_ros_node\.py' '^python3 -B -u -m ros2_stm_connect\.stm_bridge'
      '^python3 -B -u -m main_control' '^python3 -B -u lora_mux\.py' '^python3 -B -u pty_fake_stm32\.py')

ours_running() {
    local pattern
    for pattern in "${OURS[@]}"; do pgrep -f "$pattern"; done | xargs
}

stop_leftovers() {
    local pattern pid found=
    for pattern in "${OURS[@]}"; do
        for pid in $(pgrep -f "$pattern"); do
            [ -z "$found" ] && echo "關掉上次留下來的程式：" && found=1
            ps -o pid=,args= -p "$pid"
            stop_one "$pid" "$pid"
        done
    done
    [ -z "$found" ] && echo "沒有留下來的程式"
    [ -L "$FAKE_LINK" ] && rm -f "$FAKE_LINK"
    if systemctl is-enabled --quiet $GS_SERVICE 2> /dev/null && ! systemctl is-active --quiet $GS_SERVICE; then
        echo "組員的 $GS_SERVICE 沒在跑，開回來："
        start_gs && echo "✓ 已開回來"
    fi
}

if [ $STOP = 1 ]; then
    stop_leftovers
    exit 0
fi

# ---- 開之前先檢查 ----
[ -d "$MOWER/main_control" ] || fail "找不到組員的 main_control：$MOWER/main_control"
[ -d "$BRIDGE_DIR" ] || fail "找不到 stm_bridge：$BRIDGE_DIR"
running=$(ours_running)
[ -n "$running" ] && fail "上次沒關乾淨，還有這些在跑：
$(ps -o pid=,args= -p "${running// /,}")
先執行：./start_remote.sh --stop"
port_busy $TCP && fail "TCP $TCP 已經有人在用（組員的 main_control 在跑？）：
$(ss -ltnp "( sport = :$TCP )" | tail -n +2)"
if [ $FAKE = 1 ]; then
    [ -L "$FAKE_LINK" ] && rm -f "$FAKE_LINK"  # 上次沒關乾淨留下的
else
    [ -e "$STM" ] || fail "找不到 STM32：$STM（USB 有插嗎？）"
    owner=$(holders "$STM") && fail "STM32 $STM 被佔用，先關掉：
$owner"
    pgrep -f stm_bridge > /dev/null && fail "stm_bridge 已經在跑（組員的 mower_stm_bridge？），先關掉"
fi

if [ -z "$LORA" ]; then
    # 共用：組員的 gs 服務要開著，而且它的資料埠要空著給分流程式
    port_busy $GS_API_PORT || fail "組員的 LoRa 程式沒在跑（127.0.0.1:$GS_API_PORT 沒人）。
先開：sudo systemctl start $GS_SERVICE
這台沒有裝 gs 服務的話，改用：./start_remote.sh --lora-port /dev/lora_gs"
    clients=$(ss -tnp state established "( dport = :$GS_DATA_PORT )" | tail -n +2)
    [ -n "$clients" ] && fail "已經有程式連著組員 LoRa 的資料埠 $GS_DATA_PORT（組員的 main_control？），分流程式連不上：
$clients"
    for port in $MUX_API_PORT $((MUX_API_PORT + 1)) $MUX_REMOTE_PORT; do
        port_busy $port && fail "TCP $port 已經有人在用（分流程式要用）：
$(ss -ltnp "( sport = :$port )" | tail -n +2)"
    done
    NODE_LORA=socket://127.0.0.1:$MUX_REMOTE_PORT
    MAIN_LORA=(--bridge-api "http://127.0.0.1:$MUX_API_PORT")
    LORA_TEXT="跟地面站共用（分流程式 $MUX_API_PORT）"
else
    [ -e "$LORA" ] || fail "找不到 LoRa：$LORA（USB 有插嗎？）"
    # lora_gs 平常被組員的地面站服務佔著：先停，結束時開回來
    if [ "$(readlink -f "$LORA")" = "$(readlink -f /dev/lora_gs 2>/dev/null)" ] \
            && systemctl is-active --quiet $GS_SERVICE; then
        echo "停掉組員的 $GS_SERVICE（結束時會開回來），可能要輸入 sudo 密碼："
        sudo systemctl stop $GS_SERVICE || fail "停不掉 $GS_SERVICE"
        GS_STOPPED=1
    fi
    owner=$(holders "$LORA") && fail "LoRa $LORA 被佔用，先關掉：
$owner"
    NODE_LORA=$LORA
    MAIN_LORA=(--no-lora)
    LORA_TEXT="遙控獨占 $LORA"
fi

LOG_DIR=$HOME/remote_lora/logs/$(date +%Y%m%d-%H%M%S)$([ $FAKE = 1 ] && echo -fake)
mkdir -p "$LOG_DIR"
echo "模式 $([ $FAKE = 1 ] && echo '假 STM32' || echo '真車')｜STM32 $STM｜LoRa $LORA_TEXT｜ROS_DOMAIN_ID $ROS_DOMAIN_ID｜TCP $TCP"
echo "紀錄：$LOG_DIR"

# ---- 依序開 ----
if [ $FAKE = 1 ]; then
    start fake-stm32 "$HERE" python3 -B -u pty_fake_stm32.py --auto --link "$FAKE_LINK"
    wait_until 5 "${PIDS[-1]}" test -L "$FAKE_LINK" || fail "假 STM32 沒開起來，看 $LOG_DIR/fake-stm32.log"
fi
if [ -z "$LORA" ]; then
    start mux "$COMMON" python3 -B -u lora_mux.py orin --upstream-api "http://127.0.0.1:$GS_API_PORT" \
        --api-port $MUX_API_PORT --remote-port $MUX_REMOTE_PORT
    wait_until 10 "${PIDS[-1]}" mux_connected || fail "分流程式連不上組員的 LoRa 程式，看 $LOG_DIR/mux.log"
fi
start main "$MOWER" python3 -B -u -m main_control --stm-port "$STM" "${MAIN_LORA[@]}" --no-auto-map --tcp-port $TCP
wait_until 15 "${PIDS[-1]}" port_busy $TCP || fail "main_control 沒開起來，看 $LOG_DIR/main.log"
start bridge "$BRIDGE_DIR" python3 -B -u -m ros2_stm_connect.stm_bridge --ros-args -p tcp_port:=$TCP
sleep 1
start node "$HERE" python3 -B -u lora_ros_node.py --lora-port "$NODE_LORA" "${NODE_ARGS[@]}"
echo "全部開好了。按 Ctrl+C 全部關掉。"

# 任何一支自己結束，就全部關掉（不留半套在跑）
wait -n
for i in "${!PIDS[@]}"; do
    kill -0 "${PIDS[$i]}" 2>/dev/null || echo "✗ ${NAMES[$i]} 自己結束了，全部關閉（看 $LOG_DIR/${NAMES[$i]}.log）"
done
exit 1
