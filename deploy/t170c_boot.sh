#!/bin/bash
# 开机拉起调度。已有进程则退出，避免再做一次 home。
# 先等到 USB 串口和 CAN 口出现，再启动。
# 只有这次启动还没打印 T170C_DEBUG_READY 就退出时才会再拉起，最多 3 次。
# 一旦进过调度，进程再退出也不拉起，避免任务中途再 home。
set -u
ROOT=/home/ti5robot/T170Clean
LOG="$ROOT/logs/t170c_boot.log"
BIN="$ROOT/build_robot/t170c_debug"
export PATH="/usr/bin:/bin"
export HOME="/home/ti5robot"
mkdir -p "$ROOT/logs"
cd "$ROOT"

log() {
    echo "$(date -Iseconds) $*" >> "$LOG"
}

if pgrep -f "$BIN" >/dev/null 2>&1; then
    log "已有 t170c_debug，跳过"
    exit 0
fi

child=0
forward_stop() {
    if [ "$child" -gt 0 ]; then
        kill -TERM "$child" 2>/dev/null || true
        wait "$child" 2>/dev/null || true
    fi
    exit 0
}
trap forward_stop TERM INT

serial_count() {
    if [ -d /dev/serial/by-path ]; then
        find /dev/serial/by-path -mindepth 1 -maxdepth 1 -type l 2>/dev/null | wc -l
    else
        echo 0
    fi
}

can_count() {
    find /sys/class/net -maxdepth 1 -name 'can*' 2>/dev/null | wc -l
}

speaker_up() {
    aplay -l 2>/dev/null | grep -q 'USB Audio Device'
}

# 这次重启里，cron 起来约 30 秒时串口、CAN、喇叭都还没有，扫电机直接失败。
# 等到至少 4 路串口和 can0，最多 3 分钟。喇叭不是启动条件。
for i in $(seq 1 90); do
    serial_n="$(serial_count)"
    can_n="$(can_count)"
    speaker=0
    if speaker_up; then
        speaker=1
    fi
    if [ "$serial_n" -ge 4 ] && [ "$can_n" -ge 1 ]; then
        log "设备就绪 serial=${serial_n} can=${can_n} speaker=${speaker}"
        break
    fi
    if [ "$i" -eq 1 ] || [ $((i % 15)) -eq 0 ]; then
        log "等待设备 serial=${serial_n} can=${can_n} speaker=${speaker}"
    fi
    if [ "$i" -eq 90 ]; then
        log "等了 180 秒设备仍不齐，仍尝试启动 serial=${serial_n} can=${can_n}"
    fi
    sleep 2
done
sleep 2

ready_after_mark() {
    awk -v mark="$1" '
        index($0, mark) { seen = 1; next }
        seen && index($0, "T170C_DEBUG_READY") { found = 1 }
        END { exit found ? 0 : 1 }
    ' "$LOG"
}

for attempt in 1 2 3; do
    mark="BOOTMARK attempt=${attempt} pid=$$ time=$(date +%s)"
    log "开机启动 --dispatch 第${attempt}次"
    echo "$mark" >> "$LOG"
    "$BIN" --dispatch >> "$LOG" 2>&1 &
    child=$!
    while kill -0 "$child" 2>/dev/null; do
        if ready_after_mark "$mark"; then
            log "已就绪，进入调度，不再重试"
            wait "$child"
            rc=$?
            child=0
            log "调度进程退出 code=${rc}，不再拉起"
            exit "$rc"
        fi
        sleep 1
    done
    wait "$child"
    rc=$?
    child=0
    log "尚未就绪就退出 code=${rc}"
    if [ "$attempt" -lt 3 ]; then
        sleep 5
    fi
done

log "三次都没进调度，停止"
exit 1
