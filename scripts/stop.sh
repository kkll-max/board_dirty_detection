#!/bin/sh
# 停止 start.sh 启动的全部进程，并清掉中间文件
# 用法: sh /userdata/iris/scripts/stop.sh

RUN=/userdata/iris/run
killed_subimg=0

stop_one() {
    name=$1
    f="$RUN/$name.pid"
    [ -f "$f" ] || return 0
    pid=$(cat "$f")
    if kill "$pid" 2>/dev/null; then
        echo "已停止 $name (pid $pid)"
        [ "$name" = "sub_img_imu" ] && killed_subimg=1
    else
        echo "$name (pid $pid) 未在运行"
    fi
    rm -f "$f"
}

stop_one stream
stop_one cleanup
stop_one sub_img_imu

# 兜底：pid 文件丢失时按命令行收掉本方案的进程（不会动 sensor / mower_*）
for p in $(ps -ef | grep -v grep | grep -E 'stream_server --watch /userdata/iris/temp|/goko/app/sub_img_imu|rm -f /userdata/iris/temp' | awk '{print $2}'); do
    if kill "$p" 2>/dev/null; then
        echo "兜底停止 pid $p"
        [ "$(cat /proc/$p/comm 2>/dev/null)" = "sub_img_imu" ] && killed_subimg=1
    fi
done

# 记录取流节点停止时刻：Cyber 的订阅者租约 12 秒，
# 租约没过期就重启新实例，新实例会在约 10 秒后收不到图。
[ "$killed_subimg" = "1" ] && date +%s >"$RUN/sub_img_stopped_at"

# 清掉残留的中间 PNG
rm -f /userdata/iris/temp/*/*.png
echo "已清空 /userdata/iris/temp"
