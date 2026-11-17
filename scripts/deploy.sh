#!/bin/sh
# 部署到开发板：上传 bin/、models/、scripts/ 并在远端校验 md5，然后只重启
# stream_server（sub_img_imu 若在运行会继续运行，不触发 12 秒租约等待）。
#
# 用法:
#   sh scripts/deploy.sh [user@host]     # 默认 root@192.168.137.6
#
# 依赖本机 ssh/scp 已能免密登录板子。上传先写 *.new，校验通过才替换，
# 避免半截文件覆盖正在使用的程序。
#
# 在 WSL 等没有板子网段路由的环境里，用 SSH/SCP 环境变量指向 Windows OpenSSH，例如:
#   SSH="/mnt/c/Windows/System32/OpenSSH/ssh.exe -i C:\\Users\\HP\\.ssh\\active_key" \
#   SCP="/mnt/c/Windows/System32/OpenSSH/scp.exe -i C:\\Users\\HP\\.ssh\\active_key" \
#     sh scripts/deploy.sh root@192.168.137.6

set -e

HOST=${1:-root@192.168.137.6}
ROOT=$(cd -- "$(dirname -- "$0")/.." && pwd)
DST=${DEPLOY_DIR:-/userdata/iris}
SSH=${SSH:-ssh}
SCP=${SCP:-scp}
FILES="bin/stream_server bin/min_dirty_infer models/model.bin models/labels.txt scripts/start.sh scripts/stop.sh"

echo "部署 $ROOT -> $HOST:$DST"

for f in $FILES; do
    $SCP -q "$ROOT/$f" "$HOST:$DST/$f.new"
done

# 本地 md5 清单 -> 远端 md5sum -c 校验 *.new
manifest=$(mktemp)
trap 'rm -f "$manifest"' EXIT
for f in $FILES; do
    sum=$(md5sum "$ROOT/$f" | cut -d' ' -f1)
    echo "$sum  $DST/$f.new"
done > "$manifest"

$SSH "$HOST" "md5sum -c -" < "$manifest"

# 校验通过后替换并重启推流服务（不 pkill sub_img_imu）
$SSH "$HOST" "set -e
    pkill -x stream_server 2>/dev/null || true
    sleep 1
    for f in $FILES; do mv -f '$DST/'\$f.new '$DST/'\$f; done
    chmod 755 '$DST'/bin/* '$DST'/scripts/*.sh 2>/dev/null || true
    sh '$DST/scripts/start.sh'"

echo "部署完成: http://$HOST:8080/"
