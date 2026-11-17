#!/bin/sh
# 构建板端程序（aarch64），产物到 ../bin/。
#
# 用法:
#   sh scripts/build.sh                # 自动寻找交叉编译器
#   CROSS_CC=/path/to/gcc sh scripts/build.sh
#
# OE 容器里通常已有 /opt/arm-gnu-toolchain-*/bin/aarch64-none-linux-gnu-gcc。
# 原理就是 make -C <工程根目录>，编译器检测结果通过 CROSS_CC 传给 Makefile。

set -e

HERE=$(cd -- "$(dirname -- "$0")" && pwd)
ROOT=$(cd "$HERE/.." && pwd)

CC=${CROSS_CC:-}
if [ -z "$CC" ]; then
    for p in /opt/arm-gnu-toolchain-*/bin/aarch64-none-linux-gnu-gcc; do
        if [ -x "$p" ]; then
            CC=$p
            break
        fi
    done
fi
if [ -z "$CC" ] && command -v aarch64-linux-gnu-gcc >/dev/null 2>&1; then
    CC=$(command -v aarch64-linux-gnu-gcc)
fi
if [ -z "$CC" ]; then
    echo "错误: 找不到 aarch64 交叉编译器" >&2
    echo "  OE 容器里通常有 /opt/arm-gnu-toolchain-*/bin/aarch64-none-linux-gnu-gcc" >&2
    echo "  也可用 CROSS_CC=/path/to/aarch64-linux-gnu-gcc sh scripts/build.sh" >&2
    exit 1
fi

echo "CC: $CC"
make -C "$ROOT" CROSS_CC="$CC"

ls -l "$ROOT/bin"
if command -v readelf >/dev/null 2>&1; then
    echo "--- 依赖的动态库 ---"
    readelf -d "$ROOT/bin/stream_server" | grep NEEDED || true
    echo "--- 需要的 glibc 符号版本 ---"
    readelf -Ws "$ROOT/bin/stream_server" | grep -oE "GLIBC_[0-9.]+" | sort -Vu | tail -3 || true
fi
