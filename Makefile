# 交叉编译 goko（RDK X5, aarch64, glibc ≤ 2.34）板端程序
#
# 用法:
#   make                                  # 构建 bin/stream_server、bin/min_dirty_infer
#   make clean
#   CROSS_CC=/path/aarch64-none-linux-gnu-gcc make
#
# 在 OE 容器 openexplorer/ai_toolchain_ubuntu_20_x5_gpu:v1.2.8-py310 里直接 make；
# 该镜像自带 /opt/arm-gnu-toolchain-*/bin/aarch64-none-linux-gnu-gcc。

CROSS_CC ?= $(firstword $(wildcard /opt/arm-gnu-toolchain-*/bin/aarch64-none-linux-gnu-gcc))
ifeq ($(strip $(CROSS_CC)),)
CROSS_CC := aarch64-linux-gnu-gcc
endif
CC := $(CROSS_CC)

CFLAGS  := -O2 -std=c11 -Wall -Wextra -pthread -Ithird_party \
           -ffunction-sections -fdata-sections
LDFLAGS := -Wl,--gc-sections -static-libgcc -s
LDLIBS  := -ldl -lm -pthread

SRC_DIR := src
BIN_DIR := bin
PROGS   := $(BIN_DIR)/stream_server $(BIN_DIR)/min_dirty_infer

.PHONY: all clean
all: $(PROGS)

$(BIN_DIR)/%: $(SRC_DIR)/%.c | $(BIN_DIR)
	$(CC) $(CFLAGS) $(LDFLAGS) -o $@ $< $(LDLIBS)

$(BIN_DIR):
	mkdir -p $@

clean:
	rm -f $(PROGS)
