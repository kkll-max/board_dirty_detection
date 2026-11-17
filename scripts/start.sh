#!/bin/sh
# 启动取流 + MJPEG 网页推流（中间帧不落盘）
# 用法: sh /userdata/iris/scripts/start.sh
# 停止: sh /userdata/iris/scripts/stop.sh
# 网页: http://<板子IP>:8080/
#
# 实测结论：
# 1) 中间目录必须放内存盘。写到 eMMC 时 sub_img_imu 的存图队列追不上 10fps，
#    延迟会积到 10 秒；挂 tmpfs 后不再积压。
# 2) 默认 2 模式（双目），推左目画面 640x448。源要软编 3 路 PNG（左/右/视差）。
#    推流默认原分辨率（STREAM_PUB_WIDTH=0 = 不缩），网页工具栏的「推流宽度」
#    下拉框可随时切换原始/1282/960/640/480/320，改完下一帧立即生效。
#    带宽紧张时再缩：板子走 2.4G Wi-Fi（客户端互传 ping 0.3-0.7s、有效吞吐约
#    130 KB/s），并排原图 1282x448@q70 每帧约 90 KB、7.5 fps 产出 675 KB/s，
#    链路送不完就积压成几秒延迟；缩到 640 宽后每帧约 20 KB、编码耗时也从
#    ~50 ms 降到 ~13 ms。也可用 STREAM_QUALITY=55 降质量、STREAM_FPS=5 降帧率。
#    板端 Wi-Fi 省电也会拉高延迟，脚本会顺手关掉。
#    吃满一个 A55 核。用 taskset 把它绑到 2,4 号核、推流绑到 6 号核后，端到端
#    稳定在 0.2-0.35s（不绑核会因调度抖动在 0.9-1.4s）；实测只绑核即可，
#    不需要 nice，对割草机其它节点干扰最小。
#    备选 1 模式（单目前视 640x384）：CPU 约 41%：
#        SUB_IMG_MODE=1 sh /userdata/iris/scripts/start.sh
# 3) sub_img_imu 停掉后租约还在 12 秒，期间重启的新实例约 10 秒后会静默失联，
#    所以 start 会先等租约过期，见下面第 1 步。
# 4) 默认加载新模型 models/model.bin（6 类分割：clean / dirt_dot / glare /
#    water / grass_residue / mud_film），类名从模型同目录 labels.txt 读取。
#    clean(class0) 不叠加，其余脏污类按调色板 50% 叠加，所有脏污类合计占比
#    超过 0.5% 时显示 LENS DIRTY，结果同时出现在 /status.json 的
#    dirty / fg_ratio / cls_ratio / infer_ms 字段。
#    网页上的「是否开启脏污检测」按钮可随时开关：开启 = 每帧推理并叠加，
#    画面变为并排「左原图 | 右检测图」，两半各用一个蓝框标出 640x384 检测范围；
#    关闭 = 仅推流单幅原图（省 CPU/BPU）。
#    被判脏污的帧还会把「原图 | 掩码 | 叠加图」三联图存 JPG 到
#    /userdata/iris/dirty_imgs，默认最多保留 100 张（超出删最旧）。
#    对比图默认最多每 1 秒存一张（STREAM_SAVE_INTERVAL_MS，0 = 每脏污帧都存）：
#    三联图编码 + 写 eMMC 约 50~80 ms，脏污持续时逐帧保存会把推流卡到 5 fps
#    以下、网页延迟明显变大。
#    关闭推理：STREAM_MODEL= sh /userdata/iris/scripts/start.sh
#    换模型：  STREAM_MODEL=/path/model.bin sh /userdata/iris/scripts/start.sh
#    换标签：  STREAM_LABELS=/path/labels.txt sh /userdata/iris/scripts/start.sh
#    改目录：  STREAM_DIRTY_DIR=/path sh ...   # 默认 /userdata/iris/dirty_imgs
#    改上限：  STREAM_DIRTY_MAX=20 sh ...      # 0 = 不保存对比图
#    删掉已存的对比图：rm -f /userdata/iris/dirty_imgs/dirty_*.jpg
# 5) 录制模式默认关闭，可选 -a / -b（数字为最多保留的帧数，默认 100，
#    超出删最旧）：
#      STREAM_RECORD=a       每帧都存原图+检测图
#      STREAM_RECORD=b       只存被判脏污的帧（用于回看干净镜头下的误检）
#    对应命令行 -a [N] / -b [N]。文件为 orig_<帧名>.jpg / det_<帧名>.jpg，
#    存到 /userdata/iris/on_time；每帧两张、约 7fps 时盘占用随上限恒定。
#    改目录：STREAM_RECORD_DIR=/path
#    改上限：STREAM_RECORD_MAX=200（默认 100；0 = 不保存）
#    停止录制：不传 STREAM_RECORD，或执行 stop.sh
# 6) 图片采集：网页上的「保存 / 保存至 / 新建」把当前原始画面存成 JPG，
#    根目录 /userdata/iris/data_get，选择的位置记在 data_get/.selected。
# 7) 单图/文件夹耗时：bin/min_dirty_infer -i <图片或目录> [-o <输出>]
#    默认模型 /userdata/iris/models/model.bin，labels 取模型同目录 labels.txt。

BASE=/userdata/iris
RUN=$BASE/run
TMP=$BASE/temp
LOG=$BASE/logs
mkdir -p "$RUN" "$TMP" "$LOG"
# 固定工作目录：sub_img_imu 会在 CWD 下建 dumps/ 写 bvar 统计，
# 从只读的 /root 启动会每 10 秒报一次 Fail to create directory。
cd "$BASE" || exit 1

# glog 每次启动都会新建一个 sub_img_imu.log.INFO.日期.pid，只保留最新
# LOG_KEEP 个（默认 5），有新的就删最旧的；其余日志一律放 logs/ 下。
LOG_KEEP=${LOG_KEEP:-5}
case "$LOG_KEEP" in ''|*[!0-9]*) LOG_KEEP=5 ;; esac
[ "$LOG_KEEP" -lt 1 ] && LOG_KEEP=1
export LOG_KEEP

# 兼容旧版本：把散落在 iris 根目录的 glog 文件收进 logs/
if ls "$BASE"/sub_img_imu.log.INFO.* >/dev/null 2>&1; then
    mv -f "$BASE"/sub_img_imu.log.INFO.* "$LOG"/ 2>/dev/null
fi
if [ -L "$BASE/sub_img_imu.INFO" ]; then
    mv -f "$BASE/sub_img_imu.INFO" "$LOG"/ 2>/dev/null
fi

prune_logs() {
    n=$(ls -1 "$LOG"/sub_img_imu.log.INFO.* 2>/dev/null | wc -l)
    while [ "${n:-0}" -gt "$LOG_KEEP" ]; do
        old=$(ls -1t "$LOG"/sub_img_imu.log.INFO.* 2>/dev/null | tail -n 1)
        [ -n "$old" ] || break
        rm -f "$old"
        n=$((n - 1))
    done
}

# sub_img_imu 模式：2 = 双目（默认），1 = 单目。可用 SUB_IMG_MODE 覆盖。
MODE=${SUB_IMG_MODE:-2}
if [ "$MODE" = "1" ]; then
    CH=mono_left
else
    CH=stereo_left
fi

# 脏污检测模型：默认 iris/models/model.bin；STREAM_MODEL= 置空则只推流不推理
MODEL=${STREAM_MODEL-$BASE/models/model.bin}
if [ -n "$MODEL" ] && [ ! -f "$MODEL" ]; then
    echo "警告: 模型不存在，本次不开启脏污检测: $MODEL"
    MODEL=
fi
INFER_OPT=
if [ -n "$MODEL" ]; then
    echo "模型: $MODEL"
    INFER_OPT="--model $MODEL"
    # 类别名文件：默认用模型同目录的 labels.txt（stream_server 也会自动找）
    LABELS=${STREAM_LABELS-$BASE/models/labels.txt}
    if [ -n "$LABELS" ] && [ -f "$LABELS" ]; then
        INFER_OPT="$INFER_OPT --labels $LABELS"
        echo "标签: $LABELS"
    fi
    INF_CN=开启
else
    INF_CN=关闭
fi
# 脏污判定阈值（所有脏污类面积占比 %），默认用程序内置的 0.5%
DIRTY_OPT=
[ -n "${STREAM_DIRTY_RATIO:-}" ] && DIRTY_OPT="--dirty-ratio $STREAM_DIRTY_RATIO"

# 脏污对比图（原图|可视化 并排）：默认 $BASE/dirty_imgs，最多 100 张
SAVE_OPT=
SAVE_CN=关闭
if [ -n "$MODEL" ]; then
    SAVE_DIR=${STREAM_DIRTY_DIR-$BASE/dirty_imgs}
    SAVE_MAX=${STREAM_DIRTY_MAX:-100}
    case "$SAVE_MAX" in
        ''|*[!0-9]*) SAVE_MAX=100 ;;
    esac
    # 对比图限速：默认 1000 ms 一张，0 = 每脏污帧都存
    SAVE_INT=${STREAM_SAVE_INTERVAL_MS:-1000}
    case "$SAVE_INT" in
        ''|*[!0-9]*) SAVE_INT=1000 ;;
    esac
    SAVE_OPT="--save-dir $SAVE_DIR --save-max $SAVE_MAX --save-interval $SAVE_INT"
    [ "$SAVE_MAX" -gt 0 ] &&
        SAVE_CN="$SAVE_MAX 张 -> $SAVE_DIR（间隔 ${SAVE_INT} ms，0 = 不限）"
fi

# 录制模式：STREAM_RECORD=a（每帧）/ b（仅脏污帧），默认关闭
REC_OPT=
REC_CN=关闭
RMODE=
case "${STREAM_RECORD:-0}" in
    0|'') ;;
    a|A|all|1) RMODE=a ;;
    b|B|dirty|2) RMODE=b ;;
    *) echo "警告: STREAM_RECORD=${STREAM_RECORD} 无效（用 a 或 b），本次不录制"
       RMODE= ;;
esac
if [ -n "${RMODE:-}" ]; then
    if [ -z "$MODEL" ]; then
        echo "警告: 未开启脏污检测模型，忽略录制模式"
        RMODE=
    else
        REC_DIR=${STREAM_RECORD_DIR-$BASE/on_time}
        REC_MAX=${STREAM_RECORD_MAX:-100}
        case "$REC_MAX" in
            ''|*[!0-9]*) REC_MAX=100 ;;
        esac
        REC_OPT="-$RMODE $REC_MAX --record-dir $REC_DIR"
        if [ "$RMODE" = "b" ]; then
            REC_CN="-b 仅脏污帧，保留最新 $REC_MAX 帧 -> $REC_DIR"
        else
            REC_CN="-a 每帧，保留最新 $REC_MAX 帧 -> $REC_DIR"
        fi
    fi
fi

# 中间文件放内存盘（重启后自动失效，这里每次启动时补挂）
if ! mount | grep -q " on $TMP type tmpfs"; then
    mount -t tmpfs -o size=64m tmpfs "$TMP" && echo "已挂载 tmpfs: $TMP"
fi
mkdir -p "$TMP/stereo_left" "$TMP/stereo_right" "$TMP/stereo_disp" "$TMP/mono_left" "$TMP/mono_right"

# sub_img_imu 的输出路径写死，用软链接指到 temp
IMG=/userdata/mono_orig/image_data
mkdir -p /userdata/mono_orig
if [ -L "$IMG" ]; then
    : # 已是软链
elif [ ! -e "$IMG" ]; then
    ln -s "$TMP" "$IMG"
elif [ -d "$IMG" ] && [ -z "$(ls -A "$IMG" 2>/dev/null)" ]; then
    rmdir "$IMG" && ln -s "$TMP" "$IMG"   # 空目录，直接换成软链
else
    # 真实目录会让 PNG 落到 eMMC、推流目录却收不到图，必须先处理
    echo "错误：$IMG 是真实目录（不是软链），sub_img_imu 会把帧写到 eMMC 而推流收不到。"
    echo "      确认内容后删除再重试：rm -rf $IMG"
    exit 1
fi

export LD_LIBRARY_PATH=/goko/lib:/app/lib:/app/pub/lib:/middleware/lib:/middleware/pub/lib:/usr/hobot/lib:/usr/hobot/lib/sensor:/system/lib:/system/usr/lib:/lib
export CYBER_PATH=/goko/config/cyber_rt/

alive() { [ -f "$1" ] && kill -0 "$(cat "$1")" 2>/dev/null; }

# 1) 取流节点（Cyber -> PNG，双目约 10fps）
if ! alive "$RUN/sub_img_imu.pid"; then
    # Cyber 订阅者租约是 12 秒：刚停掉的旧实例租约没过期就起新的，
    # 新实例约 10 秒后会静默收不到图。stop 脚本会记录停止时刻。
    if [ -f "$RUN/sub_img_stopped_at" ]; then
        wait=$((15 - $(date +%s) + $(cat "$RUN/sub_img_stopped_at")))
        if [ "$wait" -gt 0 ]; then
            echo "等待旧订阅者租约过期 ${wait}s ..."
            sleep "$wait"
        fi
        rm -f "$RUN/sub_img_stopped_at"
    fi
    # 绑到 2,4 号核：避开 CPU0 上的相机/系统负载，避免跨核迁移抖动
    # GLOG_log_dir 让 glog 的 sub_img_imu.log.INFO.* 也落到 logs/ 下
    prune_logs
    GLOG_log_dir="$LOG" nohup taskset -c 2,4 /goko/app/sub_img_imu "$MODE" \
        </dev/null >"$LOG/sub_img.log" 2>&1 &
    echo $! >"$RUN/sub_img_imu.pid"
fi

# 2) 右目/视差没人消费，定期清掉；每分钟顺便把超量的 glog 删到只剩 LOG_KEEP 个
if ! alive "$RUN/cleanup.pid"; then
    nohup sh -c 'i=0; while :; do
        rm -f /userdata/iris/temp/stereo_right/*.png \
              /userdata/iris/temp/stereo_disp/*.png \
              /userdata/iris/temp/mono_right/*.png
        i=$((i + 1))
        if [ $((i % 60)) -eq 0 ]; then
            n=$(ls -1 /userdata/iris/logs/sub_img_imu.log.INFO.* 2>/dev/null | wc -l)
            while [ "${n:-0}" -gt "${LOG_KEEP:-5}" ]; do
                old=$(ls -1t /userdata/iris/logs/sub_img_imu.log.INFO.* 2>/dev/null | tail -n 1)
                [ -n "$old" ] || break
                rm -f "$old"
                n=$((n - 1))
            done
        fi
        sleep 1
    done' </dev/null >/dev/null 2>&1 &
    echo $! >"$RUN/cleanup.pid"
fi

# 3) 推流（左目，推完即删）
# Wi-Fi 省电会把 ping 拉到 1s、吞吐砍半，启动时关掉（重启后需重跑 start.sh）
if command -v iwconfig >/dev/null 2>&1; then
    iwconfig wlan0 power off >/dev/null 2>&1 && echo "Wi-Fi 省电: 已关闭"
fi
# 推流画面宽度上限（0 = 原始分辨率，也是默认；网页下拉框可随时改）
PUB_W=${STREAM_PUB_WIDTH:-0}
case "$PUB_W" in ''|*[!0-9]*) PUB_W=0 ;; esac
QUAL=${STREAM_QUALITY:-70}
case "$QUAL" in ''|*[!0-9]*) QUAL=70 ;; esac
FPS=${STREAM_FPS:-10}
case "$FPS" in ''|*[!0-9]*) FPS=10 ;; esac
if ! alive "$RUN/stream.pid"; then
    cd "$BASE" || exit 1
    # 绑到 6,7 号核：与取流节点分开；网页/保存等线程也能落到第 7 核，
    # 避免点「保存」时 q95 编码抢走推流线程的 CPU 造成画面卡顿
    nohup taskset -c 6,7 $BASE/bin/stream_server --watch "$TMP/$CH" --consume --purge \
        --port 8080 --pub-width "$PUB_W" --quality "$QUAL" --fps "$FPS" \
        $INFER_OPT $DIRTY_OPT $SAVE_OPT $REC_OPT </dev/null >"$BASE/logs/stream.log" 2>&1 &
    echo $! >"$RUN/stream.pid"
fi

sleep 2
# glog 新文件已生成，再剪一次，保证启动后立即不超过 LOG_KEEP 个
prune_logs
echo "sub_img_imu pid: $(cat "$RUN/sub_img_imu.pid" 2>/dev/null)  (mode $MODE, 推 $CH, 脏污检测: $INF_CN)"
echo "推流画面  : 宽<=$PUB_W（0=不缩）, JPEG q$QUAL, fps<=$FPS"
echo "脏污对比图: $SAVE_CN"
echo "逐帧录制  : $REC_CN"
echo "cleanup     pid: $(cat "$RUN/cleanup.pid" 2>/dev/null)"
echo "stream      pid: $(cat "$RUN/stream.pid" 2>/dev/null)"
echo -n "status: "
wget -qO- http://127.0.0.1:8080/status.json 2>/dev/null || echo "未就绪（看 $BASE/logs/stream.log）"
echo
echo "浏览器打开: http://<板子IP>:8080/"
