# board_dirty_detection — 镜面脏污检测（量化 + 板端）

割草机镜头脏污检测的完整交付：BPU 6 类分割模型的量化工程（`05/`），以及
goko（RDK X5，Buildroot 2022.08）上纯 C 的推流 / 检测 / 网页程序。

板子是 Buildroot 2022.08，没有 Python / gcc / 包管理器。板端程序全部是交叉编译的
aarch64 纯 C：图片编解码用 stb，推理走板端 `/usr/lib/libdnn.so`（hbDNN C API），
不依赖 OpenCV / numpy。

## 目录

```text
board_dirty_detection/
├── Makefile                  交叉编译入口（make / make clean）
├── src/
│   ├── stream_server.c       推流 + 网页 + 实时脏污检测
│   └── min_dirty_infer.c     单图/文件夹耗时统计
├── third_party/              stb_image / stb_image_write / hb_dnn / hb_sys 头文件
├── scripts/
│   ├── build.sh              自动找交叉编译器并 make（OE 容器里用）
│   ├── deploy.sh             scp 到板子、md5 校验后重启 stream_server
│   ├── start.sh              板端一键启动
│   └── stop.sh               板端一键停止
├── models/
│   ├── model.bin             当前 6 类分割模型（05 版，md5 835338e1…，2,361,314 B）
│   └── labels.txt            clean / dirt_dot / glare / water / grass_residue / mud_film
├── bin/
│   ├── stream_server         已交叉编译的 aarch64 产物
│   └── min_dirty_infer       已交叉编译的 aarch64 产物
├── 05/                       模型量化工程（makertbin 的输入与产物）
│   ├── best.onnx  config.yaml  labels.txt
│   ├── cal_img/              校准图 194 张 PNG（98 MB）
│   ├── cal_raw/              校准图对应的 float32 二进制（同名 .raw，546 MB，已随仓库提交）
│   ├── output/               量化产物：model.bin、*_quant_info.json、各阶段 onnx、subgraph 图
│   ├── .hb_check/            makertbin 的中间检查产物（27 MB，随仓库保留）
│   └── hb_mapper_{checker,makertbin}.log
└── README.md
```

- 板端部署位置：`bin/*` → `/userdata/iris/bin/`，`models/*` → `/userdata/iris/models/`，
  `scripts/{start,stop}.sh` → `/userdata/iris/scripts/`；
- 一键部署：`sh scripts/deploy.sh root@<板子IP>`（脚本默认 `root@192.168.137.6`，
  当前板子是 `root@10.0.0.23`；上传为 `*.new`、远端 md5 校验通过后才替换，
  只重启 `stream_server`）；WSL 里没有板子网段路由、或私钥只在 Windows 上时，
  用 `SSH=`/`SCP=` 环境变量指向 Windows OpenSSH（示例见 `scripts/deploy.sh`
  头部注释）；
- 量化产物升级后：把 `05/output/model.bin` 覆盖到 `models/model.bin`，再跑
  `deploy.sh` 同步到板端（`deploy.sh` 会把模型一起上传）；
- 仓库里的 `bin/*` 为已交叉编译的 aarch64 产物（板端现运行的版本即由本工程源码
  编译）。md5 会随编译时目录变化：stb 头文件断言把 `__FILE__` 绝对路径嵌入了
  二进制，功能不受影响。

## 管线一览

```text
best.onnx ──hb_mapper makertbin──> output/model.bin ──scp──> /userdata/iris/models/model.bin
                                                              │
bin/stream_server（交叉编译的 aarch64 纯 C） ─── dlopen libdnn.so ──┘
     ▲
     └── /goko/app/sub_img_imu 2 写出的 PNG（640x448 双目左目）→ 检测 → MJPEG 网页
```

## 快速上手

```sh
# 1) 重新量化（可选，OE 容器；cal_raw 已随仓库提供，直接喂给 makertbin）
cd 05
hb_mapper makertbin config.yaml --model-type onnx

# 2) 交叉编译板端程序（OE 容器）
cd ..
sh scripts/build.sh                          # 产物在 bin/

# 3) 部署到板子并重启推流（默认 root@192.168.137.6，当前板子 10.0.0.23）
sh scripts/deploy.sh root@<板子IP>
```

浏览器打开 `http://<板子IP>:8080/`。

## 板端目录（`/userdata/iris`）

```text
/userdata/iris/
├── bin/
│   ├── stream_server       推流 + 网页（采集、脏污检测开关）
│   └── min_dirty_infer     单图/文件夹耗时统计
├── models/
│   ├── model.bin           6 类分割模型（BPU）
│   └── labels.txt          clean / dirt_dot / glare / water / grass_residue / mud_film
├── scripts/
│   ├── start.sh            一键启动（取流 + tmpfs + 推流网页）
│   └── stop.sh             一键停止并清理中间文件
├── logs/                   stream.log、sub_img.log、sub_img_imu.log.INFO.*
│                           （glog 每次启动新建一个，只保留最新 5 个）
├── run/                    *.pid、租约等待标记
├── temp/                   中间 PNG（start.sh 自动挂 64 MB tmpfs，推完即删）
├── data_get/               网页「保存」的数据集根目录（.selected 记录当前文件夹）
├── dirty_imgs/             脏污帧对比图（滚动保留最新 100 张）
└── dumps/                  sub_img_imu 的 bvar 统计（vendor 自动生成，可删）
```

## 启动 / 停止

```sh
sh /userdata/iris/scripts/start.sh     # 启动
sh /userdata/iris/scripts/stop.sh      # 停止
```

`start.sh` 依次做四件事：

1. 把 `temp/` 挂成 64 MB tmpfs（必须，否则 eMMC 写入跟不上会积压延迟）；
2. 拉起 `/goko/app/sub_img_imu 2`（双目，`taskset -c 2,4`）；
3. 拉起清理循环，每秒删掉没人消费的右目/视差 PNG；
4. 拉起 `bin/stream_server`（`taskset -c 6,7`）：

```sh
bin/stream_server --watch temp/stereo_left --consume --purge --port 8080 \
    --pub-width 0 --quality 70 --fps 10 \
    --model models/model.bin --labels models/labels.txt \
    --save-dir dirty_imgs --save-max 100 --save-interval 1000
```

pid 在 `run/*.pid`。停止后不要立刻重启：Cyber 订阅者租约 12 秒，`start.sh` 会
自动等待并打印 `等待旧订阅者租约过期 Ns ...`，属正常现象。

### 常用环境变量（不改脚本即可调整）

| 变量 | 作用 |
| --- | --- |
| `SUB_IMG_MODE=1` | 单目前视 640×384（默认 2 = 双目左目 640×448） |
| `STREAM_PUB_WIDTH=0` | 启动时的推流画面宽度上限，默认 `0` = 原始分辨率；网页「推流宽度」下拉框可随时改（`/res?w=N`）。并排原图 1282×448 每帧约 90 KB，缩到 640 宽约 20 KB、编码耗时从 ~50 ms 降到 ~13 ms |
| `STREAM_QUALITY=70` | 推流 JPEG 质量（数据集「保存」仍是 q95 原图） |
| `STREAM_FPS=10` | 推流帧率上限（0 = 不限） |
| `STREAM_MODEL=` | 置空 = 只推流不推理；也可给其它 `.bin` 路径 |
| `STREAM_LABELS=` | 类别名文件（默认模型同目录 `labels.txt`） |
| `STREAM_DIRTY_RATIO=1.0` | 脏污判定阈值（所有脏污类合计占比 %，默认 0.5） |
| `STREAM_REGION_MIN_PX=16` | 连通域最小面积（模型像素），更小的当噪点丢掉（默认 16） |
| `STREAM_DIRTY_MAX=20` | 对比图保留张数（0 = 不保存，默认 100） |
| `STREAM_DIRTY_DIR=...` | 对比图目录（默认 `dirty_imgs`） |
| `STREAM_SAVE_INTERVAL_MS=1000` | 两张对比图的最小间隔（0 = 每脏污帧都存，默认 1000） |
| `STREAM_RECORD=a` / `b` | 逐帧录制：`a` 每帧、`b` 仅脏污帧（默认关） |
| `STREAM_RECORD_MAX=200` | 录制保留帧数（默认 100） |
| `LOG_KEEP=5` | `logs/` 下 glog 日志保留个数，超出删最旧的 |

## 网页功能

顶部按钮：

- **保存** —— 把当前画面（叠加前原图）存成 JPG 到所选目录；
- **保存至** —— 浏览/切换 `data_get` 下的保存目录（持久化到 `.selected`）；
- **新建** —— 在当前目录下新建子文件夹；
- **是否开启脏污检测** —— 开：每帧推理并叠加，画面变为并排两列
  「左原图 | 右检测图」（原尺寸 1282 = 640×2 + 2px 间隔），两半各用 2px 青蓝色
  实线框出实际送入网络的 640×384 检测范围（共两个框）；右半图还会按类别色
  画出每个脏污连通域的包围盒（外圈 1px 黑边），横幅显示
  `LENS DIRTY xx.x% N=个数`；关：只推流单幅原图（640×448），不做任何推理。
  没有加载模型时按钮显示「无模型」并禁用。
- **推流宽度** —— 下拉框实时切换推流分辨率：原始（默认，不缩放）/ 1282 /
  960 / 640 / 480 / 320，按宽度等比缩小，下一帧立即生效；接口为
  `/res?w=N`（0 = 原始）。带宽紧张时缩到 640 左右。

画面下方是类别图例（颜色 + 类名 + 当前占比）。状态栏显示分辨率、帧率、
帧序号和「脏污: 是 xx.x%」。

HTTP 接口：

| 路径 | 说明 |
| --- | --- |
| `/` | 预览页 |
| `/stream.mjpg` | MJPEG 流 |
| `/status.json` | 状态：fps、`infer_on`、`has_model`、`n_class`、`cls_ratio`、`fg_ratio`、`n_region`、`regions[]`（连通域类别/原图坐标/面积）等 |
| `/infer?on=0\|1` | 开关实时脏污检测 |
| `/res?w=N` | 切换推流宽度（0 = 原始尺寸，其余等比缩到 N 宽），即时生效 |
| `/labels.json` | 类别名与叠加颜色 |
| `/snapshot.jpg` | 当前帧 JPEG |
| `/save` | 保存当前画面到所选目录 |
| `/dirs.json?path=` / `/setdir?path=` / `/mkdir?parent=&name=` | 目录浏览/切换/新建 |

## 模型与类别

- 模型：6 类逐像素分割，输入 `1×3×384×640` RGB（板上 NV12），输出 `[1,384,640]`
  UINT8 类别图，march `bayes-e`；当前为 `05/` 量化版（`models/model.bin`
  2,361,314 B，md5 `835338e138cd031896c6bef951999de5`，与 `05/output/model.bin`
  一致；上一版 04 为 2,230,242 B，md5 `b1da867a…`）；
- 类别：`clean`（不叠加）、`dirt_dot`（红褐 `#a52a2a`）、`glare`（明黄
  `#ffff00`）、`water`（蓝 `#0000ff`）、`grass_residue`（绿 `#00c800`）、
  `mud_film`（黄褐 `#b47828`）；
- 所有脏污类（class ≥ 1）合计占比 ≥ 0.5% 判脏，脏污区域按上述颜色 50% 叠加
  并显示 `LENS DIRTY xx.x%`，同时保存一张「原图 | 掩码 | 叠加图」三联图到
  `dirty_imgs/`（滚动 100 张，默认最多每 1 秒一张）。

## 脏污连通域

每帧推理后，在 640×384 的类别图上做 **8 邻域连通域分析**（同一类别才算连在
一起），用于把「脏污区域」从逐像素掩码变成一块块独立区域：

- 面积小于 `--region-min-px`（默认 16 个模型像素）的当噪声丢掉；
- 最多保留面积最大的 16 个连通域；
- 页面在检测图（右半）上按类别色画 2px 包围盒（外圈 1px 黑边，深浅背景
  都可见），脏污帧的三联图里也带框；横幅显示 `LENS DIRTY xx.x% N=个数`；
- `/status.json` 新增 `n_region` 和 `regions[]`，每项为
  `{"cls":1,"x":12,"y":30,"w":8,"h":6,"area":42}`：`x/y/w/h` 是**原图**
  （如 640×448）像素坐标、`y` 向下，`area` 是连通域在模型网格上的像素数；
  按面积从大到小排列，未开启推理或没有连通域时为空数组。

连通域会画在叠加图（检测图）上，因此所有叠加可视化——网页右半图、`dirty_imgs/`
三联图的 overlay 面板、`-a/-b` 录制的 `det_*.jpg`——都自动带框。

## 落盘规则

- `temp/` 是 tmpfs，`stream_server` 用 `--consume` 推完即删，重启自动清空；
- `dirty_imgs/` 只存被判脏污的帧，滚动上限 100 张（`--save-max`），默认
  最多每 1 秒存一张（`--save-interval` / `STREAM_SAVE_INTERVAL_MS`）；脏污持续
  时逐帧保存会让三联图编码 + eMMC 写入占满 CPU，推流掉帧、网页延迟变大；
- `data_get/` 只在网页点「保存」时写；
- `on_time/` 仅在 `STREAM_RECORD=a/b` 时创建，滚动保留 N 帧；
- 所有运行日志统一在 `logs/`：`stream.log`、`sub_img.log` 每次启动覆盖，
  `sub_img_imu.log.INFO.*`（glog）只保留最新 `LOG_KEEP`（默认 5）个，
  有新的就删最旧的；
- 其余推理过程不落盘。

## 耗时统计（在板子上跑）

耗时统计由 `bin/min_dirty_infer` 完成，**直接在板子上运行**（它自己 dlopen
`/usr/lib/libdnn.so` 用 BPU 推理，不需要 PC 参与）。默认模型
`/userdata/iris/models/model.bin`，类名自动取同目录 `labels.txt`。

```sh
# 单张图片
/userdata/iris/bin/min_dirty_infer -i /path/图片.png

# 整个文件夹（递归子目录），只打统计
/userdata/iris/bin/min_dirty_infer -i /path/图片目录

# 文件夹 + 输出并排图（vis_<原名>.png：左原图 | 右叠加，宽 2×）
/userdata/iris/bin/min_dirty_infer -i /path/图片目录 -o /userdata/iris/temp/min_vis

# 换模型 / 换标签
/userdata/iris/bin/min_dirty_infer -m /path/model.bin -l /path/labels.txt -i /path/图片
```

输入图片要求：**高 448**（双目左目原图，自动上下各裁 32 行）或 **高 384**
（已是模型输入尺寸，不裁行），**宽 ≥ 640 且多出的列数为偶数**，不做缩放。
尺寸不符的图片会被逐张打印「警告: 跳过…」并不计入耗时；如果目录里一张都
处理不了，只会看到警告，不会打印耗时统计。

每张图打印：各类占比（类名来自 labels.txt）、`fg(class1..5)` 合计与
DIRTY/clean，以及分阶段耗时：

```text
预处理 = 解码 + 裁剪 + RGB→NV12
纯推理 = hbDNNInfer 提交 + WaitTaskDone
后处理 = label 解码 + 统计
单列   = 输入拷贝+flush、叠加、拼图（左原图+右叠加横向拼接）、PNG 编码、写盘
```

`-o` 输出的是「左原图 | 右可视化」并排 PNG（宽度为原图 2 倍，640×448 输入
得到 1280×448）；单图输入时 `-o` 就是文件路径，目录输入时作为输出目录。
并排图的 PNG 编码约 0.4 s/张，会明显拉高单图 total，纯推理统计可省略 `-o`。

文件夹模式结束会打印各阶段平均值和「平均每图 total」。模型加载只做一次，
单独显示为「一次性，不计入单图耗时」。板端实测（04 版模型，10 张 640×448
校准图）平均约 38.8 ms/张（预处理 29.2 + 纯推理 7.2 + 后处理 1.2，不含写图）；
`stream_server` 的 `/status.json` 里 `infer_ms`（04 版模型，含预处理、叠加和
连通域）实测 12–22 ms：干净帧约 12 ms，13 个连通域的脏污帧约 18 ms。

## 重新编译

在 OE 容器 `openexplorer/ai_toolchain_ubuntu_20_x5_gpu:v1.2.8-py310` 里：

```sh
cd /open_explorer/dirty1109/board_dirty_detection
sh scripts/build.sh       # 自动找交叉编译器，make 出 bin/ 下两个程序
# 等价于:
make CROSS_CC=/opt/arm-gnu-toolchain-11.3.rel1-x86_64-aarch64-none-linux-gnu/bin/aarch64-none-linux-gnu-gcc
make clean                # 清理产物
```

产物只依赖 `libc`/`libm`（GLIBC ≤ 2.34）。部署用
`sh scripts/deploy.sh root@<板子IP>`：先上传 `*.new`、远端 md5 校验，再停
`stream_server`、替换、执行 `start.sh`（`sub_img_imu` 不受影响）。

## 运行环境与链路

- 板子：RDK X5 / `goko`，Buildroot 2022.08，自带 `/usr/lib/libdnn.so`；
- 推流链路：板子走 2.4G Wi-Fi，客户端互传有效吞吐约 130 KB/s。并排原图
  1282×448@q70 每帧约 90 KB，链路送不完会积压；网页「推流宽度」缩到 640
  宽约 20 KB/帧（编码耗时也从 ~50 ms 降到 ~13 ms）。

## 排查

```sh
/lib/libc.so.6 | head -1    # 期望 glibc ≥ 2.34
ls /usr/lib | grep -E 'libdnn|libhbrt|libcnn_intf|libhbmem|libalog|libstdc'
tail -f /userdata/iris/logs/stream.log
```

- `无法加载 libdnn.so`：确认上面依赖齐全，程序会自动预加载 `/usr/hobot/lib`
  下的 `libalog → libhbmem → libcnn_intf`；
- 网页延迟大 / 掉帧：先看 `logs/stream.log` 里每 10 秒一行的
  `每帧耗时(ms): 解码 | 检测 | 推流编码 | 存图`，哪一项大就先削哪一项
  （脏污持续时把 `STREAM_SAVE_INTERVAL_MS` 调大最有效）；如果板端算得过来、
  但网页仍延迟，就是 Wi-Fi 链路问题：在网页「推流宽度」下拉框缩到 640
  （约 20 KB/帧），或调小 `STREAM_QUALITY` / `STREAM_FPS`，并确认板端 `iwconfig wlan0` 显示
  `Power Management:off`（`start.sh` 会自动关）；
- `seq` 长时间不涨：多半是停得不久就重启，等 15 秒租约过期再启动；
- 板端 libc 是 musl 或 < 2.34 时，用对应工具链改 `CROSS_CC` 重编；
- 板子重启后**不会自动拉起任何服务**（连 `/goko` 的厂商节点也没有），要手动
  `sh /userdata/iris/scripts/start.sh`；
- 远程 `pkill -f stream_server` 会把 ssh 连接一起打断（返回 255、无输出）：
  `pkill -f` 会匹配到 sshd 拉起的 `bash -c '...'` 包装进程，它的命令行里
  正好含这个字符串。改用 `pkill -x stream_server`；临时实例
  `stream_server.new` 的进程名被截成 15 字符，是 `stream_server.n`；
- 手动跑临时实例自测时（换端口/换输入目录）要自己带库路径，否则报
  `libcnn_intf.so.1: cannot open shared object file`：

```sh
export LD_LIBRARY_PATH=/goko/lib:/app/lib:/app/pub/lib:/middleware/lib:/middleware/pub/lib:/usr/hobot/lib:/usr/hobot/lib/sensor:/system/lib:/system/usr/lib:/lib
cd /userdata/iris
nohup taskset -c 5,7 ./bin/stream_server --watch temp/cctest --port 8082 \
    --model models/model.bin --labels models/labels.txt --save-max 0 \
    </dev/null >logs/cctest.log 2>&1 &
# 往 temp/cctest 丢一张 640x448 PNG，再读 http://127.0.0.1:8082/status.json
# 看 n_region/regions，抓 /snapshot.jpg 核对框的位置和颜色
```

  临时实例不要加 `--consume`，否则会把自测图片删掉；也不要和 8080 的正式
  实例用同一个 watch 目录。
