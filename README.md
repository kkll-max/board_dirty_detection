# board_dirty_detection — 镜面脏污检测（量化 + 板端）

割草机镜头脏污检测的完整交付：BPU 6 类分割模型的量化工程（`05/`），以及板上的推流 / 检测 / 网页程序。

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

- 板端部署位置：`/userdata/iris
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
# 1) 重新量化（docker工具链）
cd 05
hb_mapper makertbin config.yaml --model-type onnx

# 2) 交叉编译板端程序（docker工具链）
cd ..
sh scripts/build.sh                        

# 3) 部署到板子并重启推流
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

### 常用环境变量

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
  「左原图 | 右检测图」，两半各用 2px 青蓝色实线框出实际送入网络的 640×384 检测范围；右半图还会按类别色
  画出每个脏污连通域的包围盒（外圈 1px 黑边）
- **推流宽度** —— 下拉框实时切换推流分辨率,按宽度等比缩小，下一帧立即生效

画面下方是类别图例（颜色 + 类名 + 当前占比）。状态栏显示分辨率、帧率。

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

```text
预处理 = 解码 + 裁剪 + RGB→NV12
纯推理 = hbDNNInfer 提交 + WaitTaskDone
后处理 = label 解码 + 统计
单列   = 输入拷贝+flush、叠加、拼图（左原图+右叠加横向拼接）、PNG 编码、写盘
```