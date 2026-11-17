# lianghua — 脏污分割模型量化（ONNX → BPU `.bin`）

当前板端使用的 **6 类镜面脏污分割模型**的完整量化工程（2026-11-13 构建，即此前的
`dirty1109/04/`）。量化工具为地平线 `hb_mapper`（OE 容器
`openexplorer/ai_toolchain_ubuntu_20_x5_gpu:v1.2.8-py310`，hbdk 3.49.15 /
horizon_nn 1.1.0 / hb_mapper 1.24.3）。

## 目录

```text
board_dirty_detection/lianghua/
├── best.onnx                 训练导出的浮点 ONNX（6.0 MB）
├── config.yaml               hb_mapper 量化配置
├── labels.txt                类别名（6 类）
├── cal_img/                  校准原图，240 张 640x448 PNG
├── cal_raw/                  hb_mapper 读取的校准数据，240 个 float32 raw
├── cal_img_to_raw.py         cal_img → cal_raw 转换脚本
├── hb_mapper_makertbin.log   本次量化完整日志
└── output/                   量化产物
    ├── model.bin             ★ 上板模型（2,230,242 B，md5 b1da867a297f4b023babd29677c87530）
    ├── model_quant_info.json 量化信息（权重/激活 bit 宽、逐层配置）
    ├── model_quantized_model.onnx    量化后仿真模型（PC 端 HB_ONNXRuntime 可用）
    ├── model_calibrated_model.onnx   校准后模型
    ├── model_optimized_float_model.onnx / model_original_float_model.onnx
    └── main_graph_subgraph_0.json/.html  编译报告（算子分布、耗时估算）
```

> 校准大文件（`cal_img` 94 MB、`cal_raw` 676 MB）保留在本地工程里，用于复现量化；
> 只部署板端时不需要它们。

## 模型规格

| 项 | 值 |
| --- | --- |
| 任务 | 逐像素 6 类语义分割 |
| 训练输入 | `1x3x384x640`，RGB，NCHW，`norm_type: data_scale`，`scale_value: 1/255` |
| 板上输入 | `nv12`（pyramid），程序侧预处理为 NV12 后直接提交 BPU |
| 输出 | `[1,384,640]` UINT8 类别图 |
| 架构 | `march: bayes-e`，`core_num: 1`，`compile_mode: latency`，`O3` |
| 特殊节点 | `/model.10/m/m.0/attn/Softmax` 指定 BPU 运行，输入/输出 int16 |
| 模型文件 | `model.bin` 2,230,242 B，md5 `b1da867a297f4b023babd29677c87530` |

类别（`labels.txt`，class 0 为背景）：

| 索引 | 类名 | 叠加颜色 |
| --- | --- | --- |
| 0 | clean | 不叠加 |
| 1 | dirt_dot | 红褐 `#a52a2a` |
| 2 | glare | 明黄 `#ffff00` |
| 3 | water | 蓝 `#0000ff` |
| 4 | grass_residue | 绿 `#00c800` |
| 5 | mud_film | 黄褐 `#b47828` |

板端所有 class ≥ 1 的面积合计占比 ≥ 0.5% 判为脏污。

## 重新量化

1. **生成校准数据**（只需 `cal_img`，输出 `cal_raw`）：

   ```sh
   python3 cal_img_to_raw.py            # ./cal_img -> ./cal_raw
   ```

   每个 `.raw` 为 2,949,120 B（640×384×3×float32）：图片上下各裁 32 行取中间
   640×384，BGR→RGB，HWC→NCHW，取值 0..255。脚本产物与现有 `cal_raw`
   逐字节一致（已验证）。

2. **运行 hb_mapper**（在 OE 容器里，仓库挂到 `/open_explorer`）：

   ```sh
   docker run --rm -v /home/iris/D-Robotics_x5_open_explorer_v1.2.8-py310_20240926:/open_explorer \
       -w /open_explorer/dirty1109/board_dirty_detection/lianghua \
       openexplorer/ai_toolchain_ubuntu_20_x5_gpu:v1.2.8-py310 \
       hb_mapper makertbin config.yaml --model-type onnx
   ```

   输出写到 `config.yaml` 的 `working_dir`（`output/`），日志 `hb_mapper_makertbin.log`。

3. **部署**：`output/model.bin` → 板端 `/userdata/iris/models/model.bin`
   （仓库内先同步到 `../models/model.bin`），labels 未变时无需动
   `/userdata/iris/models/labels.txt`；然后重启 `stream_server`
   （`pkill -x stream_server; sh /userdata/iris/scripts/start.sh`）。
   板端日志会打印 `模型: model (...), 输入 640x384 NV12, 6 类` 可确认加载成功。

## 配置要点（`config.yaml`）

- `calibration_parameters.cal_data_dir: cal_raw`，`cal_data_type: float32`，
  校准算法 `default`；
- `input_type_rt: nv12`，`input_layout_train: NCHW`；
- `output_model_file_prefix: model` 决定产物名 `model.bin`；
- 校准图选取的是真实镜头画面（含脏污/干净/雨天等场景），不要用训练集原图替代。

## 板端性能参考

- `stream_server` 每帧「检测」（预处理 + 纯推理 + 后处理 + 叠加）：04 模型实测
  约 11–14 ms（`/status.json` 的 `infer_ms`），推流 7–8 fps 时 BPU 无排队；
- 03 模型用 `min_dirty_infer` 测过纯推理约 6.7 ms/张，04 同架构预计相当；
  需要精确数字时在板上执行
  `bin/min_dirty_infer -i <640x448 图片目录>`（详见 `../README.md`）。
