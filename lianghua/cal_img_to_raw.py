#!/usr/bin/env python3
"""把 cal_img 里的校准图转成 hb_mapper 量化用的 float32 raw。

config.yaml 中 calibration_parameters.cal_data_dir 指向 cal_raw，hb_mapper
逐个读取目录下的 .raw 文件，每个文件必须是模型训练输入的 float32 连续内存：
    RGB、NCHW、取值 0..255

预处理：
    1. 读图（BGR uint8）
    2. 长边缩到 640（INTER_LINEAR，和 ultralytics rect=True 一致）
    3. 按 5:3 宽高比居中裁切到 640x384
       - 太宽裁左右，太扁裁上下
       - 裁切后尺寸仍不对则兜底 resize
    4. BGR -> RGB
    5. HWC -> CHW -> NCHW，转 float32，按原名另存为 .raw

用法（OE 容器或本机均可，需要 opencv-python）:
    python3 cal_img_to_raw.py                  # cal_img -> cal_raw
    python3 cal_img_to_raw.py --src 图片目录 --dst 输出目录
"""

from __future__ import annotations

import argparse
import math
import sys
from pathlib import Path

import cv2
import numpy as np

IMGSZ = 640          # 长边缩放目标（和训练 imgsz 一致）
HEIGHT = 384         # 模型输入高
WIDTH = 640         # 模型输入宽
IMG_EXTS = {".png", ".jpg", ".jpeg", ".bmp", ".webp"}


def training_resize_crop(image: np.ndarray) -> np.ndarray:
    """复刻训练预处理：长边缩到 imgsz，再按目标宽高比居中裁切。

    与 make_calib.py 的 training_resize_crop 逐行对齐：
    - 缩放部分和 ultralytics BaseDataset.load_image(rect_mode=True) 一致
    - 裁切部分和 tools/crop_dataset.aspect_crop_box 一致
    """
    h0, w0 = image.shape[:2]

    # ---- 第一步：长边缩放到 imgsz ----
    ratio = IMGSZ / max(h0, w0)
    if ratio != 1:
        w = min(math.ceil(w0 * ratio), IMGSZ)
        h = min(math.ceil(h0 * ratio), IMGSZ)
        image = cv2.resize(image, (w, h), interpolation=cv2.INTER_LINEAR)

    # ---- 第二步：按宽高比居中裁切 ----
    height, width = image.shape[:2]
    target_ratio = WIDTH / HEIGHT
    source_ratio = width / height

    if abs(source_ratio - target_ratio) < 1e-6:
        top, left, new_h, new_w = 0, 0, height, width
    elif source_ratio > target_ratio:       # 太宽，裁左右
        new_w = round(height * target_ratio)
        left = (width - new_w) // 2
        top, new_h = 0, height
    else:                                   # 太扁，裁上下
        new_h = round(width / target_ratio)
        top = (height - new_h) // 2
        left, new_w = 0, width

    image = image[top:top + new_h, left:left + new_w]

    # ---- 兜底：裁切后尺寸仍不对则 resize ----
    if (new_h, new_w) != (HEIGHT, WIDTH):
        image = cv2.resize(image, (WIDTH, HEIGHT), interpolation=cv2.INTER_LINEAR)

    return image


def convert(img_dir: Path, raw_dir: Path) -> tuple[int, int]:
    raw_dir.mkdir(parents=True, exist_ok=True)
    ok = skip = 0
    for path in sorted(img_dir.iterdir()):
        if path.suffix.lower() not in IMG_EXTS:
            continue
        img = cv2.imread(str(path), cv2.IMREAD_COLOR)
        if img is None:
            print(f"跳过 {path.name}: 读取失败", file=sys.stderr)
            skip += 1
            continue

        # 和 make_calib.py 完全一致的预处理链路
        img = training_resize_crop(img)          # 缩放 + 裁切 → 640x384 BGR
        rgb = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
        nchw = rgb.transpose(2, 0, 1)[np.newaxis, ...].astype(np.float32)

        expected = 1 * 3 * HEIGHT * WIDTH * 4
        if nchw.nbytes != expected:
            print(f"跳过 {path.name}: {nchw.nbytes} 字节，应为 {expected}",
                  file=sys.stderr)
            skip += 1
            continue

        nchw.tofile(str(raw_dir / (path.stem + ".raw")))
        ok += 1
    return ok, skip


def main() -> int:
    here = Path(__file__).resolve().parent
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--src", type=Path, default=here / "cal_img",
                    help="校准图片目录（默认 ./cal_img）")
    ap.add_argument("--dst", type=Path, default=here / "cal_raw",
                    help="raw 输出目录（默认 ./cal_raw）")
    args = ap.parse_args()
    if not args.src.is_dir():
        print(f"目录不存在: {args.src}", file=sys.stderr)
        return 1
    ok, skip = convert(args.src, args.dst)
    print(f"done: {ok} 张 -> {args.dst}（跳过 {skip} 张，每张 {HEIGHT*WIDTH*3*4} B）")
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
