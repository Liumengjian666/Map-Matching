#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
把 FAST-LIVO2 输出的 PCD 地图转换为机器狗端更容易加载的 NPZ 格式。

为什么要转换：
1. PCD 文本/二进制解析比较慢，机器狗端每次启动都读 PCD 会浪费时间。
2. NPZ 只保存 xyz/颜色等 numpy 数组，Python 节点加载更快。
3. 可以在转换时做体素降采样，减少机器狗端内存和 KDTree 构建时间。

使用示例：
rosrun dog_prior_map_localization pcd_to_npz_map.py \
  --input /path/to/loop2_all_downsampled_points.pcd \
  --output /path/to/loop2_prior_map_voxel_0p30.npz \
  --voxel 0.30
"""
import argparse
import struct
from pathlib import Path

import numpy as np


def read_pcd_xyz(path: Path) -> np.ndarray:
    """读取常见 PCD 文件，只返回 N×3 的 xyz 点。

    当前兼容 FAST-LIVO2 常见的 binary PCD：FIELDS x y z rgb。
    如果以后换成 ascii PCD，也做了简单兼容。
    """
    raw = path.read_bytes()
    header_end = raw.find(b"DATA ")
    if header_end < 0:
        raise RuntimeError(f"不是合法 PCD 文件，找不到 DATA 字段: {path}")
    line_end = raw.find(b"\n", header_end)
    header = raw[: line_end + 1].decode("latin1")
    data = raw[line_end + 1 :]

    fields = []
    sizes = []
    types = []
    points = None
    data_mode = None
    for line in header.splitlines():
        items = line.split()
        if not items:
            continue
        if items[0] == "FIELDS":
            fields = items[1:]
        elif items[0] == "SIZE":
            sizes = [int(v) for v in items[1:]]
        elif items[0] == "TYPE":
            types = items[1:]
        elif items[0] == "POINTS":
            points = int(items[1])
        elif items[0] == "DATA":
            data_mode = items[1]

    if not {"x", "y", "z"}.issubset(set(fields)):
        raise RuntimeError(f"PCD 缺少 x/y/z 字段: {fields}")

    if data_mode == "ascii":
        arr = np.loadtxt(path, comments="#", skiprows=len(header.splitlines()), dtype=np.float32)
        xyz_idx = [fields.index("x"), fields.index("y"), fields.index("z")]
        return arr[:, xyz_idx].astype(np.float32)

    if data_mode != "binary":
        raise RuntimeError(f"暂不支持 DATA {data_mode}，请先转成 binary/ascii PCD")

    # 构造 numpy dtype，兼容 F/U/I 三类 4 字节字段。
    dtype_fields = []
    for field, size, typ in zip(fields, sizes, types):
        if size != 4:
            raise RuntimeError(f"当前脚本只处理 4 字节字段，发现 {field} size={size}")
        if typ == "F":
            dt = "<f4"
        elif typ == "U":
            dt = "<u4"
        elif typ == "I":
            dt = "<i4"
        else:
            raise RuntimeError(f"未知 PCD TYPE: {typ}")
        dtype_fields.append((field, dt))
    arr = np.frombuffer(data, dtype=np.dtype(dtype_fields), count=points)
    xyz = np.vstack([arr["x"], arr["y"], arr["z"]]).T.astype(np.float32)
    xyz = xyz[np.isfinite(xyz).all(axis=1)]
    return xyz


def voxel_downsample(points: np.ndarray, voxel: float) -> np.ndarray:
    """简单体素降采样：每个体素保留第一个点，速度快、够机器狗端使用。"""
    if voxel <= 0:
        return points
    keys = np.floor(points / voxel).astype(np.int64)
    _, unique_idx = np.unique(keys, axis=0, return_index=True)
    return points[np.sort(unique_idx)]


def main():
    parser = argparse.ArgumentParser(description="Convert PCD prior map to low-compute NPZ map")
    parser.add_argument("--input", required=True, help="FAST-LIVO2 输出的 PCD 地图")
    parser.add_argument("--output", required=True, help="输出 NPZ 地图")
    parser.add_argument("--voxel", type=float, default=0.30, help="转换时体素降采样大小，单位 m")
    args = parser.parse_args()

    in_path = Path(args.input).expanduser()
    out_path = Path(args.output).expanduser()
    out_path.parent.mkdir(parents=True, exist_ok=True)

    points = read_pcd_xyz(in_path)
    raw_num = len(points)
    points = voxel_downsample(points, args.voxel)

    np.savez_compressed(
        out_path,
        points=points.astype(np.float32),
        voxel_size=np.float32(args.voxel),
        source=str(in_path),
        raw_points=np.int64(raw_num),
    )
    print(f"输入点数: {raw_num}")
    print(f"输出点数: {len(points)}")
    print(f"保存: {out_path}")


if __name__ == "__main__":
    main()
