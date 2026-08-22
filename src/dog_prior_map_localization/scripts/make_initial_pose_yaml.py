#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""生成初始位姿 YAML 小工具。"""
import argparse
import math
from pathlib import Path


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--x", type=float, default=0.0)
    parser.add_argument("--y", type=float, default=0.0)
    parser.add_argument("--z", type=float, default=0.0)
    parser.add_argument("--roll", type=float, default=0.0, help="deg")
    parser.add_argument("--pitch", type=float, default=0.0, help="deg")
    parser.add_argument("--yaw", type=float, default=0.0, help="deg")
    parser.add_argument("--output", required=True)
    args = parser.parse_args()
    text = f"""filter:
  initial_position: [{args.x:.6f}, {args.y:.6f}, {args.z:.6f}]
  initial_rpy_deg: [{args.roll:.6f}, {args.pitch:.6f}, {args.yaw:.6f}]
"""
    out = Path(args.output).expanduser()
    out.parent.mkdir(parents=True, exist_ok=True)
    out.write_text(text, encoding="utf-8")
    print(f"已保存: {out}")


if __name__ == "__main__":
    main()
