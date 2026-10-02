#!/usr/bin/env python3
"""R5 guarded reset debugging: frozen inputs, no GT, no core tuning."""
import argparse
from pathlib import Path
from p6_a3g_r4_run import ROOT, run

START = "68005b531c8127904f63c42de5890748bea39196"

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--executable", type=Path, required=True)
    parser.add_argument("--directory", type=Path, required=True)
    parser.add_argument("--limit", type=int, required=True)
    parser.add_argument("--config", type=Path,
                        default=ROOT / "config/p6_tracking_reinitialization.conf")
    args = parser.parse_args()
    raise SystemExit(run(args.executable.resolve(), args.directory.resolve(),
                         args.limit, args.config.resolve(), START))
