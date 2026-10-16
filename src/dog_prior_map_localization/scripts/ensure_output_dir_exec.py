#!/usr/bin/env python3
"""Create a configured run-output directory, then exec the wrapped ROS node."""

import os
import sys
from pathlib import Path


def main():
    if len(sys.argv) < 2:
        raise SystemExit("usage: ensure_output_dir_exec.py COMMAND [ARG ...]")
    output_dir = os.environ.get("P45_OUTPUT_DIRECTORY", "")
    if output_dir:
        Path(output_dir).mkdir(parents=True, exist_ok=True)
    os.execvpe(sys.argv[1], sys.argv[1:], os.environ)


if __name__ == "__main__":
    main()
