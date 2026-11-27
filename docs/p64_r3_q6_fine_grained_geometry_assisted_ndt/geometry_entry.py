"""Q6 geometry subprocess entry point."""
import runpy
from pathlib import Path

runpy.run_path(str(Path(__file__).resolve().parent.parent /
                   "p64_r3_q5_integrated_baseline_v1/geometry.py"), run_name="__main__")
