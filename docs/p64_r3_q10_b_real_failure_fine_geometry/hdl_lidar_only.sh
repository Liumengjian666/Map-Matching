#!/usr/bin/env bash
set -euo pipefail
d=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
upstream=$(realpath "$1"); inputs=$(realpath "$2"); out=$(realpath -m "$3")
wrapper="$out.wrapper.sh"
stream="$out.stream.py"
q9="$d/../p64_r3_q9_public_benchmark_fine_geometry"
python3 - "$upstream/replay_hdl.sh" "$q9/stream_hdl.py" "$q9" "$wrapper" "$stream" <<'PY'
from pathlib import Path
import sys
wrapper,script,lib,out,bridge=sys.argv[1:]
src=Path(script).read_text()
src=src.replace("from paired import read_pcd",f"sys.path.insert(0,{lib!r})\nfrom paired import read_pcd")
old="(map_pub,scanner,imu,initial)"
assert src.count(old)==1
Path(bridge).write_text(src.replace(old,"(map_pub,scanner,initial)"))
w=Path(wrapper).read_text()
start=[l for l in w.splitlines() if "/stream_hdl.py" in l]
assert len(start)==1
assert w.count("/hdl_q9/use_imu true")==1
w=w.replace("/hdl_q9/use_imu true","/hdl_q9/use_imu false").replace(start[0],f'/usr/bin/python3 "{bridge}" "$inputs" "$result/trajectory.csv"')
Path(out).write_text(w)
PY
bash "$wrapper" "$upstream" "$inputs" "$out" 11751
