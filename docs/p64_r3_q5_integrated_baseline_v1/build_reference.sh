#!/usr/bin/env bash
set -euo pipefail
here=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
up=/home/jian/p64_r3_q4_ws/run/reference/xicp
ref=/home/jian/dcreg_reference
[[ $(git -C "$up" rev-parse HEAD) == 0fbe4175ea205a271f85287abfd8048e5f7dd32a ]]
[[ $(git -C "$ref" rev-parse HEAD) == 8ce8451b15491a4bbe17cf85ab02a8bed6696861 ]]
mkdir "$1"
python3 - "$up" "$1" <<'PY'
from pathlib import Path
import sys
p=Path(sys.argv[1])/'libpointmatcher/pointmatcher/ICP.cpp'
lines=p.read_text().splitlines()
ranges=[]
for key in ('::decideLocalizabilityLevel(', '::countContributionValuesAndCheckWhetherProblemIsConstrainedVeryWell('):
    head=next(i for i,l in enumerate(lines) if key in l)
    tail=next(i for i in range(head,len(lines)) if lines[i]=='}')
    ranges.append('\n'.join(lines[head:tail+1]).replace('typename PointMatcher<T>::','XCore::')
        .replace('PointMatcher<T>::ICP::','XCore::'))
(Path(sys.argv[2])/'xicp_extracted.inc').write_text('\n\n'.join(ranges)+'\n')
PY
env -u LD_LIBRARY_PATH g++ -std=c++17 -O1 -DNDEBUG -I"$1" -I"$ref/DCReg/include" -I/usr/include/eigen3 \
  -I/usr/include/pcl-1.10 "$here/ref_degeneracy.cpp" -lpcl_io -lpcl_common -lpcl_kdtree -ltbb -o "$1/reference"
