#!/usr/bin/env bash
set -euo pipefail
repo=$(realpath "$1")
out=$(realpath -m "$2")
pybind_headers=$(realpath "$3")
test "$(git -C "$repo" rev-parse HEAD)" = 5ea1c0dece0bbd22fed6c54baeea6b31fb98a802
test "$(git -C "$repo/libE57Format" rev-parse HEAD)" = 1914b8ea972251d3bb49a33828497dde683205d9
mkdir -p "$out/pye57"
cmake -S "$repo/libE57Format" -B "$out/e57_nontest" -DE57_GIT_SUBMODULE_UPDATE=OFF -DCMAKE_POSITION_INDEPENDENT_CODE=ON -DCMAKE_BUILD_TYPE=Release -DE57_BUILD_TEST=OFF -DE57_RELEASE_LTO=OFF
cmake --build "$out/e57_nontest" -j3
cp "$repo/src/pye57/"*.py "$out/pye57/"
g++ -std=c++17 -O2 -shared -fPIC $(/usr/bin/python3-config --includes) -I"$pybind_headers" -I"$repo/libE57Format/include" -I"$repo/libE57Format/src" -I"$out/e57_nontest" '-DVERSION_INFO="0.4.15"' '-DREVISION_ID="pye57-0.4.15"' "$repo/src/pye57/libe57_wrapper.cpp" "$out/e57_nontest/libE57Format.a" -lxerces-c -o "$out/pye57/libe57$(/usr/bin/python3-config --extension-suffix)"
PYTHONPATH="$out" /usr/bin/python3 -c 'import pye57; print(pye57.__file__)'
