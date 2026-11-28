#!/usr/bin/env bash
set -euo pipefail
here=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
source_dir=$(realpath "$1")
out=$(realpath -m "$2")
mkdir -p "$out"
cmake -S "$source_dir" -B "$out/lib" -DCMAKE_BUILD_TYPE=Release -DE57_BUILD_TEST=OFF -DE57_RELEASE_LTO=OFF
cmake --build "$out/lib" -j3
g++ -std=c++17 -O3 -I"$source_dir/include" -I"$out/lib" -I/usr/include/eigen3 \
    "$here/e57_crop.cpp" "$out/lib/libE57Format.a" -lxerces-c -o "$out/e57_crop"
