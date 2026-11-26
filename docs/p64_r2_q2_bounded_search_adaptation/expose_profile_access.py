"""Isolate instrumentation access in a generated copy of the Q1 scoring header."""
import sys
from pathlib import Path

source, output = (Path(item) for item in sys.argv[1:])
text = source.read_text()
old = """    const auto it = cells_.find(voxel(p, 1));
    associated = it != cells_.end();
    if (!associated) {
      return std::log(outlierRatio);
    }
    const auto& cell = it->second;
"""
new = """    const auto* cell = lookup(p);
    associated = cell != nullptr;
    return evaluate(p, cell);
  }

  const ParkCell* lookup(const Point& p) const {
    const auto it = cells_.find(voxel(p, 1));
    return it == cells_.end() ? nullptr : &it->second;
  }

  static double evaluate(const Point& p, const ParkCell* ptr) {
    if (!ptr) return std::log(outlierRatio);
    const auto& cell = *ptr;
"""
assert text.count(old) == 1
text = text.replace(old, new)
anchor = "  const TargetGrid::Map& cells()"
assert text.count(anchor) == 1
text = text.replace(anchor, "  const ParkMap& parkMap() const { return park_; }\n\n" + anchor)
with output.open("x") as stream:
    stream.write(text)
