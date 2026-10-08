#!/usr/bin/env python3
"""Build-time literal extraction of frozen math/BASE263 source; no data reader."""
import argparse
import hashlib
import json
from pathlib import Path
import subprocess

ROOT = Path(__file__).resolve().parents[4]
START = "3a98a3cd1d64d7aba6888d3136295a0b41c38fa6"
HISTORY = "9945c4f5c3d7759104de108a594bcaf2553fd78c"
ENERGY = "src/dog_prior_map_fastlio2_frontend_exp/scripts/p9/p9_ndt_energy_contract.cpp"
GENERATOR = "src/dog_prior_map_fastlio2_frontend_exp/scripts/p7/dual_u_r1_multistart_runner.cpp"


def region(source, begin, end):
    if source.count(begin) != 1 or source.count(end) != 1:
        raise RuntimeError("ambiguous frozen source region")
    return source[source.index(begin):source.index(end)]


def generate(output):
    sources = {path: subprocess.check_output(["git", "show", sha + ":" + path], cwd=ROOT)
               for path, sha in ((ENERGY, START), (GENERATOR, HISTORY))}
    if (ROOT / ENERGY).read_bytes() != sources[ENERGY]:
        raise RuntimeError("frozen energy source changed")
    energy, generator = (sources[path].decode() for path in (ENERGY, GENERATOR))
    prelude = "#pragma once\n#include <Eigen/Core>\n#include <Eigen/Geometry>\n#include <array>\n#include <cmath>\n#include <iomanip>\n#include <sstream>\n#include <stdexcept>\n#include <string>\n#include <vector>\n"
    geometry = prelude + "namespace r4_frozen_geometry {\nconstexpr double kPi = 3.14159265358979323846;\n"
    geometry += region(energy, "std::vector<std::string> split(", "std::uint64_t parseU64(")
    geometry += region(energy, "Eigen::Matrix4f parseMatrix16(", "std::uint64_t bits(double value)")
    geometry += "}\n"
    seeds = prelude + '#include "dog_prior_map_fastlio2_frontend_exp/frontend_types.hpp"\n'
    seeds += "namespace r4_frozen_seed {\nnamespace paper = dog_prior_map_fastlio2_frontend_exp;\n"
    for begin, end in (("struct Seed {", "std::vector<std::string> splitCsv("),
                       ("paper::Pose3d parsePose(", "std::vector<CohortFrame> readCohort("),
                       ("std::vector<Seed> planarSeeds(", "std::vector<Seed> widePlanarSeeds("),
                       ("Eigen::Matrix4f expSE3(", "double translationDistance(const paper::Pose3d&")):
        seeds += region(generator, begin, end)
    seeds += "}\n"
    output.mkdir(parents=True, exist_ok=True)
    generated = {"p9_r4_frozen_geometry.hpp": geometry, "p9_r4_frozen_seeds.hpp": seeds}
    for name, content in generated.items():
        (output / name).write_text(content)
    manifest = dict(start_sha=START, generator_git_sha=HISTORY,
        source_sha256={path: hashlib.sha256(data).hexdigest() for path, data in sources.items()},
        generated_sha256={name: hashlib.sha256(content.encode()).hexdigest() for name, content in generated.items()},
        extraction="verbatim contiguous frozen regions, no rewriting")
    (output / "r4_codegen_manifest.json").write_text(json.dumps(manifest, indent=2, sort_keys=True) + "\n")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("output", type=Path)
    generate(parser.parse_args().output)
