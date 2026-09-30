#!/usr/bin/env python3
"""Read-only A2C boundary audit against the authorized frozen source commit."""
import hashlib
from pathlib import Path
import subprocess

START_SHA = "7329f74e5691c08379251046cce268477677f4f3"
ROOT = Path(__file__).resolve().parents[3]
REL = "src/dog_prior_map_fastlio2_frontend_exp/scripts/p6_i1_branched_recovery.cpp"


def main():
    original = subprocess.check_output(
        ["git", "show", f"{START_SHA}:{REL}"], cwd=ROOT, text=True)
    current = (ROOT / REL).read_text()
    stripped = current.replace(
        '#include "dog_prior_map_fastlio2_frontend_exp/fixed_lag_production.hpp"\n', "")
    stripped = stripped.replace('\n#include "p6_a2c_fixed_lag_producer.hpp"\n', "")
    start = stripped.index('    if (argc == 2 && std::string(argv[1]) == "A2C_FIXTURE")')
    end = stripped.index('    if (((argc >= 11 && argc <= 15)', start)
    stripped = stripped[:start] + stripped[end:]
    stripped = "\n".join(line for line in stripped.split("\n")
                         if '<< "  p6_i1_branched_recovery FULL_FIXED_LAG_V2_EXPERIMENTAL' not in line)
    if stripped != original:
        raise RuntimeError("legacy producer changed beyond additive include/dispatch/usage")
    print("LEGACY_FULL_SOURCE_BYTE_PARITY_PASS sha256=" +
          hashlib.sha256(original.encode()).hexdigest())
    producer = (Path(__file__).parent / "p6_a2c_fixed_lag_producer.hpp").read_text()
    handoff = producer[producer.index("FixedLagProducerResult runFixedLagProducer("):]
    for forbidden in ("FastLio2IkfomFrontend", "frontend.predict", "initializer.predict",
                      "applyPoseMeasurement", "applyProjected", "setWindowPredictionSeed",
                      "projectMapProductPoseCovariance", "getState()", "get_P()"):
        if forbidden in handoff:
            raise RuntimeError("post-handoff second estimator dependency: " + forbidden)
    for required in ("adapter.prepareStateAt(event.stamp_ns,&predicted", "prediction*T_il",
                     "adapter.latestMarginalCovariance(&prior", "prior.map_pose_covariance6",
                     "evaluateSelectedLidarNis(predicted,measurement,prior"):
        if required not in handoff:
            raise RuntimeError("missing real producer connection: " + required)
    print("POST_HANDOFF_IKFOM_CALLS_ZERO_AND_WINDOW_ONLY_PREDICTION_PASS")


if __name__ == "__main__":
    main()
