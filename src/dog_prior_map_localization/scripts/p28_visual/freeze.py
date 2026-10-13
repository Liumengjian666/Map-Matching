#!/usr/bin/env python3
"""P28 input/output freeze. Never accepts or reads GT."""
import argparse
import hashlib
import json
from pathlib import Path
import yaml


def sha(path):
    digest = hashlib.sha256()
    with Path(path).open("rb") as stream:
        for block in iter(lambda: stream.read(1 << 20), b""):
            digest.update(block)
    return digest.hexdigest()


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--repo", type=Path, required=True)
    parser.add_argument("--runtime", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--run", type=Path, action="append", default=[])
    args = parser.parse_args()
    if args.output.exists():
        raise ValueError("FREEZE_ALREADY_EXISTS")
    repo = args.repo.resolve()
    config = repo / "src/dog_prior_map_localization/config/p18_corridor01_ssd_map.yaml"
    overlay = repo / "src/dog_prior_map_localization/config/p28_mature_visual.yaml"
    map_path = Path(yaml.safe_load(config.read_text())["map"]["pcd_fallback_path"])
    files = [config, overlay, map_path,
        repo / ".p10_corridor01_deterministic_oosm_replay_r1/input_v2/corridor01_p10_causal_startup5_eval35_fixedlag_v2.bag",
        repo / ".p17_visual_aided_ndt_r1/visual_input/visual.csv",
        repo / ".p28_r2_mature_visual_module/feature_input_validated/tracks.csv",
        repo / ".p28_r2_mature_visual_module/feature_input_validated/extraction.json"]
    files += [args.runtime / "lib/dog_prior_map_localization" / name
              for name in ("dog_prior_map_ekf_node_cpp", "dog_prior_map_ndt_node_cpp")]
    files.append(repo / "docs/p10_corridor01_deterministic_oosm_replay_r1/posthoc_metrics.json")
    for run in args.run:
        for file in run.iterdir():
            if file.is_file() and file.suffix in (".csv", ".json", ".yaml", ".bag", ".log"):
                files.append(file)
    hashes = {str(file.resolve()): sha(file) for file in files}
    expected = {config: "dc0850f91b43a4afed2e692644ac292db96a599e33fdafb414f3a986bc743220",
                map_path: "103a01b2c89ca2adbd2b2e22256acba6e91f40b1028857c2103f08c6c3eb8f8f",
                files[3]: "7f72f808a9f2282c65705e7e1e731061bb2cc2950329e46da768608e4a7d552e",
                files[4]: "10869519890a5adcb6f3310bba05f379a959018cf123ea8a90c1cf4ea1314c79",
                files[5]: "9873f7b570f72595d5ab3479f4f48a0bb42728024c3ee830eabeac6593e2441c"}
    for path, digest in expected.items():
        if hashes[str(path.resolve())] != digest:
            raise ValueError("FROZEN_INPUT_CHANGED:" + str(path))
    result = dict(protocol="P28_R2_PRE_GT_OUTPUT_FREEZE" if args.run else "P28_R2_INPUT_FREEZE",
                  GT_LOADED=False, files_sha256=hashes,
                  raw_camera_full_hash_rechecked=False,
                  raw_camera_provenance="See extraction.json: bounded selected-image read and content hash")
    args.output.write_text(json.dumps(result, indent=2) + "\n")
    print(json.dumps(dict(protocol=result["protocol"], files=len(files))))


if __name__ == "__main__":
    main()
