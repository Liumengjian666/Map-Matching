"""Freeze P23 inputs/IDs and configure one direct capture replay; no GT."""
import argparse
import csv
import hashlib
import json
from pathlib import Path
import rosbag
import yaml


def sha(path):
    digest=hashlib.sha256()
    with Path(path).open('rb') as stream:
        for block in iter(lambda:stream.read(1<<20),b''):digest.update(block)
    return digest.hexdigest()


def prepare(repo):
    repo=repo.resolve()
    cache=repo/'.p23_r1_budgeted_coupled_subspace_refinement'
    if cache.exists():raise ValueError('P23_CACHE_EXISTS')
    baseline=repo/'src/dog_prior_map_localization/config/p18_corridor01_ssd_map.yaml'
    config=yaml.safe_load(baseline.read_text())
    bag=repo/'.p10_corridor01_deterministic_oosm_replay_r1/input_v2/corridor01_p10_causal_startup5_eval35_fixedlag_v2.bag'
    visual=repo/'.p17_visual_aided_ndt_r1/visual_input/visual.csv'
    assets={Path(config['map']['pcd_fallback_path']):'103a01b2c89ca2adbd2b2e22256acba6e91f40b1028857c2103f08c6c3eb8f8f',
            bag:'7f72f808a9f2282c65705e7e1e731061bb2cc2950329e46da768608e4a7d552e',
            visual:'10869519890a5adcb6f3310bba05f379a959018cf123ea8a90c1cf4ea1314c79',
            baseline:'dc0850f91b43a4afed2e692644ac292db96a599e33fdafb414f3a986bc743220',
            repo/'devel/lib/dog_prior_map_localization/dog_prior_map_ekf_node_cpp':'fbc1750dc605a7fd3b9549b310394fbff3a5781cad2272ef55b9f49df8c3435a'}
    for path,digest in assets.items():
        if sha(path)!=digest:raise ValueError('FROZEN_INPUT_CHANGED:'+str(path))
    selection=repo/'docs/p22_r1_fixed_source_ndt_pair_test/frame_selection.csv'
    with selection.open() as stream: chosen=list(csv.DictReader(stream))
    assert [int(row['frame_index']) for row in chosen]==[2,284,293,296,308,347,368,396]
    # Original CSV stores seconds as double. Recover unique exact ROS ns from
    # the original bag by full timestamp equality, never nearest-frame matching.
    by_seconds={float(row['scan_stamp']):row for row in chosen}
    found=[]
    with rosbag.Bag(str(bag)) as source:
        for _,message,_ in source.read_messages(topics=['/p10/corridor01/raw_timed_points']):
            row=by_seconds.get(message.header.stamp.to_sec())
            if row is not None:
                found.append(dict(row,scan_ns=str(message.header.stamp.to_nsec())))
    if len(found)!=8 or len({row['scan_ns'] for row in found})!=8:
        raise ValueError('EXACT_SCAN_IDENTITY_NOT_ESTABLISHED')
    cache.mkdir();(cache/'sources').mkdir()
    requests=[row['frame_index']+':'+row['scan_ns'] for row in found]
    extra=dict(experimental_ndt_source_capture_enable=True,
               experimental_ndt_source_capture_directory=str(cache/'sources'),
               experimental_ndt_source_capture_requests=requests)
    config['output'].update(extra)
    configuration=cache/'capture_config.yaml'
    configuration.write_text(yaml.safe_dump(config,sort_keys=False))
    check=yaml.safe_load(configuration.read_text())
    for key in extra:check['output'].pop(key)
    if check!=yaml.safe_load(baseline.read_text()):raise ValueError('CONFIG_CONTRACT_FAILED')
    assets[selection]=sha(selection)
    result=dict(TASK='P23-R1-BUDGETED-COUPLED-SUBSPACE-REFINEMENT',START_SHA='805515a59df230f0742c91543c42dd3600b16bd3',
        GT_LOADED=False,files_sha256={str(path):digest for path,digest in assets.items()},
        selection=found,capture_requests=requests,capture_config=str(configuration),
        capture_config_sha256=sha(configuration),configuration_changes=list(extra),
        pose_file_layout='little-endian float64 row-major: previous_used,initial_guess,imu_rotation_prior,raw_ndt,final_used')
    (cache/'input_freeze.json').write_text(json.dumps(result,indent=2)+'\n')
    print(json.dumps(dict(capture_requests=requests,GT_LOADED=False),indent=2))


if __name__=='__main__':
    parser=argparse.ArgumentParser();parser.add_argument('--repo',type=Path,required=True)
    prepare(**vars(parser.parse_args()))
