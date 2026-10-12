"""One P21 input freeze; no sensor extraction and no GT."""
import argparse
import json
import socket
import sys
import time
from pathlib import Path
import psutil
import yaml

sys.path.insert(0,str(Path(__file__).resolve().parent.parent/'p17_visual'))
from prepare import sha


def freeze(repo):
    repo=repo.resolve();cache=repo/'.p21_r1_causal_ekf_translation_prior'
    if cache.exists():raise ValueError('P21_CACHE_EXISTS')
    config=repo/'src/dog_prior_map_localization/config/p18_corridor01_ssd_map.yaml'
    new=repo/'src/dog_prior_map_localization/config/p21_corridor01_ekf_translation.yaml'
    a,b=[yaml.safe_load(p.read_text()) for p in (config,new)]
    if b['lidar_update'].pop('experimental_ekf_translation_fallback_enable') is not True or a!=b:
        raise ValueError('CONFIG_CONTRACT_FAILED')
    assets={Path(a['map']['pcd_fallback_path']):'103a01b2c89ca2adbd2b2e22256acba6e91f40b1028857c2103f08c6c3eb8f8f',
        repo/'.p10_corridor01_deterministic_oosm_replay_r1/input_v2/corridor01_p10_causal_startup5_eval35_fixedlag_v2.bag':'7f72f808a9f2282c65705e7e1e731061bb2cc2950329e46da768608e4a7d552e',
        repo/'.p17_visual_aided_ndt_r1/visual_input/visual.csv':'10869519890a5adcb6f3310bba05f379a959018cf123ea8a90c1cf4ea1314c79',
        config:'dc0850f91b43a4afed2e692644ac292db96a599e33fdafb414f3a986bc743220',
        repo/'devel/lib/dog_prior_map_localization/dog_prior_map_ekf_node_cpp':'fbc1750dc605a7fd3b9549b310394fbff3a5781cad2272ef55b9f49df8c3435a'}
    for path,digest in assets.items():
        if sha(path)!=digest:raise ValueError('FROZEN_INPUT_CHANGED:'+str(path))
    assets[new]=sha(new)
    ndt=repo/'devel/lib/dog_prior_map_localization/dog_prior_map_ndt_node_cpp';assets[ndt]=sha(ndt)
    for proc in psutil.process_iter(['name']):
        if proc.info['name'] in ('dog_prior_map_ndt_node_cpp','dog_prior_map_ekf_node_cpp'):
            raise ValueError('RESIDUAL_LOCALIZATION_PROCESS')
    with socket.socket() as s:s.bind(('127.0.0.1',11387))
    cache.mkdir()
    result=dict(TASK='P21-R1-CAUSAL-EKF-TRANSLATION-PRIOR-ABLATION',START_SHA='98c08420267d9a6dd969e850f3a5f9e640c47cb5',
        GT_LOADED=False,wall_sec=time.time(),files_sha256={str(p):h for p,h in assets.items()},
        config_difference='lidar_update.experimental_ekf_translation_fallback_enable: absent/default false -> true',
        beta=.6,prediction_max_age_ms=20.,new_ndt_sha256=sha(ndt))
    (cache/'input_freeze.json').write_text(json.dumps(result,indent=2)+'\n');print(json.dumps(result,indent=2))


if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--repo',type=Path,required=True);freeze(**vars(p.parse_args()))
