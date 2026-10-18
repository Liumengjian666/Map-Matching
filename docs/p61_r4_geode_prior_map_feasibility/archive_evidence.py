"""Freeze small official reference assets; never reads or creates a sensor map."""
import hashlib
import json
from pathlib import Path
import shutil
import subprocess

DOC=Path(__file__).resolve().parent
EVIDENCE=Path('/home/jian/livox_ws/p61_r4_geode_evidence')


def archive():
    (DOC/'official').mkdir(exist_ok=False)
    mapping={'gt_download_response.bin':'Shield_tunnel1.txt',
             'drive_gamma_config_response.bin':'gamma_drive.yaml',
             'gamma2GT_leica.py':'gamma2GT_leica.py','rmse.py':'rmse.py',
             'fastlio_carol.yaml':'fastlio_carol.yaml','fastlio_carol.launch':'fastlio_carol.launch',
             'fastlio_test_geode.py':'fastlio_test_geode.py','fastlio_laserMapping.cpp':'fastlio_laserMapping.cpp',
             'evaluation_README.md':'evaluation_README.md',
             'repository_head.json':'repository_head.json','dataset_tree.json':'dataset_tree.json',
             'evaluation_repository.json':'evaluation_repository.json',
             'evaluation_tree.json':'evaluation_wrong_branch_404.json',
             'evaluation_master_tree.json':'evaluation_master_tree.json','map_listing.json':'map_listing.json'}
    sources={}
    for source, target in mapping.items():
        source=EVIDENCE/source; destination=DOC/'official'/target
        with source.open('rb') as src,destination.open('xb') as dst:shutil.copyfileobj(src,dst)
        raw=destination.read_bytes()
        sources[str(destination.relative_to(DOC))]={'source':str(source),'bytes':len(raw),'sha256':hashlib.sha256(raw).hexdigest()}
        receipt=source.with_suffix(source.suffix+'.http.json')
        if receipt.exists():
            sources[str(destination.relative_to(DOC))]['http']=json.loads(receipt.read_text())
    for name in ('time_split_freeze.json','gt_feasibility.json','reference_split_feasibility.png'):
        source=EVIDENCE/'split_analysis'/name
        with source.open('rb') as src,(DOC/name).open('xb') as dst:shutil.copyfileobj(src,dst)
    with (EVIDENCE/'calibration_audit_v2.json').open('rb') as src,(DOC/'calibration_audit.json').open('xb') as dst:
        shutil.copyfileobj(src,dst)
    with (DOC/'source_manifest.json').open('x') as stream:json.dump(sources,stream,indent=2)
    gt=json.loads((DOC/'gt_feasibility.json').read_text())
    result={'task':'P61-R4-GEODE-PRIOR-MAP-FEASIBILITY',
            'final_result':'GEODE_PRIOR_MAP_CONTRACT_NOT_READY',
            'start_sha':'c916eaa7cb8dca831d6722ae6dcd4d47418f8616',
            'code_sha':subprocess.check_output(['git','-C',str(DOC.parents[1]),'rev-parse','HEAD'],text=True).strip(),
            'official_dataset_sha':'c6e930623d4fed450d7fc50e16e3ffe0288b692b',
            'official_evaluation_sha':'1f008a7249e36393a1752622de50660b77b5b7f4',
            'official_gt_downloaded':True,'gt_orientation_available':False,
            'official_tunnel_map_found':False,'map_folder_visible_files':['stairs.las'],
            'position_only_revisit_evidence':True,'actual_lidar_coverage_certified':False,
            'non_gt_mapper_source_available':True,'validated_mapping_trajectory_available':False,
            'frozen_prior_map_available':False,'legal_test_initial_pose_available':False,
            'can_start_continuous_prior_map_ndt':False,
            'primary_blocker':'NO_FROZEN_GT_INDEPENDENT_TUNNEL_PRIOR_MAP_WITH_COVERAGE_AND_MAP_FRAME',
            'mapping_route':'CONDITIONAL_SAME_SEQUENCE_DISJOINT_TIME_WINDOWS_ONLY_NOT_INDEPENDENT_ACQUISITION',
            'gt_feasibility':gt,'gt_used_for_mapping':False,'gt_used_for_initialization':False,
            'gt_trajectory_fit':False,'map_generation_calls':0,'ndt_align_calls':0,'native_jet_calls':0,
            'ros_started':False,'ekf_calls':0,'coupled_calls':0,'recovery_calls':0,
            'raw_bag_read':False,'hikvision_access':False,'push_executed':False}
    with (DOC/'results.json').open('x') as stream:json.dump(result,stream,indent=2)
    print('ARCHIVED',len(sources),'small sources; no map/align/ROS executed')


if __name__=='__main__':archive()
