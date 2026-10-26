#!/usr/bin/env python3
"""Seal completed diagnostics and bounded storage inventory; starts no ROS/NDT."""
import hashlib,importlib.util,json,shutil,subprocess,sys
from pathlib import Path
sys.dont_write_bytecode=True
P=Path(__file__).resolve().parents[1];R=P.parents[1]
def sha(p):
 h=hashlib.sha256()
 with Path(p).open('rb') as f:
  for b in iter(lambda:f.read(1048576),b''):h.update(b)
 return h.hexdigest()
start=json.loads((P/'evidence/START_IDENTITY.json').read_text())
spec=importlib.util.spec_from_file_location('stable',R/'docs/p62_r23_real_imu_closure/tools/prepare_inputs.py');m=importlib.util.module_from_spec(spec);spec.loader.exec_module(m)
after=m.stable_snapshot();assert after==start['stable_snapshot']
binaries={}
for f in [R/'.r30_diagnostic_build/reference_seeded_probe',R/'.r30_diagnostic_build/ndt_grid_order_reproducer',R/'devel/lib/dog_prior_map_localization/dog_prior_map_ndt_node_cpp',R/'devel/lib/dog_prior_map_localization/ndt_target_resolution_contract_test']:
 binaries[str(f)]={'bytes':f.stat().st_size,'sha256':sha(f)}
requested=json.loads((P/'PROBE_PRE_REGISTRATION.json').read_text())['requests']
for item in requested:
 assert sha(item['pcd_path'])==item['pcd_sha256']
 item['matrix_sha256']=sha(item['reference_matrix'])
# The repair must leave all EKF, propagation, OOSM and map-selection files byte-identical.
models=json.loads((R/'docs/p62_r29_full_sequence_benchmark/BUILD_FREEZE.json').read_text())['model_sources_unchanged']
assert all(sha(R/f)==digest for f,digest in models.items())
identity={'start_sha':start['git'],'remote_source_sha':start['remote_head'],'R29_online_code_sha':'f66a55457ba3e63d55ad8d588fb2cf5b1ead169b','diagnostic_probe_code_sha':'e5dffad5b23485078e9ab17fdf4987107913626f','repair_code_sha':'02490a7ee7376baa76abff8495865a3a020c7210','repair_binary_not_used_for_new_trajectory':True,'binaries':binaries,'probe_inputs':requested,'stable_after':after,'stable_matches_before':True,'EKF_IMU_Deskew_OOSM_map_models_unchanged':True,'new_native_align_calls':8,'new_formal_trajectories':0,'new_ROS_EKF_runs':0,'new_sensor_downloads':0}
(P/'evidence/FINAL_IDENTITY.json').write_text(json.dumps(identity,indent=2)+'\n')
paths=[R,R/'build',R/'devel',R/'.r30_diagnostic_build',Path('/home/jian/livox_ws/p62_r29_full_sequence_ws/build'),Path('/home/jian/livox_ws/p62_r29_full_sequence_ws/devel'),Path('/home/jian/rosbag/public_datasets/koide_p62_r29'),Path('/home/jian/livox_ws/p62_r29_full_sequence_data')]
inventory=[]
for p in paths:
 b=int(subprocess.check_output(['du','-B1','-s',str(p)],text=True).split()[0]);inventory.append({'path':str(p),'allocated_bytes':b})
root=shutil.disk_usage(R);ext=shutil.disk_usage('/media/jian/HIKVISION');assert root.free>20*1024**3
storage={'root_free_before_worktree_bytes':35244015616,'root_free_after_bytes':root.free,'observed_free_delta_bytes':root.free-35244015616,'delta_includes_other_filesystem_activity':True,'reserve_bytes':20*1024**3,'external_free_bytes':ext.free,'inventory':inventory,'deleted_files':0,'cleanup_approved':False}
(P/'evidence/STORAGE_AUDIT.json').write_text(json.dumps(storage,indent=2)+'\n')
print('IDENTITY_AND_STORAGE_SEALED',root.free,'bytes free')
