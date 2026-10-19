#!/usr/bin/python3
"""Freeze approved scopes and immutable existing evidence before attribution."""
import csv,json,hashlib,subprocess
from asset_gate import HERE,REPO,R8,RUN,BASE,sha

PHASES={'main':[5159,5229],'pre':[5140,5157],'transition':[5155,5165],
        'severe':[5169,5175],'recovery':[5220,5240],'secondary':[5035,5045]}
SELECTED=sorted({i for a,b in PHASES.values() for i in range(a,b+1)})
PREDECESSORS=sorted({i-1 for i in SELECTED}-{*SELECTED})


def main():
    gate=json.loads((HERE/'asset_gate_before.json').read_text());assert gate['status']=='PASS'
    with (R8/'scan_manifest.csv').open() as f:scans={int(x['scan_index']):x for x in csv.DictReader(f)}
    source_identity=[]
    files=['src/dog_prior_map_localization/'+p for p in (
      'src/dog_prior_map_ndt_node.cpp','src/imu_deskew_runtime.cpp','src/core/imu_deskew.cpp',
      'src/fusion/ndt_observation.cpp','src/ros_output.cpp','src/imu_processor.cpp',
      'src/dog_prior_map_ekf_node_core.cpp','include/dog_prior_map_localization/core/frame_conversions.hpp')]
    for path in files:
        version='17922f2b63dae1f2af34f353287728d50a312f73'
        content=subprocess.check_output(['git','show',version+':'+path],cwd=REPO)
        digest=hashlib.sha256(content).hexdigest();assert sha(REPO/path)==digest,path
        source_identity.append(dict(path=path,commit=version,sha256=digest,working_copy_identical=True))
    for name in ('prepare_inputs.py','replay_full.py','terminal_contract.py'):
        path='docs/p61_r8_geode_full_return_control/'+name;version='1367e1d9587d1f90a47bf6f44405239a7b65e4f6'
        content=subprocess.check_output(['git','show',version+':'+path],cwd=REPO)
        digest=hashlib.sha256(content).hexdigest()
        source_identity.append(dict(path=path,commit=version,sha256=digest,working_copy_identical=sha(REPO/path)==digest))
    result=dict(R8_archive=BASE,R9_resume_start='aea49024c5da5294b716bc17247713c4fca7b833',
       phases=PHASES,selected_indices=SELECTED,predecessors_only_for_increments=PREDECESSORS,
       selected_count=len(SELECTED),density_radii_m=[2,5],gt_read=False,
       input_assets=gate['assets'],actual_source_identity=source_identity,
       selected_scan_identity=[scans[i] for i in SELECTED],
       no_source_reconstruction=True,no_new_optimizer_calls=True)
    assert len(SELECTED)==112 and PREDECESSORS==[5034,5139]
    with (HERE/'analysis_inputs_frozen.json').open('x') as f:json.dump(result,f,indent=2);f.write('\n')
    print('Immutable input SHA:',sha(HERE/'analysis_inputs_frozen.json'))


if __name__=='__main__':main()
