"""Archive actual lightweight P23 evidence; leave all point clouds/bags local."""
import argparse
import csv
import json
import shutil
from pathlib import Path
import numpy as np
from prepare_capture import sha


def refresh_hashes(output):
    hashes={str(path.relative_to(output)):sha(path) for path in output.rglob('*')
            if path.is_file() and path.name!='artifact_hashes.json'}
    (output/'artifact_hashes.json').write_text(json.dumps(hashes,indent=2)+'\n')
    return hashes


def archive(repo):
    repo=repo.resolve();cache=repo/'.p23_r1_budgeted_coupled_subspace_refinement'
    output=repo/'docs/p23_r1_budgeted_coupled_subspace_refinement'
    if (output/'results.json').exists():raise ValueError('ARCHIVE_EXISTS')
    inventory=[]
    def copy(source,target):
        if target.exists():raise ValueError('ARCHIVE_TARGET_EXISTS:'+str(target))
        target.parent.mkdir(parents=True,exist_ok=True);shutil.copyfile(source,target)
        inventory.append(dict(source=str(source),archive=str(target.relative_to(output)),sha256=sha(source)))
    for name in ('input_freeze.json','runtime_input_freeze.json','offline_input_freeze.json','pre_gt_freeze.json','capture_config.yaml'):
        copy(cache/name,output/name)
    for name in ('source_capture_summary.csv','source_hash_parity.csv','baseline_parity.csv'):
        copy(cache/'capture_gate'/name,output/name)
    copy(cache/'capture_gate/results.json',output/'capture_results.json')
    for source in (cache/'offline').iterdir():
        if source.suffix in ('.csv','.json','.log','.txt'):copy(source,output/'offline'/source.name)
    for name in ('nominal_parity.csv','weak_geometry.csv','weak_candidates.csv','coupled_correction.csv',
                 'ndt_paired_results.csv','runtime_breakdown.csv','resource_summary.csv'):
        copy(cache/'offline'/name,output/name)
    for name in ('per_frame_accuracy.csv','accuracy_comparison.csv','step_limit_response.csv','results.json'):
        copy(cache/'posthoc_evaluation'/name,output/name)
    for name in ('ndt_diagnostics.csv','ekf_lineage.csv','deskew.csv','oosm.csv','oosm_fixed_lag.csv',
                 'rosparams.yaml','execution.json','resources.csv','ekf_runtime.csv','parameter_parity.json','launch.log'):
        copy(cache/'runs/capture_visual'/name,output/'runs/capture_visual'/name)
    records=[]
    for source in sorted((cache/'sources').glob('*.poses.f64')):
        frame=source.name.split('_')[1].split('.')[0]
        matrices=np.fromfile(source,dtype='<f8').reshape(5,4,4)
        for label,matrix in zip(('previous_used','initial_guess','imu_rotation_prior','raw_ndt','final_used'),matrices):
            records.append(dict(frame_index=frame,stage=label,
                **{'m'+str(i)+str(j):matrix[i,j] for i in range(4) for j in range(4)}))
    with (output/'captured_pose_matrices.csv').open('w',newline='') as stream:
        writer=csv.DictWriter(stream,fieldnames=list(records[0]),lineterminator='\n')
        writer.writeheader();writer.writerows(records)
    # Inventory includes raw local inputs/results WITHOUT copying them into Git.
    local_files=[path for folder in (cache/'sources',cache/'runs/capture_visual')
                 for path in folder.iterdir() if path.is_file()]
    (output/'local_artifact_inventory.json').write_text(json.dumps(
        {str(path):dict(bytes=path.stat().st_size,sha256=sha(path)) for path in local_files},indent=2)+'\n')
    (output/'source_inventory.json').write_text(json.dumps(inventory,indent=2)+'\n')
    hashes=refresh_hashes(output)
    print(json.dumps(dict(archived_files=len(hashes),raw_clouds_committed=0,rosbags_committed=0)))


if __name__=='__main__':
    parser=argparse.ArgumentParser();parser.add_argument('--repo',type=Path,required=True)
    parser.add_argument('--hash-only',action='store_true')
    args=parser.parse_args()
    if args.hash_only:
        print('ARCHIVE_MANIFEST_REFRESH',len(refresh_hashes(args.repo/'docs/p23_r1_budgeted_coupled_subspace_refinement')))
    else:archive(args.repo)
