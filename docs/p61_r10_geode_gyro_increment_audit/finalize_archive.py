#!/usr/bin/python3
"""Identity comparison and R10 archival checksums; no numerical re-estimation."""
import json,subprocess
from check_assets import HERE,REPO,sha


def main():
    before=json.loads((HERE/'asset_gate_before.json').read_text())
    after=json.loads((HERE/'asset_gate_after.json').read_text())
    def identity(x):return [(a['path'],a['size_bytes'],a['sha256'],a['status']) for a in x['assets']]
    assert before['status']==after['status']=='PASS' and identity(before)==identity(after)
    pin=json.loads((HERE/'inputs_frozen.json').read_text());results=json.loads((HERE/'numerical_results.json').read_text())
    assert sha(HERE/'PRE_REGISTRATION.md')==pin['pre_registration_sha256']
    assert sha(HERE/'run_audit.py')==results['script_sha256']
    comparison=dict(status='PASS',comparisons=len(before['assets']),unique_paths=len({a['path'] for a in before['assets']}),
        all_original_assets_unchanged=True,before_sha256=sha(HERE/'asset_gate_before.json'),after_sha256=sha(HERE/'asset_gate_after.json'))
    (HERE/'asset_gate_comparison.json').write_text(json.dumps(comparison,indent=2)+'\n')
    (HERE/'results.json').write_text(json.dumps(dict(task='P61-R10-GEODE-INDEPENDENT-GYRO-ANGULAR-INCREMENT-CHECK',
        numerical=results,asset_gate=comparison,push_executed=False,
        limitation='Frozen-bias angular increment consistency only; no absolute pose/translation/basin certification.'),indent=2)+'\n')
    products=sorted(p for p in HERE.rglob('*') if p.is_file() and '__pycache__' not in p.parts and p.suffix!='.pyc'
        and p.name not in ('analysis_manifest.json','checksums.sha256'))
    manifest=dict(task='P61-R10-GEODE-INDEPENDENT-GYRO-ANGULAR-INCREMENT-CHECK',
        branch='research/p61-r10-geode-gyro-increment-audit',start_sha=pin['start_sha'],
        actual_analysis_code_sha=results['code_sha'],packaging_parent_head=subprocess.check_output(['git','rev-parse','HEAD'],cwd=REPO,text=True).strip(),
        result=results['result'],scope='Exactly5154->5155 through5164->5165,11 intervals;5154 predecessor only',
        inputs_frozen_sha256=sha(HERE/'inputs_frozen.json'),input_assets=before['assets'],
        scripts={str(p.relative_to(HERE)):sha(p) for p in products if p.suffix=='.py'},
        outputs={str(p.relative_to(HERE)):dict(sha256=sha(p),size_bytes=p.stat().st_size) for p in products},
        source_units='gyro_radps in external imu_link; exact integer ROS epoch ns; Raw M_L converted to M_I',
        bias='R6 frozen initial bias, fullprecision, unchanged; later drift unverified',
        gt_reads=0,ros_localization_nodes=0,rosbag_play=0,new_ndt_align=0,ekf_runs=0,new_localization_trajectory_estimates=0,
        provenance='Existing Raw poses; inspected frozen IMU; new interval rotations/residuals, not a new trajectory',
        push_executed=False,checksum_convention='manifest excludes itself/checksums; checksums includes manifest but not itself')
    (HERE/'analysis_manifest.json').write_text(json.dumps(manifest,indent=2)+'\n')
    products.append(HERE/'analysis_manifest.json')
    (HERE/'checksums.sha256').write_text(''.join(sha(p)+'  '+str(p.relative_to(HERE))+'\n' for p in sorted(products)))
    print(json.dumps(comparison,indent=2));print('R10 checksum files:',len(products))


if __name__=='__main__':main()
