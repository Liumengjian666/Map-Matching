#!/usr/bin/python3
"""Package hashes of R9-only derived artifacts after immutable asset comparison."""
import json,subprocess
from asset_gate import HERE,REPO,sha


def main():
    before=json.loads((HERE/'asset_gate_before.json').read_text())
    after=json.loads((HERE/'asset_gate_after.json').read_text())
    assert before['status']==after['status']=='PASS'
    def identities(snapshot):return [(x['asset'],x['path'],x['size_bytes'],x['sha256'],x['status']) for x in snapshot['assets']]
    assert identities(before)==identities(after)
    assert len(before['assets'])==125
    comparison=dict(status='PASS',comparisons=125,unique_absolute_paths=len({x['path'] for x in before['assets']}),
        all_frozen_assets_unchanged=True,before_sha256=sha(HERE/'asset_gate_before.json'),after_sha256=sha(HERE/'asset_gate_after.json'))
    (HERE/'asset_gate_comparison.json').write_text(json.dumps(comparison,indent=2)+'\n')
    assert sha(HERE/'analysis_inputs_frozen.json')=='2bf69f3273559f28b7aca43853fa275b448734ec46b91429129fd3970c8ac71b'
    products=sorted(p for p in HERE.iterdir() if p.is_file() and p.name not in ('analysis_manifest.json','checksums.sha256'))
    scripts={p.name:sha(p) for p in products if p.suffix=='.py'}
    manifest=dict(task='P61-R9-GEODE-ANOMALY-READONLY-ATTRIBUTION',result='MIXED_OR_INCONCLUSIVE',
        branch='research/p61-r9-geode-anomaly-readonly-attribution',
        original_start_sha='eba814000a06a2b110acf3835247cd6cffe9db6b',resume_start_sha='aea49024c5da5294b716bc17247713c4fca7b833',
        actual_numerical_analysis_code_sha='f1d4ad321b6c77d7305ed42f015f95fc77c59e1f',
        packaging_parent_head=subprocess.check_output(['git','rev-parse','HEAD'],cwd=REPO,text=True).strip(),
        immutable_input_manifest_sha256=sha(HERE/'analysis_inputs_frozen.json'),
        input_file_identities='asset_gate_before.json: assets; analysis_inputs_frozen.json: actual_source_identity and selected_scan_identity',
        actual_scope={'unique_scans':112,'intervals':[[5035,5045],[5140,5240]],'predecessors_only':[5034,5139]},
        asset_gate=comparison,script_sha256=scripts,
        output_files={p.name:dict(sha256=sha(p),size_bytes=p.stat().st_size) for p in products},
        provenance={'original_R8_metrics':'verbatim existing geometry and poses','new_metrics':'SE3 residuals, fixed-radius origin density, time/coordinate diagnostics',
                    'point_time_records':'inspection of frozen pre-deskew raw bytes, not recreated runtime Source',
                    'cause_assessment':'explicit evidence-limited human-readable scientific assessment'},
        external_cli_review_executed=False,external_cli_status='Optional offer sent; no confirmation received at packaging; no external CLI run',
        localization_replays=0,gt_reads=0,new_ndt_aligns=0,new_ekf_runs=0,new_ros_localization_nodes=0,push_executed=False,
        hash_convention='Manifest excludes its own bytes and checksums.sha256; checksums includes manifest but excludes itself.')
    (HERE/'analysis_manifest.json').write_text(json.dumps(manifest,indent=2)+'\n')
    files=sorted(p for p in HERE.iterdir() if p.is_file() and p.name!='checksums.sha256')
    (HERE/'checksums.sha256').write_text(''.join(sha(p)+'  '+p.name+'\n' for p in files))
    print(json.dumps(comparison,indent=2));print('R9 products:',len(files))


if __name__=='__main__':main()
