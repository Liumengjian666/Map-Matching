#!/usr/bin/env python3
"""Collect the already-frozen three projections; plot only, no NN/optimization."""
import argparse
import csv
import hashlib
import json
from pathlib import Path
import shutil

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np
import open3d as o3d
from scipy.spatial.transform import Rotation

HERE=Path(__file__).resolve().parent
WS=HERE.parents[1]


def save(name,value):
    with (HERE/name).open('x') as stream:
        json.dump(value,stream,indent=2,allow_nan=False);stream.write('\n')


def write_csv(name,rows):
    with (HERE/name).open('x',newline='') as stream:
        writer=csv.DictWriter(stream,fieldnames=list(rows[0]),lineterminator='\n')
        writer.writeheader();writer.writerows(rows)


def main(cache):
    frozen=json.loads((cache/'input_fixed/input_freeze.json').read_text())
    if [s['target_s'] for s in frozen['samples']]!=[67,100,150]:raise ValueError('FRAME_BUDGET')
    old=json.loads((WS/'docs/corridor01_gt_frame_initialization/transforms.json').read_text())
    M_O=np.array(old['matrices']['T_M_L0']['values'])
    E=np.array(frozen['T_I_L_frozen_proper'])
    save('frame_matrices.json',dict(T_I_L_official_original=old['matrices']['T_I_L_original'],
        T_I_L_runtime_proper=E.tolist(),T_L_I_runtime_proper=np.linalg.inv(E).tolist(),
        T_M_O_engineering_definition=M_O.tolist(),T_O_M=np.linalg.inv(M_O).tolist(),
        T_yaml_direction_UNCONFIRMED=frozen['T_yaml_unknown_semantics'],
        T_M_G_certified=None,PCD_VIEWPOINT_metadata_only=old['map_parity']['raw_viewpoint']))
    shutil.copyfile(cache/'input_fixed/input_freeze.json',HERE/'input_freeze.json')
    raw_map=np.asarray(o3d.io.read_point_cloud(frozen['target_map']).points)
    rows,resources=[],[]
    fig,axes=plt.subplots(3,2,figsize=(13,14))
    map_plot=raw_map[::4]
    for i,sample in enumerate(frozen['samples']):
        root=cache/'input_fixed'/str(sample['target_s'])/'geometry'
        with (root/'geometry.csv').open() as stream:pair=list(csv.DictReader(stream))
        if len(pair)!=2 or (root/'ndt.csv').exists():raise ValueError('QUERY_BUDGET')
        points=np.fromfile(root/'preprocessed_source.xyz',dtype='<f4').reshape(-1,3)
        if len(points)!=1400:raise ValueError('SOURCE_POINTS')
        for which,item in enumerate(pair):
            expected=np.array(sample['H_A_conditional' if which==0 else 'H_B_conditional'])
            actual=np.array([float(item[f'T{r}{c}']) for r in range(4) for c in range(4)]).reshape(4,4)
            if not np.allclose(expected,actual,rtol=0,atol=1e-14):raise ValueError('HYPOTHESIS_CHANGED')
            projected=points@actual[:3,:3].astype('f4').T+actual[:3,3].astype('f4')
            rows.append(dict(target_s=sample['target_s'],raw_frame_index=sample['raw_frame_index'],scan_ns=sample['scan_ns'],
                hypothesis='A_GT_LIDAR_G_EQ_M' if which==0 else 'B_GT_IMU_G_EQ_M',
                **{k:v for k,v in item.items() if k!='hypothesis'}))
            for col,(d0,d1) in enumerate([(0,1),(0,2)]):
                ax=axes[i,col]
                if which==0:ax.scatter(map_plot[:,d0],map_plot[:,d1],s=.2,c='.65',label='Original PCD XYZ')
                ax.scatter(projected[:,d0],projected[:,d1],s=2,c=['tab:blue','tab:red'][which],
                    alpha=.7,label=['A: GT directly','B: GT * T_I_L'][which])
                ax.set_xlabel(['X','Y','Z'][d0]+' (m)');ax.set_ylabel(['X','Y','Z'][d1]+' (m)')
                ax.set_title(f'raw{sample["raw_frame_index"]}, +{sample["relative_header_s"]}s, G=M condition')
                ax.axis('equal');ax.grid(alpha=.2)
        text=(root/'resources.txt').read_text()
        def value(prefix):
            return next(line.strip().rsplit(': ',1)[1] for line in text.splitlines() if line.strip().startswith(prefix))
        wall=value('Elapsed (wall clock)');minute,second=wall.split(':')
        resources.append(dict(target_s=sample['target_s'],NN_queries=2,NDT_aligns=0,ICP_calls=0,
            wall_s=60*int(minute)+float(second),CPU_user_s=value('User time'),CPU_system_s=value('System time'),
            peak_RSS_KiB=value('Maximum resident set size')))
    for ax in axes[:,0]:ax.legend(markerscale=3,fontsize=7)
    fig.suptitle('P33: direct fixed GT hypotheses versus released original map; no alignment or optimization')
    fig.tight_layout(rect=[0,0,1,.98]);fig.savefig(HERE/'three_frame_projection.png',dpi=130);plt.close(fig)
    if len(rows)!=6:raise ValueError('TOTAL_QUERY_BUDGET')
    write_csv('direct_projection.csv',rows);write_csv('runtime_summary.csv',resources)
    exact=np.array(frozen['GT_at_exact_first_LiDAR_header_plus67']['T_G_GT'])
    Y=np.array(frozen['T_yaml_unknown_semantics'])
    save('yaml_time_comparison.json',dict(exact_sensor_header_origin_plus67=frozen['GT_at_exact_first_LiDAR_header_plus67'],
        same_world_condition_only=True,
        exact_A_translation_difference_m=float(np.linalg.norm(Y[:3,3]-exact[:3,3])),
        exact_A_rotation_difference_deg=float(Rotation.from_matrix(Y[:3,:3].T@exact[:3,:3]).magnitude()*180/np.pi),
        exact_B_translation_difference_m=float(np.linalg.norm(Y[:3,3]-(exact@E)[:3,3])),
        exact_B_rotation_difference_deg=float(Rotation.from_matrix(Y[:3,:3].T@(exact@E)[:3,:3]).magnitude()*180/np.pi),
        nearest_scan_comparison=frozen['YAML_vs_GT_near_sensor67_comparison_NOT_semantic_proof']))
    save('results.json',dict(task='P33-CORRIDOR01-COMPLETE-FRAME-RELATION',
        FINAL_RESULT='FRAME_RELATION_UNRESOLVED_DIRECT_GT_MAP_IDENTITY_NOT_SUPPORTED',
        START_SHA='af7f2873f72a0ed9d0048bebedde5ec18ab2e603',END_SHA=None,
        END_SHA_note='Resolve archive commit with git log -1 --format=%H -- docs/p33_corridor01_complete_frame_relation; final full SHA in chat',
        branch='research/p9-r4-heldout-visual-evidence',PUSH_EXECUTED=False,
        map_SHA256=frozen['target_map_SHA256'],GT_SHA256=frozen['GT_SHA256'],
        runtime_source_changed=False,algorithm_parameters_changed=False,original_data_changed=False,
        direct_GT_map_identity_supported_in_H_A_or_H_B=False,GT_sensor_reference_certified=False,
        world_darpa_direction_certified=False,comment_s67_reference_certified=False,
        official_paper_supports_map_constrained_GT_generation=True,
        released_files_same_export_frame_certified=False,
        scans_projected=3,NN_queries=6,NDT_align_calls=0,ICP_calls=0,localization_runs=0,
        sources_match_real_decoded_bag=True,unit_tests_passed=3,
        map_bounds_m=dict(min=raw_map.min(0).tolist(),max=raw_map.max(0).tolist()),
        source_mode='RAW_HEADER_REFERENCE_NO_DESKEW',projection_rows=rows,
        official_GT_numeric_lineage_inherited_from_P34=True,
        large_input_SHA_recomputed=False,small_input_SHA_recomputed=True,
        resources=resources,external_CLI_status='PENDING_EXACT_COMMAND_CONFIRMATION',
        NEXT='Obtain the author export/frame binding for the exact PCD and TUM files, including world_darpa and # s 67 meaning; no additional search authorized'))
    print(json.dumps(dict(scans=3,NN_queries=6,NDT=0,ICP=0,all_coverage_8m_zero=all(float(r['coverage_8m'])==0 for r in rows),
        map_bounds=[raw_map.min(0).tolist(),raw_map.max(0).tolist()],resources=resources),indent=2))


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('--cache',type=Path,required=True)
    main(**vars(parser.parse_args()))
