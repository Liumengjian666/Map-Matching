#!/usr/bin/env python3
"""Archive frozen bridge and one raw geometry probe without GT or pose alignment."""
import collections
import csv
import json
from pathlib import Path
import shutil
import numpy as np
from scipy.spatial.transform import Rotation
from prepare_bridge import ROOT,MAP,MAP_SHA,TEST,sha

HERE=Path(__file__).resolve().parent


def rows(path):
    with path.open() as f:return list(csv.DictReader(f))


def pcd_xyz(path):
    header={}
    with path.open('rb') as f:
        while True:
            line=f.readline().decode('ascii').strip()
            if line.startswith('#'):continue
            k,*v=line.split();header[k]=v
            if k=='DATA':break
        assert header['DATA']==['binary'] and set(header['SIZE'])=={'4'} and set(header['TYPE'])=={'F'}
        values=np.frombuffer(f.read(),dtype=np.dtype([(k,'<f4') for k in header['FIELDS']]))
    assert len(values)==int(header['POINTS'][0])
    return np.stack([values[k] for k in ['x','y','z']],axis=1)


def write_xyz(path,xyz):
    with path.open('xb') as f:
        f.write(f'VERSION 0.7\nFIELDS x y z\nSIZE 4 4 4\nTYPE F F F\nCOUNT 1 1 1\nWIDTH {len(xyz)}\nHEIGHT 1\nPOINTS {len(xyz)}\nDATA binary\n'.encode());f.write(np.asarray(xyz,dtype='<f4').tobytes())


def main():
    bridge=ROOT/'bridge_run';probe=ROOT/'ndt_probe';view=ROOT/'visualization';view.mkdir(exist_ok=False)
    frozen=json.loads((bridge/'initialization_freeze.json').read_text())
    assert sha(MAP)==MAP_SHA
    assert sha(bridge/'initial_state.json')==frozen['initial_state_sha256']
    assert sha(bridge/'initial_covariance.csv')==frozen['covariance_sha256']
    gate=json.loads((bridge/'r5_state_consistency.json').read_text());assert gate['pass_gate']
    trajectory=rows(bridge/'mapping_trajectory.csv');replay=rows(bridge/'replay.csv')
    assert len(trajectory)==len(replay)==2984
    for r,p in zip(trajectory,replay):
        assert r['sequence']==p['sequence']
        r['scan_start_ns']=p['scan_start_ns'];r['raw_point_end_ns']=p['raw_point_end_ns']
    with (HERE/'state_bridge_trajectory.csv').open('w',newline='') as f:
        w=csv.DictWriter(f,fieldnames=list(trajectory[0]),lineterminator='\n');w.writeheader();w.writerows(trajectory)
    ndt=rows(probe/'ndt.csv')[0];geometry=rows(probe/'geometry.csv');hessian=rows(probe/'hessian.csv')[0]
    assert int(ndt['align_calls'])==1
    initial=np.array(frozen['initial_state']['T_M_L']).reshape(4,4)
    final=np.array([float(ndt[f'final{i}{j}']) for i in range(4) for j in range(4)]).reshape(4,4)
    xyz=pcd_xyz(probe/'preprocessed_source.pcd');map_xyz=pcd_xyz(MAP)
    before=xyz@initial[:3,:3].T+initial[:3,3];after=xyz@final[:3,:3].T+final[:3,3]
    write_xyz(view/'initial_raw_projected.pcd',before);write_xyz(view/'ndt_raw_projected.pcd',after)
    imu_manifest=json.loads((ROOT/'bridge_input.json').read_text())
    resources=json.loads((bridge/'mapper_resources.json').read_text())
    runtime=dict(bridge_wall_sec=frozen['bridge_wall_sec'],bridge_peak_rss_mib=resources['peak_rss_kib']/1024,
        bridge_user_cpu_sec=resources['user_cpu_sec'],bridge_system_cpu_sec=resources['system_cpu_sec'],
        source_tree_temporary_map_points=resources['exported_points'],
        native_ndt_align_ms=float(ndt['align_ms']),target_build_ms=float(ndt['target_build_ms']),
        diagnostic_jet_ms=float(hessian['jet_ms']),jet_calls=int(hessian['jet_calls']),align_calls=1,
        ndt_peak_rss_kib=None,ndt_memory_measurement_source='probe_resource.txt (/usr/bin/time -v)')
    for line in (ROOT/'probe_resource.txt').read_text().splitlines():
        if 'Maximum resident set size' in line:runtime['ndt_peak_rss_kib']=int(line.split(':')[-1].strip())
    runtime['ndt_peak_rss_mib']=runtime['ndt_peak_rss_kib']/1024
    actual_success=ndt['converged']=='1' and ndt['pose_valid']=='1'
    result=dict(FINAL_RESULT='CAUSAL_INIT_ESTABLISHED' if actual_success else 'CAUSAL_INIT_ESTABLISHED_BUT_NDT_FAILED',
        START_SHA='610f74e6503edad2c3bbf4eb865f0076b0d447eb',
        CODE_SHA='0532c3b2d882327c7f22f57a53add6c7ceb11983',
        NDT_PROBE_CODE_SHA='8751df36f19d4a72e09f9602afe603973028bd6b',
        PUSH_EXECUTED=False,map_sha256=MAP_SHA,map_points=len(map_xyz),map_modified=False,
        gate_A=True,gate_B=True,gate_C_raw_single_frame_diagnostic=True,
        r5_state_consistency=gate,initialization=frozen,
        bridge_lidar_including_mapping_boundary_scan=543,
        strictly_middle_complete_lidar=542,extra_mapping_boundary_scan=2442,
        strictly_middle_imu_messages=imu_manifest['counts']['/imu/data']-24418,
        newly_delivered_imu_after_r5_state=frozen['imu_delivered_after_r5_state'],
        stages=dict(collections.Counter(r['stage'] for r in trajectory)),
        ndt_smoke_frames=1,ndt_converged_frames=int(actual_success),ndt_failed_frames=int(not actual_success),
        diagnostic_raw_not_deskewed=True,diagnostic_geometry=geometry,ndt=ndt,hessian=hessian,runtime=runtime,
        fixed_50_coverage_status='POINTCLOUD_COVERAGE_NOT_CERTIFIED',
        fixed_50_same_time_map_poses=0,gt_read=False,initialization_uses_test_lidar=False,
        online_ndt_feedback=0,coupled_refinement_calls=0,recovery_calls=0,
        independent_cross_traversal_experiment=False,
        next_proposal='Separately approved 20-frame Deskew+CONTROL NDT+EKF/OOSM baseline smoke from frozen initialization; no automatic full return run.')
    (HERE/'results.json').write_text(json.dumps(result,indent=2)+'\n')
    (HERE/'runtime_statistics.json').write_text(json.dumps(runtime,indent=2)+'\n')
    for src,name in [(bridge/'initialization_freeze.json','initialization.json'),(bridge/'initial_covariance.csv','initial_covariance.csv'),
        (bridge/'r5_state_consistency.json','r5_state_consistency.json'),(probe/'ndt.csv','ndt_smoke.csv'),
        (probe/'geometry.csv','geometry_statistics.csv'),(probe/'hessian.csv','hessian_diagnostics.csv'),
        (probe/'grid.csv','gaussian_grid.csv'),(ROOT/'bridge_input.json','input_manifest.json'),
        (ROOT/'probe_input/probe_manifest.json','probe_manifest.json')]:shutil.copyfile(src,HERE/name)
    shutil.copyfile(HERE.parent/'p61_r5_geode_frozen_prior_map/map_coverage.csv',HERE/'fixed_50_coverage.csv')
    q=Rotation.from_matrix(initial[:3,:3]).as_quat();t=initial[:3,3]
    xml=f'''<launch>
  <!-- Frozen raw diagnostic snapshots ONLY; does not run mapping/localization. -->
  <arg name="rviz" default="true" />
  <node pkg="pcl_ros" type="pcd_to_pointcloud" name="r6_frozen_map" args="{MAP} 1">
    <param name="frame_id" value="camera_init"/><remap from="cloud_pcd" to="/r6/frozen_map"/>
  </node>
  <node pkg="pcl_ros" type="pcd_to_pointcloud" name="r6_initial_raw" args="{view/'initial_raw_projected.pcd'} 1">
    <param name="frame_id" value="camera_init"/><remap from="cloud_pcd" to="/r6/initial_raw"/>
  </node>
  <node pkg="pcl_ros" type="pcd_to_pointcloud" name="r6_ndt_raw" args="{view/'ndt_raw_projected.pcd'} 1">
    <param name="frame_id" value="camera_init"/><remap from="cloud_pcd" to="/r6/ndt_raw"/>
  </node>
  <node pkg="tf" type="static_transform_publisher" name="r6_frozen_initial_pose" args="{' '.join(format(float(v),'.17g') for v in [*t,*q])} camera_init r6_initial_lidar 100"/>
  <node if="$(arg rviz)" pkg="rviz" type="rviz" name="r6_snapshot_rviz" args="-d {HERE/'snapshots.rviz'}"/>
</launch>
'''
    (HERE/'review_snapshots.launch').write_text(xml)
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    fig,ax=plt.subplots(1,2,figsize=(12,5))
    center=initial[:3,3];local=map_xyz[np.linalg.norm(map_xyz[:,:2]-center[:2],axis=1)<45]
    for a,j,label in [(ax[0],1,'Y'),(ax[1],2,'Z')]:
        a.scatter(local[:,0],local[:,j],s=.3,c='gray',label='R5 frozen map')
        a.scatter(before[:,0],before[:,j],s=3,c='tab:blue',alpha=.5,label='raw at frozen t0 pose')
        a.scatter(after[:,0],after[:,j],s=3,c='tab:green',alpha=.5,label='one NDT output, no feedback')
        a.scatter([center[0]],[center[j]],s=35,c='red',marker='x',label='causal initializer')
        a.set_xlabel('M X [m]');a.set_ylabel('M '+label+' [m]');a.axis('equal');a.legend(markerscale=2)
    fig.suptitle('RAW single-frame geometry only; no GT / no deskew / no localization accuracy claim')
    fig.tight_layout();fig.savefig(HERE/'initialization_geometry.png',dpi=160);plt.close(fig)
    print('FINAL_RESULT',result['FINAL_RESULT']);print(json.dumps(runtime,indent=2))


if __name__=='__main__':main()
