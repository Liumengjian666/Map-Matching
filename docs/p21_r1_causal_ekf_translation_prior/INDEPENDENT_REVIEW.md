# Independent read-only verification

Two bounded fresh-context reviews were performed under the doubt/review skills.
The user selected single-model review only for both; no external CLI invoked.
No reviewer edited runtime code, ran ROS, rebuilt binaries, or added experiments.

Interface findings addressed before the runs: validate publisher/frame/finite
quaternion and pose before watermark advancement; reject duplicate/reversed
samples; refuse pending overflow; distinguish callback wall time from receipt
ROS time; refuse unavailable per-scan diagnostics when P21 enabled. A later
bounded reread found no remaining substantive issue in these fixes.

After the output freeze, an independent numerical review confirmed:

- All86 pre-GT freeze hashes pass.
- All864 scalar metrics and24102 per-sample error records reproduce exactly.
- Original5s persistence and same-history metrics reproduce exactly.
- 8004 received high-rate messages exactly match the recorded output messages.
- IMU-lineage→LiDAR sample differences: position7.133128933837889e-15m,
  rotation1.4605768126962264e-14deg, velocity9.745330031231542e-15m/s.
- Header ns versus serialized double state timestamp differs by at most
  2.384185791015625e-7s. Selection/age uses integer ns, not this rounded field.
  Minimum actual selected age is0.02003ms; no future state is hidden by this
  representation residual.
- All226 selections are the latest received sample<=target. Independent
  reproduction of the existing constant-body-twist helper has maximum
  prediction difference2.7755575615628914e-17m/1.03278269009744e-14deg.
- Actual initial translation equals recorded prediction translation exactly.
- Primary cohorts:121 visual/195 EKF/0 history NDT samples;
  high-rate2440/3956/0. Empty groups are reported empty.

The automatic gate's lineage check only matches time and accepted-observation
provenance; it does not itself numerically reconstruct pose/twist/prediction.
The supplemental independent calculation above closes that evidence gap for
this actual run. It is not a claim that the gate rejects every hypothetical
corrupted dataset. No frozen analysis or runtime was changed after GT.

## Exact already-executed supplemental numerical check

This is an offline verification command, not a second runtime predictor or
an alternative algorithm. It opens no GT and performs no ROS playback.
Run at the actual research workspace, with the original local output bag
whose hash is in pre_gt_freeze.json. Maximum values above are its actual
exit0 output.

```bash
PYTHONDONTWRITEBYTECODE=1 /usr/bin/python3 - <<'PY'
import pathlib,csv,json,sys,numpy as np,yaml,rosbag
from scipy.spatial.transform import Rotation
repo=pathlib.Path.cwd()
run=repo/'.p21_r1_causal_ekf_translation_prior/runs/full_ekf'
def rows(p):
    with p.open() as f:return list(csv.DictReader(f))
samples=rows(run/'ndt_diagnostics.csv.ekf_samples.csv')
states=[r for r in rows(run/'ekf_lineage.csv') if r['event_type']=='IMU_HIGH_RATE_PUBLISH']
diag=rows(run/'ndt_diagnostics.csv')
params=yaml.safe_load((run/'rosparams.yaml').read_text())
Ril=Rotation.from_matrix(np.array(params['deskew']['T_imu_lidar_rotation_row_major']).reshape(3,3)).as_matrix()
til=np.array(params['deskew']['T_imu_lidar_translation_m'])
max_position=max_angle=max_v=0.;sample_by_ns={int(s['sample_ns']):s for s in samples};max_lineage_time=0.
for s,st in zip(samples,states):
    assert s['publisher']=='/dog_prior_map_ekf' and s['frame_id']==params['frames']['map_frame'] and s['child_frame_id']==params['frames']['base_frame'] and s['status']=='ACCEPTED_CURRENT_EKF_SAMPLE'
    p=np.array([float(st[k]) for k in ['p_x','p_y','p_z']]);R=Rotation.from_quat([float(st[k]) for k in ['q_x','q_y','q_z','q_w']]).as_matrix()
    pl=np.array([float(s[k]) for k in ['tx','ty','tz']]);Rl=Rotation.from_quat([float(s[k]) for k in ['qx','qy','qz','qw']]).as_matrix()
    vchild=np.array([float(s[k]) for k in ['vx','vy','vz']]);wchild=np.array([float(s[k]) for k in ['wx','wy','wz']]);v=np.array([float(st[k]) for k in ['v_x','v_y','v_z']])
    max_position=max(max_position,float(np.linalg.norm(pl-(p+R@til))))
    max_angle=max(max_angle,float(np.degrees(Rotation.from_matrix((R@Ril).T@Rl).magnitude())))
    max_v=max(max_v,float(np.linalg.norm((Rl@vchild-R@np.cross(Ril@wchild,til))-v)))
    max_lineage_time=max(max_lineage_time,abs(int(s['sample_ns'])/1e9-float(st['state_stamp'])))
recorded={}
with rosbag.Bag(str(run/'outputs.bag')) as b:
    for _,m,_ in b.read_messages(topics=['/dog_livo/odom_high_rate']):
        ns=m.header.stamp.to_nsec();assert ns not in recorded;recorded[ns]=m
assert set(recorded)==set(sample_by_ns)
for ns,m in recorded.items():
    s=sample_by_ns[ns]
    actual=[m.pose.pose.position.x,m.pose.pose.position.y,m.pose.pose.position.z,m.pose.pose.orientation.x,m.pose.pose.orientation.y,m.pose.pose.orientation.z,m.pose.pose.orientation.w,m.twist.twist.linear.x,m.twist.twist.linear.y,m.twist.twist.linear.z,m.twist.twist.angular.x,m.twist.twist.angular.y,m.twist.twist.angular.z]
    assert actual==[float(s[k]) for k in ['tx','ty','tz','qx','qy','qz','qw','vx','vy','vz','wx','wy','wz']]
max_pred_position=max_pred_angle=0.;checked=0
for d in diag:
    if d['ekf_translation_applied']!='1':continue
    ns,target=int(d['ekf_sample_ns']),int(d['ekf_target_ns']);s=sample_by_ns[ns];dt=(target-ns)/1e9
    R=Rotation.from_quat([float(s[k]) for k in ['qx','qy','qz','qw']]).as_matrix();p=np.array([float(s[k]) for k in ['tx','ty','tz']]);v=np.array([float(s[k]) for k in ['vx','vy','vz']]);w=np.array([float(s[k]) for k in ['wx','wy','wz']]);phi=w*dt;angle=np.linalg.norm(phi);x,y,z=phi;hat=np.array([[0,-z,y],[z,0,-x],[-y,x,0]])
    if angle<1e-6:J=np.eye(3)+.5*hat+hat@hat/6;dR=np.eye(3)+hat+.5*hat@hat
    else:J=np.eye(3)+(1-np.cos(angle))/(angle*angle)*hat+(angle-np.sin(angle))/(angle**3)*hat@hat;dR=Rotation.from_rotvec(phi).as_matrix()
    expected_p=p+R@J@(v*dt);expected_R=R@dR;actual_p=np.array([float(d['ekf_prediction_'+k]) for k in ['tx','ty','tz']]);actual_R=Rotation.from_quat([float(d['ekf_prediction_'+k]) for k in ['qx','qy','qz','qw']]).as_matrix()
    max_pred_position=max(max_pred_position,float(np.linalg.norm(actual_p-expected_p)));max_pred_angle=max(max_pred_angle,float(np.degrees(Rotation.from_matrix(expected_R.T@actual_R).magnitude())))
    initial=np.array([float(d['initial_guess_'+k]) for k in ['tx','ty','tz']]);assert np.array_equal(initial,actual_p)
    available=[s for s in samples if int(s['sample_ns'])<=target and float(s['callback_wall_sec'])<=float(d['ekf_application_wall_sec'])];assert ns==max(int(s['sample_ns']) for s in available)
    checked+=1
assert max_position<1e-9 and max_angle<1e-8 and max_v<1e-9 and max_pred_position<1e-9 and max_pred_angle<1e-8
print('PROVENANCE_NUMERIC_CLOSURE',dict(received_and_recorded_messages=len(samples),max_sample_lineage_position_m=max_position,max_sample_lineage_rotation_deg=max_angle,max_sample_lineage_velocity_mps=max_v,max_sample_lineage_time_sec=max_lineage_time,applications_checked=checked,max_prediction_translation_m=max_pred_position,max_prediction_rotation_deg=max_pred_angle))
PY
```
