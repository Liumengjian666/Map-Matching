# P33 bounded adversarial read-only review

Use only this input. Do not invoke tools, shell, filesystem reads, network, agents, or experiments.
Do not change files. Do not ask to rerun data. Find substantive errors or overclaims; do not validate
or provide a flattering summary. If a required fact is missing, identify it without inventing it.
This is a finite coordinate/clock diagnostic, not localization and not an accuracy improvement task.

## CONTRACT

Transform notation: p_A=T_A_B*p_B. M=actual XYZ of original released PCD;
G=released GT global frame; W,D=names in world_darpa; O=current normalized localization frame;
L=decoded ROS LiDAR; I=IMU; B=Body if independently defined.

No NDT/ICP, no 6DoF fit, no rotation/translation search. Exactly three raw scans near sensor
header-origin +67/+100/+150 seconds, selected before NN by unique nearest header (earlier tie).
For each scan test only A=GT directly and B=GT*T_I_L, both explicitly conditional on G=M.
No VIEWPOINT transform; no GT deskew. No world_darpa projection without author code establishing
its direction. Preserve unconfirmed GT reference and released map/trajectory export semantics.

## SOURCE EXCERPTS

Nearest selection, using integer nanoseconds:

```python
def nearest(scans, target):
    return min(enumerate(scans,1),key=lambda p:(abs(int(p[1]['scan_start_ns'])-target),int(p[1]['scan_start_ns'])))
```

Existing GT parser uses Decimal(seconds)*1e9, requires integral ns and monotonic records,
reads timestamp tx ty tz qx qy qz qw and finite near-unit quaternion. Existing interpolation:

```python
def at(rows,ns):
    stamps=[row[0] for row in rows]
    if not stamps[0]<=ns<=stamps[-1]:raise ValueError('NO_GT_EXTRAPOLATION')
    upper=bisect.bisect_left(stamps,ns)
    if stamps[upper]==ns:lower=upper;weight=0.
    else:lower=upper-1;weight=(ns-stamps[lower])/(stamps[upper]-stamps[lower])
    a,b=rows[lower][1],rows[upper][1]
    rotation=Rotation.from_quat(a[3:]) if lower==upper else Slerp([0.,1.],Rotation.from_quat([a[3:],b[3:]]))([weight])[0]
    T=np.eye(4);T[:3,:3]=rotation.as_matrix();T[:3,3]=(1-weight)*a[:3]+weight*b[:3]
    return T,dict(lower_ns=str(stamps[lower]),upper_ns=str(stamps[upper]),weight=weight)
```

Selected decoded messages are read at exact scan_end record ns; header and frame_id checked.
All ordered XYZ/time match the frozen raw cache through its existing canonical conversion:
cache x/y/z float32 + offset_ns uint32 -> PointCloud x/y/z/time float32(seconds).
An initial direct-layout comparison failed before any NN; it was corrected to use that converter.
Two focused tests pass. Initial test fixture omitted transaction_id, then was corrected.

For each selected raw XYZ file, the existing P34 probe performs the same range .5-80m,
source voxel .25m and cap1400, and same full original map finite XYZ plus twice .15m voxel.
No crop or pose fit. KdTree distances from transformed source to fixed target:
median=sorted[N/2], P90=sorted[floor(.9*(N-1))], coverage counts distance<=threshold.
Both hypotheses share exactly the same 1400 points and target225826 points.

## OBSERVED INPUTS AND FIXED RESULTS

Original map SHA256:
4b5231d58ebd4aeb4fc293559051dc30a18b6f07376237938d064962bfe8f40a
GT SHA256:
3cabcc78ecea4d991aa6e3eddb811cefc4fdacf5f3387b98950fa09ad338dd03

Raw PCD bounds meters:
x[-69.23334503,68.44321442], y[-77.57488251,248.62747192], z[-55.03778076,85.34908295].
PCD VIEWPOINT (tx ty tz qw qx qy qz):
[2.08626,-9.77624,.0196389,-.0844444,-.00417902,-.00205154,.996417].
The production PCD loader reads XYZ and does not apply this metadata again.

Existing runtime proper T_I_L:
R=[[.9999918597233475,-.0005161381080032746,.004001760674214531],
   [.0005196241852023933,.9999994864189835,-.0008701450879252283],
   [-.004001309503946919,.0008722174163330545,.9999916143443562]]
t=[.08,.029,.03].
It is the existing polar projection of the original calibration rotation, not a new calibration.
Original rotation orthogonality error=.0022139659, det=.9984426773, so raw numeric R is not SO(3).
No original file or runtime parameter was changed.

| Target sensor seconds | Raw index | Actual header ns | Relative header s | GT bracket ns/weight |
|---|---|---|---|---|
|67|665|1517157286055073023|66.966953993|1517157285954282999,1517157286155932903/.4998267889083647|
|100|993|1517157319135140896|100.047021866|1517157319034281015,1517157319236001015/.499999410073369|
|150|1488|1517157369057807922|149.969688892|1517157369057807922,1517157369057807922/0|

| Target | A median/P90 m | B median/P90 m | A/B <=.5m, <=2m, <=8m |
|---|---|---|---|
|67|107.896248/117.383118|107.977180/117.472748|all0|
|100|126.249275/133.511734|126.322372/133.595062|all0|
|150|170.768433/179.021454|170.694382/178.945862|all0|

Raw vs decoded LiDAR frame_id is cmu_rc2_velodyne. The Velodyne ROS driver converts packet
polar/range values to ROS XYZ. The dataset decoder Collector copies these XYZ and preserves header;
it applies no LiDAR-IMU extrinsic, global map/world transform, or deskew.

## CLOCKS AND YAML

First original LiDAR header=1517157219088119030ns; first IMU header=1517157219159216000ns;
first GT=1517157219794140100ns.
Original bag first RECORD=1690254112821741400ns (a later repack clock).
First LiDAR after original bag record-start+67:
record1690254179920640694ns, header1517157286155932903ns = sensor-origin+67.067813873s.
Decoded bag first RECORD=1517157219043494976ns. It is repacked to sensor times, LiDAR at scan_end.
First LiDAR after decoded bag record-start+67:
record1517157286055052353ns, header1517157285954282999ns = sensor-origin+66.866163969s.

Official active YAML has '# s 67' then world_darpa R/t, with no precise header timestamp or direction:
R=[[.135990,-.990409,-.024406],[.990705,.136027,.000140],[.003181,-.024198,.999702]]
t=[1.968147,-6.879292,-.896125].
YAML differs from A/B at the frozen near+67 scan by:
A 166.409287m/62.843860deg, B166.488404m/62.818831deg.
The precise sensor-origin+67 GT interpolation has p=[156.385633,54.266467,-12.829614].
These deltas are numeric same-world comparisons only, not a proof of YAML semantics.

## PRIMARY EVIDENCE BOUNDARIES

SubT-MRS paper arxiv2307.07607v3 section3.2 says GT trajectories use GT-map/current-scan
correspondences plus visual/LiDAR/IMU constraints, and validate by projecting scans to a world frame.
It does not specify the exact released SuperLoc PCD/TUM post-export frame transform.
Local GT has 1385 identical p/q numeric records to official Long_Corridor CSV, whose columns say
p_w_b/q_w_b. Official old challenge requires IMU poses for submitted estimates, but GT-export
reference is not directly proven by that requirement.

Inspected author SuperOdom commit f10e65cd50007767b22e4c401689665e20d827d6:
laserMapping.cpp223-244 reads start_pose.txt or init_x/y/z/rpy;305-315 constructs T_w_lidar;
map XYZ loaded directly. No reader for the exact world_darpa keys found in inspected source.
This is not a claim that no unpublished or other historical code could read them.

Our current initialization does not read GT: historical sensor-only first5s initializer uses
R_seed=R_yaml*A_i,t_seed=t_yaml; normalizer applies inverse(T_M_L0) to XYZ. Online O map begins
at L0 and uses T_O_L0=I; internal IMU state T_O_I0=E^-1. It does not seek +67s or parse the comment.
No new initialization, ICP, NDT or localization run was executed in this task.

## REQUESTED REVIEW OUTPUT

Identify substantive math/clock/interpretation flaws, especially any inference not warranted by
these observations. Distinguish a missing official binding from an implementation defect. State
what the fixed 3-frame evidence can and cannot establish, without recommending extra searches.
