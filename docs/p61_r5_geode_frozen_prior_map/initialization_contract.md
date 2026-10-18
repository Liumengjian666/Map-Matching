# INITIALIZATION_NOT_ESTABLISHED

Convention: `p_A=T_A_B*p_B`; M is the non-GT FAST-LIO `camera_init` gauge, I external Xsens, L Livox Avia. The mapping gauge's initial identity is **not** a test initialization.

The actual final mapping state is at author scan-end time `1706671533.4995439` seconds (exact raw point tail: `1706671533499543833` ns; author's double computation is retained separately):

```text
T_M_L(mapping_end) =
[ 0.9497247474 -0.3098819885  0.0446772584 224.9485432869 ]
[ 0.3112491743  0.9499253953 -0.0276711914  47.7472152260 ]
[-0.0338652585  0.0401857751  0.9986181692   0.8865802555 ]
[ 0             0             0             1           ]
```

This matrix is a measured non-GT **mapping-end** state, not `T_M_L(test_start)`. The frozen test start is `1706671587829840123` ns, about **54.3302963 s later**. The middle10% was not delivered to the mapper, and no state propagation over it was executed. Reusing the matrix above as the test pose would assume away real motion.

The map and all recorded states share M. The CSV stores actual IMU/LiDAR transforms, velocity, gyro bias, acceleration bias and gravity; original author state logs are retained internally. It is not a serialized full integrator/covariance restart checkpoint. No claim is made that an exact production resume can be achieved by restoring only the CSV pose.

The 50 time-uniform test samples have no certified non-GT M-frame poses in this task. All 50 are marked `POINTCLOUD_COVERAGE_NOT_CERTIFIED`. This does not mean measured overlap is zero, nor does it prove coverage insufficient. The R4 position-only revisit evidence remains a feasibility clue, not a point-cloud registration certificate.

Only next proposal: a separately approved causal LiDAR–Xsens state-bridge experiment through the fixed middle10%, preserving this prior map unchanged and excluding all bridge/test points from it, to establish and freeze `T_M_L(test_start)` and then evaluate the existing 50 coverage samples. No such experiment was run here. No GT, arbitrary identity or whole-trajectory fit is an acceptable substitute.

Full NDT localization and fused-localization Smoke remain unapproved/not run.
