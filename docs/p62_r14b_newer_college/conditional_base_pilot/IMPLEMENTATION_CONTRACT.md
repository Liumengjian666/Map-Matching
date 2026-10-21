# Conditional Base pilot — frozen implementation choices before NDT

User task: P62-R14B-BASE-CONVENTION-CONDITIONAL-PILOT. Baseline c614f5fd1d8837b4c96ec10cc1199e41d5b2418b.
Old blocked report remains valid historically and untouched. Sole registered
interpretation: PAPER_SUPPORTED_BASE_FRAME_ASSUMPTION, T_CSV=T_M_B.
This does not establish the CSV writer's file-level axes. No Optical experiment.

Chain: T_M_L=T_M_B*T_B_C*T_C_I*T_I_L. T_B_C=[[0,0,1],[-1,0,0],[0,-1,0]]
with zero translation; official Kalibr cam0 provides T_C_I; official sensor YAML
provides T_I_L and inverse T_L_I; units meters, quaternions xyzw. Matrix
orthogonality/det/inverse checks precede use. Normalization of the reference's
rounded quaternion is recorded, not a calibration fit.

File-level timestamp equality only; no per-point absolute-time assertion,
camera time shift, IMU integration or new deskew. Uncorrected scans retain
intra-scan motion distortion. Reference is MAP_ANCHORED_BENCHMARK_REFERENCE.

Selection: use retained full inventory start/end, strict integer test
20*(scan_ns-first_ns)>last_ns-first_ns; first consecutive21 scans satisfying
PCD/finite-source and exact reference timestamp identities. If fixed21 inputs
fail, stop instead of moving the window. Only first pose values read for anchor;
the evaluation reference values are deferred until native outputs and decisions
are frozen (prior task's format-only access is disclosed, not undone).

All declared PCD bytes are preserved; zero bytes strictly after declared records
may be ignored by reading. Native PCL must return exactly declared point count,
and its raw record bytes and every XYZ float bit must match independent original
record decoding. No canonical copies unless native read fails and equivalence
can be proven separately; this implementation stops on failure rather than
automatically recoding or selecting a later frame.

Build exactly one official-PLY derived map: spherical3D radius50m around anchor
LiDAR translation under the fixed Base chain, then native PCL VoxelGrid0.20m.
No subsequent recropping. Anchor Source support@0.5m must reach0.50 (existing
admission support scale) as a pre-NDT map availability check, not pose truth.
No test-reference geometry is used for this gate. Native Gaussian target grid
remains0.8m; map voxel leaf0.20m is a different quantity.

Shared predictor: right/body constant velocity
TP(t)=H_last*Exp((t-t_last)/(t_last-t_prev)*Log(H_prev^-1*H_last)).
History contains anchor and admitted CONTROL_FULL observations only; less than2
history entries holds last valid pose. A rejected CONTROL_FULL observation
does not enter history, and disables extrapolation (hold last admitted pose)
until a new full observation is admitted. Actual timestamps, not nominal dt.
Acceptance uses unchanged admissible() and the same heldout/Coverage mask for
engineering safety; FULL includes validation points in optimization, unlike SPLIT.
No PROPOSED output or evaluation reference affects common history.

Per20-frame sample: FULL1 native align, SPLIT1; PROPOSED reuses SPLIT. Original
native parameters unchanged. Existing R10 event detect(), R13 fixed45 grid and
training-only <=256-point coarse ranking, max2 native refinements/event,
20 extra total; V3 pool and D>0.02 unchanged. No forced events or diagnostic arms.
Raw optimizer output flushed before fitness/validation; START/RETURNED calls
recorded. Invalid SE3/nonfinite output records failure and stops, no repeated run.

Source adaptation uses native PCL reader then exact R8 preprocessRaw(): finite
0.5–80m,0.25m voxel,<=1400 deterministic sampling,16 fixed scanner sectors.
Coverage V2 and eight raw blocks retain frozen semantics.

Reference evaluation runs in a separate offline step only after SHA sealing all
native/decision outputs; it never aligns trajectories. PROPOSED rejection has
no accepted-pose error entry, not zero error or disguised accepted prediction.
No new localization/EKF feedback. Static-map, preflight and frame costs separated;
no10Hz claim. No automaticR15 or parameter/coordinate rerun.
