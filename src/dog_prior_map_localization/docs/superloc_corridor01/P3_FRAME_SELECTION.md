# PAPER-P3 selected frames and pose convention

The evaluation origin is sensor time `1517157224.188979`.  Nearest NDT scans
within 0.06 s were selected at N0/N1/N2 = 2/5/8 s, T0 = 10 s, T1/T2/T3 =
11.2375/13.1537/16.4819 s, and F0/F1/F2/F3/F4 = 20/25.1553/30/40/60 s.
All twelve matched frames are available; the actual timestamps and offsets are
in `p3_selected_frames.csv`.

The PREFIX10 Kabsch alignment was recomputed from the first ten seconds where
tracking is trusted.  For every scan timestamp, GT translation is linearly
interpolated and orientation is quaternion SLERP without extrapolation.  The
offline reference is explicitly named `OFFLINE_REFERENCE_POSE`:

```text
T_G_I_est_aligned = T_A * T_L_I_est
T_L_I_ref         = inverse(T_A) * T_G_I_gt
T_L_lidar_ref     = T_L_I_ref * inverse(T_l_i)
```

The last inverse follows the frozen evaluator convention in which `T_l_i`
maps the LiDAR pose representation to the IMU pose representation.  The
composition was applied consistently to every frame and is not fed back to
the online node.

`p3_local_basis.csv` stores the horizontal corridor tangent `e_parallel`, the
perpendicular `e_perp = e_z x e_parallel`, and world vertical `e_z`.  The
2-metre persistent crossing is T3: reference-relative translation is 2.02594
m and rotation error is 5.77665 deg for the raw NDT pose (2.02594 m/5.77665 deg
for the accepted pose because no step limit fired at that selected scan).

At T3, the initial prediction is already 2.04706 m from the offline reference
(5.94455 deg), while the raw NDT result is 2.02594 m away (5.77665 deg).  The
available evidence therefore points to **prediction drift already present at
the onset**, with registration retaining a nearby wrong basin; the distinction
is not claimed bitwise because the independent endpoint replay differs by up
to 0.05035 m as documented in the objective audit.
