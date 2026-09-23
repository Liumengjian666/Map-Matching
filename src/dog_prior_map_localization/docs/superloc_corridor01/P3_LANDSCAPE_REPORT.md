# PAPER-P3 NDT likelihood landscape report

## Objective values at the key times

Raw PCL objective (higher is better; NN fitness is a separate lower-is-better
quantity):

| frame | relative time | objective(reference) | objective(prediction) | objective(raw NDT) |
|---|---:|---:|---:|---:|
| N1 | 5.0000 | 2173.61 | 2333.19 | 2331.23 |
| T3 | 16.4819 | 844.944 | 3268.87 | 3290.38 |
| F1 | 25.1553 | 100.098 | 3274.52 | 3285.40 |

At the 2-metre onset and at the 5-metre crossing, the wrong baseline pose is
more competitive than the offline reference by a large margin.  This is the
central reason that scalar nearest-neighbour fitness cannot certify global
correctness in this corridor.

## 1-D and 2-D structure

The probe sampled the corridor axis from -30 to +30 m at 0.25 m and refined the
strongest local extrema in ±1.5 m at 0.05 m.  The key six frames have s-yaw
and s-n grids at the requested ranges.  Heatmaps are in `results/p3_landscape/
plots/` and mark `T_ref`, `T_pred`, and `T_ndt`.

N0 and N1 have a dominant local basin with a large score margin to the best
wrong sampled basin.  A second competitive basin is already visible at N2;
it persists through T0/T1 and becomes strongly separated by T3/F1.  F0 and F4
also exhibit weak local curvature.  The candidate table is exploratory only;
it is not a final mode detector.

## Multi-start and curvature

Original PCL NDT was run from the prescribed s=-20...20 m, yaw=-10/0/+10 deg
seeds.  Endpoint clustering remains highly fragmented under 0.25 m/1 deg,
0.5 m/2 deg, and 1 m/5 deg sensitivities (see
`p3_multistart_cluster_sensitivity.csv`), confirming that the optimizer's
basins are not a single globally attracting solution.

The exact PCL Hessian was evaluated separately for translation and rotation.
The weakest translation eigenvector is often strongly aligned with the
corridor tangent in normal and transition frames (absolute dot products about
0.86--0.98 at N0/N1/T0/T1), but alignment drops at T2/F0/F1 and the reference
translation information becomes indefinite at F1/F2.  This is evidence for a
changing local geometry, not a finalized degeneracy threshold.

The frame-level exploratory labels are in `p3_frame_classification.csv`:
N0/N1 UNIQUE_SHARP; N2/T0/T1/T2/T3/F1/F2/F3 MULTI_MODE; F0/F4 BROAD_MULTI.
They use raw score/separation reporting and are not a proposed online mode
extraction algorithm.
