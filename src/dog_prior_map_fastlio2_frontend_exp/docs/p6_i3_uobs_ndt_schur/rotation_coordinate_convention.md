# Rotation Coordinate Convention — P6-I3 R1

## PCL parameter convention

PCL 1.10 NDT evaluates its six-vector in translation-first order `[tx, ty, tz, rx, ry, rz]`; its XYZ Euler rotation is `R = Rx(rx) Ry(ry) Rz(rz)` and angles are radians. The Hessian is therefore initially expressed in Euler-parameter increments, not fixed physical roll/pitch/yaw axes.

## Physical rotation perturbation

P6-I3 reports map-frame / spatial infinitesimal rotation coordinates `d_phi`, defined by the left perturbation `R(e + d_e) R(e)^T ≈ Exp([d_phi]x)`. For the stated PCL XYZ convention:

```text
d_phi = J_spatial d_e
J_spatial = [ e_x, Rx(rx)e_y, Rx(rx)Ry(ry)e_z ]
d_e = [drx, dry, drz]^T
d_phi = [d_phi_x, d_phi_y, d_phi_z]^T
```

The Hessian input is first reordered to Euler-canonical `[rx, ry, rz, tx, ty, tz]`. Define `A = blockdiag(J_spatial^-1, I3)`; the physical-tangent Hessian is `H_phys = A^T H_euler A`. The physical order is `[d_phi_x, d_phi_y, d_phi_z, dt_x, dt_y, dt_z]` and serialized rotation axis names are `rotation_x`, `rotation_y`, `rotation_z`. These are map-frame infinitesimal axes; “roll-like / pitch-like / yaw-like” is only an approximate interpretation.

## Finite-difference validation and singularity handling

For each fixed orientation, each column is checked using `Log(R(e + eps e_i)R(e)^T)/eps`, with `eps = 1e-7 rad`. The output `coordinate_transform_validation.csv` records analytic and finite-difference columns, errors, and Jacobian condition. The maximum column error and maximum condition are reported in `summary.md`.

The implementation estimates the Jacobian condition from its singular values, rejects condition above `1/sqrt(machine epsilon)`, requires full rank under column-pivoted QR, and accepts the solve only when the inverse residual is at most `1e-10`. Invalid orientations are explicitly marked and counted; there is no silent matrix inverse or fallback to Euler axes.

## Scope

This convention changes only the offline P6-I3 Hessian analysis. It does not change the ROS runtime, NDT optimizer, thresholds, eigenvalue policy, or any visual/multi-start behavior.
