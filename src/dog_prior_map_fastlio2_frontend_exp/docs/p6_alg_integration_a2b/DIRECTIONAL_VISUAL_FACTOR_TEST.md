# Directional visual factor test

The latest causal, non-stale LiDAR risk record supplies `A_exact` and `U_w`.
The adapter computes `W=A_exact U_w`, takes its translational rows, and uses an
SVD-derived orthonormal `Q_w`. The factor then applies exactly

```
e_s  = Q_w^T e
R_s  = Q_w^T R Q_w
Ji_s = Q_w^T Ji
Jj_s = Q_w^T Jj.
```

Modes are `FULL_TRANSLATION`, `LIDAR_WEAK_TRANSLATION`, and `NOT_TRIGGERED`.
Normal LiDAR does not trigger visual fusion. Pure-rotation weakness with zero
lever arm produces `NO_TRANSLATIONAL_COMPLEMENT`. Missing map support permits a
relative 3D factor only and is labelled `RELATIVE_ONLY_NO_GLOBAL_RECOVERY`.

Sensor quality is evaluated separately from prediction consistency. Because no
window posterior covariance is exposed yet, the adapter reports the conceptual
status `WINDOW_PREDICTION_NIS_PENDING` via the sensor-only decision and never
invents an innovation chi-square value.
