# LiDAR factor test

`window_lidar_factor.cpp` consumes a map pose measurement and a reliable basis
from U_obs/Schur.  It checks basis rank, covariance SPD, and finite residuals,
then produces the projected pose residual and central-difference Jacobian.
PCL NDT is not duplicated in this module.  A rank-deficient measurement must
supply an upstream U_obs/R3 basis callback.  The callback is evaluated at
each outer linearization state and frozen while the local Jacobian is formed;
the synthetic test observed 734 such relinearizations.  Invalid/rank-zero or
non-SPD inputs are rejected at insertion and the skipped reason is reported
through `WindowSummary`.
