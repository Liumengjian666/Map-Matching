# Reproducible failure cases

No localization failure case was observed. No scan was processed by the localization system, so there are no frame IDs, sensor timestamps, geometric residuals, Hessian values, NDT scores, or estimator innovations to report.

The only observed failures are acquisition failures:

1. Ground-Challenge Corridor2's official OneDrive endpoint required Microsoft sign-in and returned HTML instead of the dataset.
2. The official SubT-MRS Long_Corridor transfer ended twice before the advertised archive size. The retained partial is not a usable ROS bag and was not extracted.

These acquisition failures must not be labeled as localization failures, degeneracy, or NDT errors.
