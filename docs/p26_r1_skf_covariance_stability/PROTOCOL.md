# P26 bounded covariance investigation

Freeze P25 map, V2 bag, enhanced visual CSV, initialization, all IMU/NDT/OOSM/DCReg/Multi-NDT parameters and velocity_blend=0.6. Use CONTROL, Visual Initial Guess off, SKF on, 0.25 replay, 150ms fixed lag. No GT import or evaluation.

Prefix duration is 19.0 seconds from bag start, covering the first scan +18.355424 s rejected visual transaction and its release. Numerical acceptance additionally reads the covariance audit rather than treating the replay runner completion flag as PSD proof. All emitted SKF measurements must retain actual updates; no pass by rejecting all measurements.

Record at IMU covariance propagation, NDT damping, SKF current/reference endpoints and update, OOSM restore, causal partial propagation, and replay completion. Aggregate all checks; retain a bounded first-fault neighborhood and one 15×15 before/after pair. Save one further pair at the pre-existing P25 failure observation, independent of GT.

Differentiate raw double eigensolver sign, floating-point Cholesky certification and 80-digit inertia of exact stored entries. Do not clip eigenvalues, add covariance jitter, alter noise or loosen quality gates. Do not label genuinely indefinite matrices as rounding noise. A factor result rejects genuine nonpositive pivots.

Final numerical contract FAILED: two IMU writes are uncertified, and the first after-matrix is independently indefinite. All 92 SKF measurements were applied, showing why zero measurement rejections is insufficient for covariance acceptance. No further replay was started.

The first diagnostic used an absolute symmetry threshold which fired before PSD inspection. It was preserved; a second PSD-specific diagnostic identified the eigenvalue false sign. Two subsequent bounded verification prefixes were run; no 40s replay. Source/API changes remain behind the default-off experimental stability flag. Existing P25 archive and untracked caches are retained.
