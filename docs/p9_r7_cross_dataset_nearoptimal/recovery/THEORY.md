# R7-R1 recovery contract (not a new evidence experiment)

Recover the exact original v1 bytes first. If unavailable, at most one complete
reconstruction is authorized, only after raw-bag hash, historical implementation
including transitive source, and persistent writable output are verified.
Container byte equality and message-content equivalence are different claims.
No conversion has been run; equivalence classifications are therefore pending.

Input semantic target remains VLP16, rotation-only scan-start deskew, original
sensor headers/topics/frames/extrinsics. A current full-SE3 output is not v1.
After input equivalence, the P9 scan-end source must be derived using verified
per-point time and IMU state propagation, not by relabeling a scan-start cloud.

Only after these gates may two fixed-parameter 2776-frame nominal replays test
source/T0/U_obs and weak-subspace repeatability. No GT initialization or resets,
no good-prefix selection, and no Floor01 T0/W2/source reuse. Tolerances must be
frozen before replay, not selected from run differences. No replay/tolerance
experiment was started here.

NDT .8/.08/1e-5/80 and P9 chart/DUAL-U/R6 rules remain unchanged. Oracle263,
B12, visual extraction and R6 statistics are forbidden in this recovery task.

This attempt stops at input recovery: persistent storage permission is absent,
and the inspected current adapter is not a certified historical generator.
No attempt is made to infer algorithm efficacy from this operational blocker.
