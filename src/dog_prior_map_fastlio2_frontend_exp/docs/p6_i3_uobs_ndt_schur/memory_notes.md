# Memory Notes

Persistent U_obs analysis state is lightweight: one canonical and one normalized 6x6 double matrix (576 bytes total), six 3x3 double matrices for the Hessian blocks/Schur terms (432 bytes), plus 6x6 / 3x3 eigensolver outputs (bounded to a few KiB). Temporary factorization/eigensolver workspaces are also fixed-size.

No standalone heap profiler or before/after RSS attribution was run. The process also owns PCL's target voxel structure and point-cloud buffers, so whole-process RSS would not isolate analyzer-only memory. The matrix-level analyzer footprint is negligible by comparison and bounded independently of the 4127-frame sequence; the tool processes Floor01 frames serially and does not retain frame clouds or Hessians across frames.
