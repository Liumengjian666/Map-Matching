# Visual factor test

The factor is a metric relative-translation factor, not an absolute visual
map observation.  For reference `i` and current `j`:

`r = p_j - p_i - R_i z_ij`.

The synthetic test compares the analytic `-I/I/R_i skew(z)` blocks against
central finite differences for both translation and right-SO3 rotation.  The
factor rejects an incorrect source semantic and never consumes visual
rotation.  It does not claim metric scale from a monocular two-frame geometry
without an upstream metric source.
