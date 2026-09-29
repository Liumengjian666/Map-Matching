# Fixed-lag marginalization test

The synthetic window inserts nine nodes, optimizes, and applies the 2 s/48
node bounds.  Old nodes are removed by a Schur complement solved with LDLT;
the retained prior includes cross information for all retained state blocks.
The test checks PSD diagnostics, bounded node count, non-zero retained cross
information, and the latest-state prediction feedback seed.  It also extends
the retained prior with zero information when a new node is appended; this
prevents a prior-dimension mismatch during incremental sliding-window use.
The nine-node full-batch and incrementally marginalized solutions differed by
`2.00975e-12 m` at the latest position in the deterministic synthetic case.
