# SOLiD provenance and compatibility patch

- Repository: https://github.com/sparolab/SOLiD
- Pinned main commit: `9a4f2895e5211f69fbbd6d4732577e1a67e9b882`
- License: upstream BSD-3-Clause, preserved in `LICENSE`.
- `src/solid.cpp` is byte-identical to upstream `cpp/src/solid.cpp`:
  SHA256 `765d21b53f4c12e2ebaabf935701c423db83076b9c8476944d836b061c711225`.
- `include/solid_module.h` imports upstream `cpp/include/solid_module.h`.
  Only sensor constants change: FOV [-24.8,+2] to [-7,+52] degrees,
  max range 80 to 40 m. A comment identifies the compatibility patch.
  Bin counts, .4 m voxel and descriptor equations remain unchanged.
- README.upstream.md preserves the original usage guidance, including the
  author's instruction to change header parameters for another LiDAR.
- The independent adapter rejects invalid inputs and zero-normalizer cases
  before calling the unchanged descriptor. It never replaces the descriptor
  equations with a new implementation. Original/adapted behavior is reported
  in `docs/p11_mature_solid_prior_map_integration_r1/`.
- Compatibility work is not an original place-recognition algorithm. Scores
  are cosine similarities, not probabilities. Database reference poses are
  mapping-state poses, not optimized query poses or verified NDT basins.
