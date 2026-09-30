# tx90 failure capsule provenance

The transient C++ binary capsule was parsed and converted to one compressed
NPZ so the large repeated matrices are not duplicated in the repository.

```text
binary source name: trajectory.csv.a3c_r1_failure_capsule.bin
binary source SHA256: 40e2e605bc71f24207ab4d5c53a662694361505fbf099b3fb67b9955aa48a959
binary source byte size: 18,242,115
binary format: P6A3CR1CAPSULE_v1; row-major float64 arrays

committed capsule: TX90_FAILURE_CAPSULE.npz
NPZ SHA256: 2f597628160e75f41bc4c13d9de015727c17b1755535ad806263f272ebcf0280
NPZ byte size: 32,432
metadata JSON SHA256: 489b46defc12b57fdb310109307459a1124a11c6ed02873b043eafba37d4e70b
```

Array shapes/dtypes:

| Arrays | Shape | dtype |
|---|---:|---|
| incoming/charted/consumed information and consumed/factor Hessians | 615×615 | float64 |
| incoming/charted/consumed gradients | 615×1 | float64 |
| production solve correction `correction_h` | 15×600 | float64 |
| production solve correction `correction_b` | 15×1 | float64 |

The NPZ includes incoming prior information/gradient, charted prior
information/gradient, consumed `H_c/g_c`, IMU/LiDAR/visual component
Hessians, and the actual production correction matrices. Visual contribution
is zero in this replay. The original 18.2 MB transient binary is not
committed; its SHA and size are recorded above and in the metadata JSON.
