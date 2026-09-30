# One authorized real replay

Exactly one new real-data replay was performed. Configuration was held to
the A3B-R2 setting: Corridor01, P3 `ADAPTIVE_SELECTED_NIS`, frame limit 100,
visual `NONE`, handoff `1517157224188979000`, profile `corridor01`, same raw
timed input, IMU, normalized map, and official calibration parameters. Both
`P6_A3B_R1_DIAGNOSTICS=1` and
`P6_A3C_R1_MARGINALIZATION_DIAGNOSTICS=1` were enabled. No GT was read.

The current input hashes matched the frozen A3B-R2 record and the raw timed
input manifest runtime gate passed:

| Input | SHA256 |
|---|---|
| `raw_timed_points.bin` | `4ba09d8ae7004056dcc4e9d63d2ab09915748c7e68bc29d0dc8bd44a24fc95ff` |
| `raw_timed_catalog.csv` | `fdaf9607bc1933269f5f999ee044d37aa1eefabedc054f522da5078ce708123f` |
| `filter_scans.csv` | `41d0b2040a5de8a8bd428c382a7b6dc18fabcaa8d3e7d2cc331018e0585edf1d` |
| `RAW_TIMED_INPUT_MANIFEST.txt` | `fb20125c63aa4c7110851b8bb08ec33e8938d94be18e28e9737d207001377575` |
| normalized map | `103a01b2c89ca2adbd2b2e22256acba6e91f40b1028857c2103f08c6c3eb8f8f` |
| official calibration parameters | `7e42752ff8b84eae2b2da8d7d9fe179db0bb8f364a923e12236d2e91336e357d` |
| IMU CSV | `7dc881d4ebfeacea9354be569a5952e5e466ccdd366637e52a7797a6f37457aa` |

The source bag SHA recorded by the raw manifest and IMU manifest is
`c9e127402463eebee25c54bc967eb377e026bbbdfc979eae853a5af134180811`.

The run reproduced tx90 at `1517157228164951397`, reason
`marginalized_prior_not_finite_psd`, after optimizer status
`ACCEPTED_UPDATE`, 8 iterations, with the same initial/final optimizer costs
as A3B-R2. This is a logical reproduction; no third retry was made.

The command was the A3B-R2 P3-100 command with fresh `/tmp` output paths and
the two diagnostics environment variables above. Resource result: 26.35 s
elapsed, 22.36 s user, 1.66 s system, 106648 KiB maximum RSS. This includes
forensic matrix diagnostics and is not a benchmark.

The C++ production expression replay from the saved capsule is a matrix-only
offline check, not another localization replay. No P3-200, full Corridor01,
Floor01, GT evaluation, or parameter sweep was run.
