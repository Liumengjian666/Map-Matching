# P43 frame selection freeze

The two additional validation scans were selected from sensor timestamps before
loading point coordinates for any P43 geometry calculation. The deterministic
rule is: start from the first PCD header timestamp in the released
`01_short_experiment/raw_format/ouster_scan` directory, then select the scan
whose filename timestamp is nearest to `t0 + 100 s` and `t0 + 150 s`.
The first P41 scan is retained as the existing reference frame.

| Role | PCD | Scan timestamp (ns) | Relative to t0 (s) | GT timestamp match | PCD SHA256 |
|---|---|---:|---:|---|---|
| Existing P41 reference | `cloud_1583836591_182590976.pcd` | `1583836591182590976` | `0.000000000` | exact | `7c2fc3a20ef266c2ac3196173c534a1acdb4436224c264bc8f2dc1b1909803a2` |
| P43 additional 1 | `cloud_1583836691_180687360.pcd` | `1583836691180687360` | `99.998096384` | exact | `07872fb6552630b50a56f6346a173f8e9b140ec32c3cb231ccbee1f41aa89c19` |
| P43 additional 2 | `cloud_1583836741_180127744.pcd` | `1583836741180127744` | `149.997536768` | exact | `4f2ec13a27fb787a360a0f971ace0958ff44299330afe8df67c5e6180106c336` |

There are 15,301 canonical timestamp-named PCD scans and the official GT CSV
contains 15,301 rows. One extra file named
`cloud_1583837971_358315008(1).pcd` is not part of the canonical timestamp
filename contract and is excluded; no source files were changed. Each selected
scan timestamp exactly matches one GT timestamp; no interpolation or
extrapolation is needed for these three frames. GT SHA256:
`b2fdfab253ce19ae58e1566c4ef0b66fa04f6feb3798d11d41ca5e809cc4363e`.

P43 geometry had not been computed when this selection was frozen. No candidate
pose, nearest-neighbor metric, or map overlap was used to select these scans.
