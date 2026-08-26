# NDT Source Point Count Tuning - 2026-08-26

Goal: test whether increasing current-frame NDT source points removes the remaining corner mismatches.

Baseline retained:
- Config default: `ndt_max_source_points: 1400`
- Run: `/home/jian/rosbag/loop3/ndt_finer_map_source_loop3_20260826_044853`
- Full-bag vs FASTLIO2Location: mean 0.417 m, RMSE 0.618 m, median 0.245 m, p90 1.243 m, p95 1.535 m, max 2.694 m.

Experiments:
- `ndt_source1800_loop3_20260826_204207`: full-bag valid, but worse overall: mean 0.438 m, RMSE 0.687 m, p95 1.549 m, max 3.406 m.
- `ndt_source1500_retry_loop3_20260826_211615`: full-bag valid, severe wrong attraction: mean 8.595 m, RMSE 16.670 m, p95 43.844 m, max 55.329 m.
- `ndt_source1600_loop3_20260826_205335`: invalid for final judgment because rosbag recording stopped at 399 s due to disk free space dropping below 1 GB.
- First `ndt_source1500_loop3_20260826_210431`: invalid because rosbag recording was disabled immediately by low disk space.

Corner-window comparison vs FASTLIO2Location:

| Run | 345-351 s max | 398-403 s max | 565-576 s max |
| --- | ---: | ---: | ---: |
| baseline 1400 | 2.429 m | 1.782 m | 2.695 m |
| source 1800 | 0.883 m | 3.226 m | 3.406 m |
| source 1500 | 0.757 m | 8.328 m | 48.673 m |

Conclusion:
- Increasing source points does help the first remaining corner around 345-351 s.
- However, it makes later repeated/corner regions worse, especially 398-403 s and 565-576 s.
- Do not change the default `ndt_max_source_points: 1400` based on this test.
- The right next step is scene-dependent candidate verification or adaptive source-point selection, not a global point-count increase.

Cleanup:
- Large rejected/invalid experiment bag folders were removed after extracting metrics to preserve disk space.
