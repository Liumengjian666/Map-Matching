# Resource comparison with measurement scopes preserved

## Integrated ablation

|Method|Translation RMSE/P95/Max m|Rotation RMSE/P95/Max deg|Nominal/extra Full|Map accepted/rejected|High-error / longest no-map|First sustained low-error raw|Callback Mean/P95/Max ms|NDT / pair peak KiB|
|---|---|---|---|---|---|---|---|---|
|A_ORIGINAL_R23|7.986422/16.421686/18.755587|13.052649/17.166246/18.375466|0/0|0/60|60/60|NONE|30.352/39.254/602.224|259060/282196|
|B_G_ONLY_R25|7.986422/16.421686/18.755587|13.052649/17.166246/18.375466|60/0|0/60|60/60|NONE|110.496/179.406/977.302|369136/389380|
|C_C_PLUS_G_R24|0.209943/0.300124/0.444282|2.984149/3.723287/11.245245|60/8|57/3|2/2|2603|55.304/111.302/1006.031|369096/392536|

B's process wall is13.308355s, C's historical wall9.655876s, A's8.051409s. These are single executions with different causal branches and initialization/cache scheduling costs, not repeated paired timing trials. C uses8 extra calls but subsequently tracks a different, easier branch; do not present the wall-clock difference as a guaranteed speedup. Callback includes in-node preprocessing, map and decision work; input-to-final-ack latency including external Deskew is in RUNTIME_MEMORY.csv. First-frame map/Gaussian construction is included in maximum; no10Hz claim.

RSS is supervisor50ms sampled per-process and simultaneous two-node sum, may miss short peaks, and excludes publisher/Deskew/RViz. A does not enter the optimizer; its lower RSS is not evidence of equal completed work.

## Historical local-map result (P45), separate experiment

The matched300-scan comparison recorded global target3142751 points versus active552716 points (21 tiles). NDT process HWM was775308KiB global versus138200KiB local:82.2% observed reduction. Translation RMSE .067985m to .061612m, but maximum increased .119675m to .216396m and rotation RMSE1.81802deg to1.87495deg. Steady align mean increased7.086ms to8.079ms (about14%). End-to-end input-to-final-ack wall33.82s exceeded29.90s sensor span. Live tile transition was not exercised; the remote replacement probe used a synthetic pose with a real scan, zero NDT. These recorded profiles are not identical-binary kernel microbenchmarks.

![P45 process memory](figures/local_map_memory.png)

R24/R25 W1/C3 active target has2297615 points and roughly360MiB NDT sampled RSS; this is a different spatial footprint. The135MiB P45 result is not a universal whole-system memory limit. No new dog runtime, throughput or memory result exists inR25.
