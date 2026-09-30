# NO-VISION mode audit

V3 CLI now accepts `argv[11] == NONE` and supplies an empty `std::vector<VisualMeasurement>`. The V3 visual-provenance argument also accepts `NONE`; V1/V2 continue to use the strict historical CSV parser and their behavior was not changed.

The new `p6_a3b_no_vision` fixture exercises the same window-owned producer using an empty visual vector. Across four producer policies it verifies six events per policy (three LiDAR start/end pairs), zero visual commits, zero visual factor count in every diagnostic event, and three LiDAR commits per policy. Release CTest passed 24/24, including the original 23 tests; the Debug-targeted test passed 1/1.

Real P0-100 and the recorded portion of P3-100 both used `visual=NONE`, `visual_provenance=NONE`. Their available event rows have zero `VISUAL_REFERENCE`/`VISUAL_CURRENT` events and zero visual factors. Since producer event construction only creates visual events from entries in the vector, empty input means no camera timestamp state can be created.

The A3B code commit is `bbadfb72a0c302b134becc68974b72d9d4d80a6d`. The only runtime change to the runner outside that CLI gate is a diagnostic sidecar recording raw and deskew point counts, stamp bounds, anchor/predicted-end poses, and point displacement statistics; it does not affect deskew, NDT, or factor math.
