# Short real-link resource sanity

These are `/usr/bin/time -v` wall/CPU/RSS measurements for this engineering check only, not a formal benchmark.

| Invocation | Wall | User | System | Max RSS | Exit |
|---|---:|---:|---:|---:|---:|
| P0-100 complete | 20.64 s | 12.41 s | 2.32 s | 72,044 KiB | 0 |
| P3-100 stopped at tx83 | 6.40 s | 5.65 s | 0.73 s | 71,264 KiB | 1 |
| Initial loader-only P0 launch | 0.02 s | 0.01 s | 0.00 s | 18,228 KiB | 127 |

The first attempt inherited `/opt/MVS/lib/64` ahead of system libraries and loaded a libusb lacking the `libusb_set_option` symbol required by system PCL. It failed before `main()` and processed no scans. Retrying with `/lib/x86_64-linux-gnu:/usr/lib/x86_64-linux-gnu` first selected the system libusb and allowed P0 to execute; this was an environment-only correction.

P0 event diagnostics include per-event NDT, linearization, solve, and marginal-covariance times. P3 diagnostics end before the failed terminal is written, so they do not contain tx83 event timings. No CPU affinity or performance tuning was performed.
