# Reproduce without touching production nodes or external storage

This records the actual executed paths. Do not rerun against an existing output
directory or overwrite the frozen successful results. R4 is complete; future runs
require new research authorization. No ROS master/IMU/GT/EKF is used here.

## Input preparation (executed once)

```bash
cd /home/jian/livox_ws/p60_r4_ground_challenge_degeneracy_ws
/usr/bin/python3 docs/p60_r4_ground_challenge_ndt_degeneracy/prepare_inputs.py /home/jian/ground-challenge/corridor2/corridor2.bag /home/jian/livox_ws/p60_ground_challenge_runs/corridor2_input_contract.json /home/jian/livox_ws/p60_r4_ground_challenge_degeneracy_runs/inputs
```

Existing input directory causes refusal. Bag and R2 contract SHA are checked
before extraction; each selected raw payload and stamp matches the R2 contract.

## Offline build and tests

```bash
cmake -S docs/p60_r4_ground_challenge_ndt_degeneracy/tools -B /home/jian/livox_ws/p60_r4_ground_challenge_degeneracy_runs/build -DCMAKE_BUILD_TYPE=Release
cmake --build /home/jian/livox_ws/p60_r4_ground_challenge_degeneracy_runs/build -j2
cd /home/jian/livox_ws/p60_r4_ground_challenge_degeneracy_runs/build
ctest --output-on-failure
cd /home/jian/livox_ws/p60_r4_ground_challenge_degeneracy_ws/docs/p60_r4_ground_challenge_ndt_degeneracy
/usr/bin/python3 -W error -m unittest test_sampling test_statistics -v
```

Standalone opt-in docs CMake only; production Catkin build/launch paths are unchanged.
GCC9.4.0, CMake3.16, C++14, PCL1.10.0, Release `-O3 -DNDEBUG`, no fast-math.
Only the existing native derivative/chart helper is retained from the source file
named `coupled_ndt_shadow.cpp`. Linker section GC removes candidate-search entry
points; `nm -C p60_r4_screen` shows coupledNominalJet, not runCoupledNdtShadow or
runWeakCoupledRefinement. No Coupled algorithm is enabled.

## Actual one-pass registration command

```bash
LD_PRELOAD=/lib/x86_64-linux-gnu/libusb-1.0.so.0 /usr/bin/time -v -o /home/jian/livox_ws/p60_r4_ground_challenge_degeneracy_runs/screen_resource.txt /home/jian/livox_ws/p60_r4_ground_challenge_degeneracy_runs/build/p60_r4_screen /home/jian/livox_ws/p60_r4_ground_challenge_degeneracy_runs/inputs/sample_manifest.csv /home/jian/livox_ws/p60_ground_challenge_runs/raw_probe_20_corrected/ndt_diagnostics.csv /home/jian/livox_ws/p60_ground_challenge_runs/raw_probe_20_corrected/hessian.csv /home/jian/livox_ws/p60_r4_ground_challenge_degeneracy_runs/formal
```

Actual stdout/stderr saved in `screen.log`; loader-only failed launch in
`loader_failure.log` and `loader_failure_resource.txt`. Those are not a second
sensor experiment. `formal` is exclusive-create. No frame/group is retried.

The first FULL pair consumes its sole align, then checks source/pose/fitness/
iterations/Hessian against frozen R2 before moving beyond the first pair.

## Frozen-output analysis

```bash
/usr/bin/python3 docs/p60_r4_ground_challenge_ndt_degeneracy/analyze_results.py
```

All run-freeze hashes are checked first. Analysis outputs are exclusive-create;
the already archived results are not overwritten. Runtime values are parsed from
the frozen `/usr/bin/time` log. The later parser regression was validated against
the existing runtime JSON without rerunning registration or changing its values.
