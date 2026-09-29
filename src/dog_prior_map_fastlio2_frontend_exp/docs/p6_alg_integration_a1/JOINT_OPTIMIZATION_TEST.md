# Joint optimization test

Release result from `p6_i6f_joint_window_test`:

```
I6F_JOINT_WINDOW_TEST_PASS initial_cost=1134.1 final_cost=8.10867e-20 initial_latest_error=0.174642 final_latest_error=2.53533e-12 hessian_rank=135 marginalized_nodes=6 retained_prior_cross_norm=1.03929e+10 duplicate_count=1 basis_relinearizations=734 visual_information_rank=3 batch_window_position_delta=2.00975e-12 incremental_truth_error=5.25622e-13
```

The test covers a single joint objective with IMU, LiDAR, and cross-state
visual factors, a duplicated observation ID, numerical rank diagnostics,
optimization cost decrease, and bounded marginalization.  The visual factor
contributed a rank-3 joint information block with non-zero reference/current
cross information.  The same observations were also processed incrementally;
the latest-state delta from the full-batch solution was `2.00975e-12 m`, and
the incremental truth error was `5.25622e-13 m`.  It is deliberately small
and deterministic.
