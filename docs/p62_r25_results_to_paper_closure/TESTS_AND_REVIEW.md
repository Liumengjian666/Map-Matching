# R25 minimal ablation verification

Release NDT rebuild passed (logs/BUILD_G_ONLY.log). Unchanged EKF and sequence-publisher executables are copied from preserved R24 binaries and SHA-verified; they are not rebuilt or changed. This preserves exact fusion/replay implementation identity.

Only candidate-search enable predicate changes behind default-false research/r25_disable_search. G-only requires explicit R24 G=true, F=false, frozen C policy. Original quality/selection remains r20::accepted/r20::choose and no other algorithm parameter is changed. Runtime must show extra_aligns=0 in every row; at most60 Full calls. Python syntax and launch parameter checks precede the single real causal run. No extra synthetic algorithm or historical NDT replay.
