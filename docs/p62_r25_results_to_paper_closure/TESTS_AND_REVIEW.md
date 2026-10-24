# R25 minimal ablation verification

Release NDT rebuild passed (logs/BUILD_G_ONLY.log). Unchanged EKF and sequence-publisher executables are copied from preserved R24 binaries and SHA-verified; they are not rebuilt or changed. This preserves exact fusion/replay implementation identity.

Only candidate-search enable predicate changes behind default-false research/r25_disable_search. G-only requires explicit R24 G=true, F=false, frozen C policy. Original quality/selection remains r20::accepted/r20::choose and no other algorithm parameter is changed. Runtime must show extra_aligns=0 in every row; at most60 Full calls. Python syntax and launch parameter checks precede the single real causal run. No extra synthetic algorithm or historical NDT replay.

Final source/numerical self-review: B native60/converged60/extra0/map0, actual EKF+TP sequences equal frozen A; C historical68 Full/map57, first sustained recovery raw2603. Native seal precedes reference reads. Plot shows full60 samples, coincidentA/B curves and initial C transient; PNG manually viewed. Manuscript has all required English sections and checks each key ablation number against source tables; no external AI opinion was invoked. Current C source acceptance and separate publication limits are described accurately, not conflated. DOG coordinate block and nonexecuted visualization-only launch are labeled.
