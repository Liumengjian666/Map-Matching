# raw62 fixed-input engineering counterfactual

Inputs: R29 EASY/C0/C raw62 native terminal and raw49 last published observation, with R30's frozen identity and source record. No new NDT, prediction, reference fitting or EKF call is used to decide this counterfactual.

Original candidate: basic quality PASS, absolute quality PASS, native finite/converged. Last published observation age0.867962201s. Old limiter changes translation by0.44624886575078776m. The shared C++ limiter reproduces the full historical LEGACY published4x4 matrix exactly (norm difference0).

|Policy|Candidate certificate|Would publish|Would update map history|Would send new EKF measurement|
|---|---|---|---|---|
|LEGACY|Original admitted terminal|Yes, altered pose|Yes|Yes; original R29 actually fused|
|STRICT_FAIL_CLOSED|Same original terminal|No|No|No|

Reason: ALTERED_OBSERVATION_NOT_CERTIFIED. Original matrices, proposed altered matrix and executable output are in evidence/raw62_*.txt and RAW62_COUNTERFACTUAL.json. Historical reference-relative errors0.425469771m/4.251052648deg raw and0.755997115m/2.981527815deg altered are context, not inputs to the decision.

The complete STRICT replay has its own earlier history. Therefore its actual raw62 TP/native output can differ from the frozen LEGACY counterfactual. Both records must be reported separately; do not claim to have run a new online trajectory with forced raw49/raw62 GT or borrowed LEGACY state.

## Exact frozen matrices (T_map_lidar)

Certified native terminal:
```text
-0.0052233459427952766 -0.15679252147674561 0.98761773109436035 -24.932594299316406
-0.99985545873641968 -0.01515984907746315 -0.0076948213391005993 3.6290168762207031
0.016178630292415619 -0.9875151515007019 -0.15669067203998566 -0.32046255469322205
0 0 0 1
```
Previous raw49 published observation:
```text
0.045106936246156693 -0.08855033665895462 0.99504983425140381 -25.849042892456055
-0.99673217535018921 0.062825620174407959 0.050774101167917252 3.4174361228942871
-0.067010700702667236 -0.99408841133117676 -0.085427097976207733 -0.42410847544670105
0 0 0 1
```
LEGACY limiter proposal / historical publication:
```text
0.0079578693811444001 -0.13782658051232133 0.99042440701912904 -25.364789422618337
-0.9999444298441702 0.005751853568296772 0.0088347837634214499 3.5292358623626883
-0.0069144441953192803 -0.99043967503569319 -0.1377731489683689 -0.36934174097286754
0 0 0 1
```

Real STRICT B/C replay both reproduced this raw62 certificate and proposal, then refused the measurement (state_unchanged=true; map_published=false; EKF_update=false). The proposed rotation differs from the certificate by1.879341919deg. Last map remainsraw49. Both subsequently refuse raw96 (translation change2.214582656m, rotation8.409265289deg). RAW62_ONLINE.json preserves full real candidate Score/Overlap/Coverage, original matrices, decision and receipt. No EKF update was simulated or inferred just from a boolean decision.
