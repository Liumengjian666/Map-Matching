# R7 frozen validation plan — NOT EXECUTED

R6 remains NEAROPTIMAL_ADMISSION_PROMISING_EXPLORATORY. R5/R4 conclusions remain
unchanged. Floor01's 96 frames are development data, not R7 validation samples.

On a verified Corridor01 source/T0/U_obs chain, preserve NDT .8/.08/1e-5/80,
predictor-conditioned WEAK2 B12 (seed122 first, original FPS), complete-link
.2 m AND 2 degrees, converged-only terminals, maximum-score representative,
and the P9 spatial map-product chart. Nominal is included.

S_star is the best observed B12/nominal score, never an oracle score.
Delta=(S_star-S_k)/N; tau=.05*max(1,abs(S_star/N)); near iff Delta<=tau.
Strict representative separation is >.2 m OR >2 degrees from nominal.
TYPE A requires nominal in-band and a separated near alternative; TYPE B
requires nominal out-of-band and such an alternative. Keep their sums of
xi xi^T and sqrt(max eigenvalue) separate; these are evidence, not covariance.

Label-blind scan ordering uses SHA256 of
`P9_R7_CORRIDOR01_CROSS_DATASET_V1` concatenated with the decimal scan index,
minimum target separation 16. Prefix 96/128/160 depends only on BASE263 label
counts (>=12 MAJOR and >=40 NO_MAJOR). No pool was generated on missing inputs.

Evidence must freeze before labels; GT risk scoring comes last. Primary U_near
requires AUC>=.80, one-sided frame permutation p<.05, LOFO BA>=.75,
delete-one-major minimum AUC>=.70, TYPE A MAJOR rate>=.60 and NO_MAJOR rate<=.20.
Ten-second block sensitivity is also required. No threshold/gamma/budget tuning.

These gates remain pending, not failed statistically. The present stop is an
input-contract stop before any scientific execution.
