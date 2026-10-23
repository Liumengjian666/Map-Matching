# Prescreen CSV field audit

The single frozen prescreen completed successfully with 726 fixed-stride samples and 756 unique PCD reads including the selected window. Its JSON summary selects winner inventory index `7566` (one-based scan ordinal `7567`) and window ordinals `7552–7582`.

The raw `PRESCREEN_GEOMETRY.csv` header calls its first column `raw_ordinal_one_based`, but the executed writer emitted `Frame.index`, which is the zero-based inventory index. The raw CSV is retained unchanged as evidence. The selected winner row therefore begins with `7566`, while its path and timestamp match inventory index `7566`; the summary's separate `winner_raw_ordinal_one_based` is correctly `7567`.

The failure in the first preregistration-generation attempt was caused by interpreting that mislabeled column as one-based. No PCD was read by the failed preregistration attempt, no pose values or map were read, no NDT ran, and no window was changed. The preregistration helper now maps the observed zero-based field and independently verifies each selected row's path and timestamp against the canonical inventory. It does not modify or rerun the prescreen.
