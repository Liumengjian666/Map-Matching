# Historical window exposure check (before R22 align)

Checked START archive reports/protocols and exact PCD basename/integer timestamps across archived MD/JSON/CSV; supplementary root-workspace report search found no additional matching experiment. Exact identity hits are saved in logs/HISTORICAL_EXACT_ID_HITS.txt. No historical evaluation-row pose/error values were used to choose these windows.

P41 first scan; P43 first/100s/150s; P44/P45 first300; P46/P47/P49 first1800; R14B768–787; R16/R17 7552–7582; R19–R21 10972–11032 and14542–14602 do not overlap. P12 index4800 concerns Loop3, not this dataset.

All four windows are eligible for temporal holdout of method/quality-threshold development in the available records. This is NOT unseen raw-data acquisition: full inventory existed and R16 sampled raw geometry without map registration/reference error. This bounded record check is not proof that no unavailable external experiment ever existed.

- W1: raw2600–2660; prior input-only geometry ordinals [2607, 2627, 2647]; no documented method-selection/error-tuning exposure.
- W2: raw4800–4860; prior input-only geometry ordinals [4807, 4827, 4847]; no documented method-selection/error-tuning exposure.
- W3: raw6500–6560; prior input-only geometry ordinals [6507, 6527, 6547]; no documented method-selection/error-tuning exposure.
- W4: raw9400–9460; prior input-only geometry ordinals [9407, 9427, 9447]; no documented method-selection/error-tuning exposure.
