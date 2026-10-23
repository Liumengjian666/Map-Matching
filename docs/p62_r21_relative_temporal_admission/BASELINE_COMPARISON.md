# R21 five-policy developmental comparison

R19/R20 are frozen historical references; only D/E were newly run. All entries have 60 actual causal outputs, including fallbacks.

| 窗口/初始 | 策略 | 平移 RMSE / P95 / Max (m) | 旋转 RMSE / P95 / Max (°) |
|---|---|---|---|
| WEAK/C0 | R19 A | 0.260090 / 0.347973 / 0.391645 | 1.827785 / 2.449893 / 3.634624 |
| WEAK/C0 | R20 B Gate | 0.265708 / 0.353501 / 0.394637 | 1.858396 / 2.588184 / 4.156104 |
| WEAK/C0 | R20 C Multi | 0.263902 / 0.359318 / 0.391822 | 1.858832 / 2.538737 / 3.701357 |
| WEAK/C0 | R21 D Relative | 0.263902 / 0.359318 / 0.391822 | 1.858832 / 2.538737 / 3.701357 |
| WEAK/C0 | R21 E Confirm | 0.263902 / 0.359318 / 0.391822 | 1.858832 / 2.538737 / 3.701357 |
| WEAK/C3 | R19 A | 0.283121 / 0.485931 / 0.622737 | 2.808664 / 7.522614 / 9.018234 |
| WEAK/C3 | R20 B Gate | 4.164507 / 6.701954 / 7.000428 | 7.016976 / 10.693101 / 11.649997 |
| WEAK/C3 | R20 C Multi | 0.266655 / 0.356058 / 0.393714 | 1.862490 / 2.486138 / 4.308431 |
| WEAK/C3 | R21 D Relative | 0.266655 / 0.356058 / 0.393714 | 1.862490 / 2.486138 / 4.308431 |
| WEAK/C3 | R21 E Confirm | 0.266655 / 0.356058 / 0.393714 | 1.862490 / 2.486138 / 4.308431 |
| STRONG/C0 | R19 A | 0.159179 / 0.187299 / 0.208194 | 2.033813 / 2.833375 / 3.979948 |
| STRONG/C0 | R20 B Gate | 0.159179 / 0.187299 / 0.208194 | 2.033813 / 2.833375 / 3.979948 |
| STRONG/C0 | R20 C Multi | 0.159179 / 0.187299 / 0.208194 | 2.033813 / 2.833375 / 3.979948 |
| STRONG/C0 | R21 D Relative | 0.159179 / 0.187299 / 0.208194 | 2.033813 / 2.833375 / 3.979948 |
| STRONG/C0 | R21 E Confirm | 0.159179 / 0.187299 / 0.208194 | 2.033813 / 2.833375 / 3.979948 |
| STRONG/C3 | R19 A | 0.947759 / 2.145176 / 2.309913 | 7.788894 / 15.936936 / 22.616286 |
| STRONG/C3 | R20 B Gate | 0.351154 / 0.877361 / 1.177489 | 4.143002 / 9.983213 / 12.735693 |
| STRONG/C3 | R20 C Multi | 0.351154 / 0.877361 / 1.177489 | 4.143002 / 9.983213 / 12.735693 |
| STRONG/C3 | R21 D Relative | 0.158692 / 0.187519 / 0.211521 | 2.071567 / 2.913251 / 3.979997 |
| STRONG/C3 | R21 E Confirm | 0.160088 / 0.189015 / 0.258568 | 2.443303 / 3.670174 / 9.076500 |

Strong C3: D and E both meet both targets; E is worse than D and adds one confirmation. Other three role/arm combinations reproduce R20 Multistart exactly. No independent confirmation/generalization claim.
