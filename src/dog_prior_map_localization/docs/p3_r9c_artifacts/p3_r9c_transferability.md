# P3-R9C old-mechanism evidence transferability

Decision: `NOT_DIRECTLY_TRANSFERABLE`.

Failure comparison category: `MIXED_STAGE_DEPENDENT_EFFECT`.

Evidence to use: common-anchor paired full-population metrics, the five predeclared W0–W4 windows, recomputed persistent crossings, and the rapid 1 m→5 m interval. W3 paired translation mean delta (legacy−mature) is 0.013477 m; W4 is 0.187970 m. Legacy persistent crossings: 0.25m=37.8203678131s; 0.5m=98.534662962s; 1m=138.271308899s; 2m=138.977272987s; 5m=140.086675882s. Mature IMU persistent crossings: 0.25m=37.8203678131s; 0.5m=98.534662962s; 1m=138.674692869s; 2m=138.977272987s; 5m=140.389256001s.

R7I/R8 fixed-cloud counterfactuals are conditional on legacy CV-deskew observations. Because R9B changes a closed-loop infrastructure (IMU propagation → deskew → NDT → EKF correction → later state/deskew), those results do not by themselves establish a mature-pipeline mechanism. If the failure largely persists, the old evidence still requires a mature-pipeline retest before being treated as mechanism evidence. If the timeline materially changes or is stage-dependent, do not transfer the old mechanism interpretation directly.

No causal claim is made that point-wise deskew alone caused an outcome. No physical root cause, collision causality, wrong mode, multimodality, or Hessian/geometry degeneracy is claimed. `P4_ALLOWED = NO`.
