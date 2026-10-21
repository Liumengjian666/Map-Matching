# Common-Prediction Pilot: NOT_RUN

The authorized rule is retained: one reference anchor at scan1 of the fixed21,
followed by20 test scans; TP uses only completed CONTROL_FULL history and past
times. With fewer than2 historical valid poses, hold the previous valid pose.
Otherwise extrapolate the historical SE(3) relative increment with actual dt.
All arms share TP, not three independent closed-loop systems. Rejected native
observations cannot be reclassified as legal map measurements.

No21-frame set or anchor was selected after coordinate admission failed. No
current/future test reference generated an online initial guess; no production
predictor, EKF, or candidate algorithm was changed. An executable CV interface
was not implemented or tested here. This document preserves the user contract;
it does not claim a completed predictor or a choice of unapproved fallback details.
