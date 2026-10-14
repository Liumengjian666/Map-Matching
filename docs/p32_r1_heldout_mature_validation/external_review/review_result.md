**DATA_NOT_READY is supported by the supplied evidence, but the proposed missing asset is slightly over-specified.** The defensible conclusion is that neither candidate has established eligibility within this inspection—not that eligible data cannot exist.

- **Corridor02 has a substantive blocker:** the released GT’s sensor reference point remains unknown. SuperOdom’s LiDAR pose output does not establish the released GT’s origin without a supported connection. Relative-drift evaluation removes the need for absolute map-to-GT alignment; it does not remove the contract’s reference-point requirement.

- **Floor01 has an actual non-GT operational initialization.** Calling H1 nonexistent, GT-derived, or automatically unacceptable would contradict the evidence. Selecting a transform direction through prior-map consistency can be an initialization adapter. However, finite, converged NDT alone does not establish the complete relationship between the runtime body, initialization and map. The supplied evidence leaves that relationship unconfirmed. Official semantic direction is one way to establish it; a supported operational coordinate chain could also satisfy the contract.

- **Floor01’s independence remains unknown.** Historical R7/P6/P9 use does not establish current-algorithm tuning. Conversely, the fact that P31 C development used Corridor01 does not complete the expressly missing Floor01 independence trace. This supports withholding admission, not declaring Floor01 disqualified.

The actionable correction is to avoid making a **literal start timestamp** mandatory. The contract requires a legitimate association with runtime initialization. That could be established through a documented first-scan relationship without a timestamp field. Likewise, the initial YAML’s difference from `start_pose.txt` is an adapter issue; it becomes a blocker only where the necessary interpretation is unsupported.

A narrower minimal NEXT would be:

> ONE traceable Corridor02 frame/init specification identifying the released GT reference point and supporting the initial transform’s source/target frames, sensor/body relationship, and association with the first runtime scan.

That is one concrete missing asset addressing the known semantic gaps. Its receipt would resolve those gaps, not automatically certify every admission requirement.

No other substantive gate error is evident: record/header time differences are not a mismatch gate here; absolute map-to-GT alignment is optional for the allowed relative evaluation; deferred full hashes and coverage checks do not explain the present rejection. **F/G NOT_RUN with null metrics is correct.**