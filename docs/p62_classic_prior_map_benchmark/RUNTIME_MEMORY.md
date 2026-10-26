# Resource status and future measurement scope

No new NDT, EKF or formal player process was run. Outdoor runtime/RSS/rate are NR. Published processing times, where cited, are author-system measurements and must not be converted into cross-hardware speedups.

The metadata inspection read official JSON, four131072-byte ZIP directory tails and one bounded metadata-entry range. It did not download the multi-GB sensor payloads or install external algorithms. Raw-asset expected sizes, expanded sizes, MD5s and disk capacities are recorded in sources/OUTDOOR_ASSET_METADATA.json. Metadata entry CRCs are verified; this does not certify full ZIP MD5s or sensor-message equality.

When admitted: measure map build/load separately, source decoding/preprocessing, nominal NDT, extra NDT, quality evaluation, observation handling and complete scan-to-final-state cycle. Report Mean/P95/Max and all event-frame maxima, actual input sensor span versus wall time, real-time factor/backlog/dropped-message counts, and denominator including empty/rejected frames. A low mean callback cannot certify10Hz complete localization.

Sample NDT, EKF and input/player RSS at the same fixed interval; report individual peaks, simultaneous pair and all related-process peaks. Separate filesystem cache, map preprocessing and raw data storage from resident localization memory. Keep actual target hash/points/tiles and target-switch count so map selection differences are visible. Record cold initialization separately from per-frame cost. Published and own hardware/scope remain distinct.

Proposed large asset destination is the existing external filesystem, not root. Both compressed+expanded outdoor sequences require17038872617 bytes before derived data; root cannot preserve20GiB headroom for that plan, external storage can. No cleanup or historical-asset deletion was performed.
