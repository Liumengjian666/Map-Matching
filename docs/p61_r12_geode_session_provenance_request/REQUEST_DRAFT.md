# Formal provenance request — DRAFT, NOT SENT

以下英文正文供用户审核后自行决定是否发送。没有发送授权，没有自动发送动作。
历史归档README的Contact段建议在官方仓库反馈问题；本轮未验证当前维护状态、
未指定或虚构个人邮箱、未创建issue。用户应先确认合适的公开/私下渠道。
无需公开设备序列号等敏感信息；可用脱敏且稳定的设备/会话标识。

---

Subject: Session-specific timestamp and gyroscope calibration provenance for GEODE Shield_tunnel1_gamma

Dear GEODE maintainers,

Thank you for making GEODE available. We are checking the provenance needed to
interpret an offline LiDAR/IMU angular-increment comparison. Could you help
identify existing acquisition records for **Shield_tunnel1_gamma**, specifically
the Livox Avia LiDAR and the external Xsens `/imu/data` stream, not the Livox
internal IMU?

We are requesting documentation, not asking you to reproduce an experiment,
perform a new calibration, or generate records that were never collected.
For every item, “not recorded”, “unknown”, “not applicable”, or “exists but cannot
be shared” is a useful answer. General device documentation is welcome as
background, but please distinguish it from records applicable to this session.

## S1. Confirm the acquisition-session / released-file relationship

Our downloaded released file is:

```
Shield_tunnel1_gamma.bag
Size: 4500527001 bytes
SHA-256: 162ed09ab6b262f89ac46c45bac36ad2335a6f869b360b80d80ddf59d0b22091
```

This is our computed file fingerprint, not a claim that you published this
checksum. Could you confirm its recording/session identity, or identify the
matching release and any conversion/repackaging steps? If the acquisition bag
has different bytes, a traceable relationship is sufficient; identical hashes
are not required for different files. Please indicate the session, equipment
and validity period to which each shared configuration/log applies.

The diagnostic ROS-time interval is
**1706671804799723787–1706671805899593546 ns**, spanning scan-end endpoints
5154–5165 in our 1-based scan ledger. These indices are local bookkeeping;
timestamps and the released-file identity are the primary references. We are
not assuming that an epoch-formatted header independently proves physical sync.

## A. Actual acquisition hardware and software

- **A1:** Actual Livox model, firmware and a session-bound device identifier
  (redacted if necessary).
- **A2:** Actual external Xsens model, firmware and corresponding identifier.
  The archived device description says MTi-30; please confirm the session rather
  than relying only on the general device list.
- **A3:** Acquisition-time driver and SDK versions/commits for both devices,
  including any local patches and the applicable validity period.
- **A4:** The actual timestamp conversion code/version used to produce ROS
  `header.stamp` and Livox point offsets, including units and clock origins.
- **A5:** The launch files and configuration actually used for this acquisition,
  with version or session linkage if available.
- **A6:** The recording program/version and any postprocessing/repackaging
  history, especially whether bag record times or message headers were replaced.

## B. Physical clock synchronization

- **B1:** The actual synchronization architecture and clock master/source for
  this session, or confirmation that no such synchronization was used.
- **B2:** Applicable PPS/PTP/common-trigger/other synchronization settings,
  including disabled or unsupported modes where relevant.
- **B3:** Existing wiring/connection and synchronization startup procedure
  records, tied to the session or a demonstrably applicable setup.
- **B4:** Existing synchronization validation logs for this recording and their
  time coverage, including any dropout/re-lock events if recorded.
- **B5:** Any independently measured cross-sensor synchronization error: the
  measured quantity, reference instrument/source, sign, units, sample window,
  statistical definition, and uncertainty or bound. If no independent error
  measurement exists, please say so; a new estimate is not requested.
- **B6:** The traceable mapping from original device time to ROS header time for
  each sensor, including clock resets/wrap handling and the meaning of Livox
  `timebase`, `offset_time`, scan reference time and bag record time. In the
  checked messages, Livox `timebase` and ROS header have different origins.
  We do not interpret their numerical difference as a LiDAR–IMU clock offset.

Our own prepared replay bag is a **local derivative**, not a second official
recording (SHA-256
`a138b5ceb0e8ee4a2e5203e70b3943596811ff15d7a45495909934d3a1494331`).
Its writer uses message header time as bag record time. We do not ask you to
identify this local file, and zero header-minus-record differences in it are
not evidence of physical synchronization.

## C. Independent gyroscope calibration / stability evidence

- **C1:** Existing independent gyro calibration records for the actual Xsens
  device used in this session and their applicability/validity period.
- **C2:** Any independently documented stationary calibration interval during
  acquisition: start/end times, time base, sample count and the independent
  basis for declaring stationarity. Low gyro magnitude or our NDT residuals
  alone would not establish stationarity.
- **C3:** Any available session temperature records, applied compensation model,
  calibration version and relevant validity range. A statement that these were
  not recorded is sufficient.
- **C4:** If independent bias estimates exist, their method, sample interval,
  units, sensor coordinate frame and axis conventions; please distinguish them
  from bias states estimated by a LiDAR/IMU localization system.
- **C5:** Any independent bias uncertainty or time-stability validation relevant
  to the diagnostic interval. A later output from the same localization
  estimator would not by itself constitute independent validation.

For context only, our fixed reference bias is an estimator state exported at
ROS time **1706671587829840123 ns**:

```
[-0.0008725237314739192,
  0.0023757483405765214,
 -0.010976615256704694] rad/s, external IMU frame
```

It is 216.969883664 seconds before the interval above. We do **not** present it
as a factory calibration, a true bias measurement, or proof of stability. We
are not requesting a bias fitted to NDT motion or a clock offset chosen to
minimize a residual.

If convenient, please answer using the item IDs and provide existing filenames,
versions, session identifiers and units. We can hash shared artifacts on receipt;
you do not need to recreate unavailable metadata. Please share only material
you are authorized to disclose; sensitive identifiers may be redacted while
preserving the session linkage. No large sensor upload is needed for this
documentation request.

Until such evidence is available, we retain “clock synchronization not
certified” and “bias stability not evaluable”. These mean insufficient
provenance, not that the dataset is unsynchronized, miscalibrated or erroneous.

Thank you for your help.

---

附注：此文件是草稿，不是已发送邮件或issue，不包含虚构发件人身份或作者答复。
