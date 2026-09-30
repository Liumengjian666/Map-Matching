# Raw timed export result

Persistent output: `/media/jian/HIKVISION/paper rosbag/SuperLoc/Corridor01/results/p6_a3a_v3_input/`. It is a new directory; old `p6_i6c_framework/input`, derived bags and frozen artifacts are unchanged. Large binary is deliberately not committed to Git.

Export and all-point validation PASS: **2777 scans, 79,932,911 points, 3,197,316,440 bytes**. Each 40-byte little-endian record is `(float64 x,y,z,intensity; uint64 absolute_sensor_point_stamp_ns)` matching A2D's reader.

First scan start: 1517157219088119030 ns. Last terminal: 1517157499159287287 ns. Physical coverage ≈280.071168257 s, independent of rosbag record-time duration.

| Product | SHA256 |
|---|---|
| raw_timed_points.bin | 4ba09d8ae7004056dcc4e9d63d2ab09915748c7e68bc29d0dc8bd44a24fc95ff |
| raw_timed_catalog.csv | fdaf9607bc1933269f5f999ee044d37aa1eefabedc054f522da5078ce708123f |
| filter_scans.csv | 41d0b2040a5de8a8bd428c382a7b6dc18fabcaa8d3e7d2cc331018e0585edf1d |

No timestamp is reconstructed from point index. Official firing offsets are packet/payload semantics. Every decoded timestamp is checked within its scan interval; scan_start is the actual minimum. Export validates all finite coordinates, zero-time rejection, offsets, record size, counts, boundaries, raw provenance and schedule identity.

## Physical-time schedule

Legacy asset terminal exact matches: **0/2777**. `raw_scan_asset_timestamp_audit.csv` records each mismatch as `RAW_SCAN_ASSET_TIMESTAMP_MISMATCH`. No snapping is performed. New `filter_scans.csv` has only raw transaction_id and true terminal stamp; it contains no legacy poses. V3 reads ScanAsset identity from raw catalog rather than legacy scans.csv. Raw timestamps remain unchanged.

The raw archive includes acquisition during initialization. Producer V3 skips scans whose real scan-start precedes the Window handoff seed, reports `raw_scans_before_handoff`, and fails if no raw scan remains. It does not retroactively deskew an unavailable historical Window state, snap time, or reindex transactions. This is tested synthetically, not by a real localization run.

## Decoder provenance and overwrite protection

Offline helper is newly built from exact pinned official rawdata.cc and calibration.cc. Source hashes are exported from the actual loaded private driver library. RawData constructor/setup/unpack/firing functions and Calibration::read symbol providers must resolve to that same library. Python verifies these identities against the clean pinned checkout and its current files, then records helper/library binary hashes. LD_* interposition variables and GLIBC_TUNABLES are removed from the decoder environment.

Library: `5c6381b49a00cb549d5535566c748e31e9bd54178dcd72704aa96b7c5389fbed`. Helper: `42f80bb479bbd5779d9671c31bdfb8701e7ba650e0920af581c84c33c1f1f45f`. Decoder source: `845351b52d3e009ef7976bbf3e5772bd26630c5c02274d200d49fb76589348ee`. Export script: `8f123fa35da32ccfd448e11bfe6816842a08cf362a4461996363e80c28eec65e`; export execution Git SHA: `09a812e01eeea5b579006569a9d6f270764e4779`.

Known old-library LD_PRELOAD fails `runtime_decoder_symbol_interposition_rejected`. Replacing the sibling library with old decoder under the expected filename fails `runtime_decoder_symbol_unidentified`. Decoder output files use exclusive fopen, and exporter refuses nonempty output directories. V3 result output also uses exclusive fd-backed creation, so input aliases cannot truncate admitted files.

These are controlled build/provenance guards, not a cryptographic attestation mechanism against an attacker forging binaries and their identity functions. The fresh build and actual run identity are archived. The temporary build directory is not the sole location of any formal sensor input; the raw products and manifests are persisted on HIKVISION.
