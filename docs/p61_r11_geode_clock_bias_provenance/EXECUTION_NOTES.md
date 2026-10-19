# Preserved bounded audit execution history

The single sensor-message read ran at CODE_SHA
`8fe5d08241be44c821b55f9d773e0eb11656eea8` using `/usr/bin/python3`.
All selected message identity checks and historical initialization checks
completed, and five CSVs were published with exclusive creation. At the final
JSON write the program exited1 with:

```
TypeError: Object of type bytes is not JSON serializable
inspect_records.py line144: json.dump(result,f,indent=2)
```

Root cause: ROS1 connection header values can be bytes in Python3. The original
incomplete `record_audit.json` is retained unchanged, not counted as a valid
result. `metadata_text` strictly decodes metadata to UTF8 with a regression test.
The source fix changes serialization only; it does not change any time, sample,
bias or inference. `complete_saved_audit.py` finishes a separate JSON from saved
CSVs/native logs plus connection/topic metadata only. It does not reread sensor
messages, rerun a filter, or replace earlier output. Its actual version is
recorded separately in analysis_manifest.json. No source version after the
initial read is described as having performed that original read.
