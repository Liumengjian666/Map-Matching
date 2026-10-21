# Preserved initial inspection failure

Command: `python3 docs/p62_r14b_newer_college/inspect_inputs.py` (script formerly in
`p62_r14b_newer_college_prior_map`, renamed without changing original outputs).
Exception: `ValueError: PCD payload length mismatch`, at first sampled scan.

`cloud_1583836591_182590976.pcd`: header 250 bytes; declared 65,536 × 29 =
1,900,544 data bytes; actual file 1,904,640 bytes, with 3,846 all-zero trailing
bytes. Local SHA256 and the same entry read directly from existing official
`ouster_scan-010.zip` both equal
`7c2fc3a20ef266c2ac3196173c534a1acdb4436224c264bc8f2dc1b1909803a2`.
No source bytes were changed. This is not evidence of extraction corruption.
The revised inspector records noncanonical length without claiming successful
decode. A real PCL reader must independently check any scans used by the pilot.

The failed attempt read the reference CSV for format/finite-value audit, before
the detailed R14B contract arrived. No pose error or registration was computed;
no reference value was used to choose frames or tune a method. This chronology
is disclosed rather than retroactively claiming the CSV was never opened.
