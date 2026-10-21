# Original PCDs: declared records preserved

All21 files: binary, FIELDS x y z intensity t reflectivity ring noise range; SIZE4 4 4 4 4 2 1 2 4; TYPE F F F F U U U U U; COUNT1 each. WIDTH1024 HEIGHT64 POINTS65536; header250 bytes; point_step29; declared records1,900,544 bytes. File length1,904,640 includes3846 trailing zero bytes outside declared points.

Python schema/padding checks and actual native PCL1.10 reads were completed for all21. PCL blob.data contains exactly declared records and equals them byte-for-byte; converted XYZ float bits equal original XYZ. PCL_READ_TEST.csv records all21 successful equivalence checks and Source FNV. No canonical copies; no input file changes; tail never interpreted as points. Any nonzero tail/schema/truncation fails closed.

Source uses the unchanged R8/R9 C++ preprocessRaw:0.5–80m,0.25m voxel, deterministic max1400 points, fixed scanner-frame16 sectors. Training=even; odd=8 validation blocks, CoverageV2 active blocks determined before optimization from point counts>=10, B>=4. No deskew; raw point t is not asserted to be certified absolute time. Motion distortion remains a limitation.
