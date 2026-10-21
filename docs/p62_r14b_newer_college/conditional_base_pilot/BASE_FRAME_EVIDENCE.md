# Base interpretation: evidence and explicit residual uncertainty

The controller approved PAPER_SUPPORTED_BASE_FRAME_ASSUMPTION before this run.
No axis interpretation was selected using overlap, NDT output or reference error.

- Original paper Sec V states reference poses refer to the device Base at the left-camera center: [official author PDF](https://www.robots.ox.ac.uk/~mobile/drs/Papers/2020IROS_ramezani.pdf). The browser could index the relevant primary-source passage but could not fetch the entire 11,599,448-byte PDF (size limit); no local full-PDF verification claimed.
- [Official pinned README](https://github.com/ori-drs/frontier_description/blob/0b039393334a7767e24a711bd05816bfc7f7485a/README.md) describes Base using x-forward/y-left/z-up and reference expressed in Base. Local prior evidence `../sources/frontier_README.md`, SHA256 2e0f8349fb588712a529ec7de1ff24c393ece1db7ac76c66403d24e3ed9a8167.
- [Official platform definition](https://ori-drs.github.io/newer-college-dataset/stereo-cam/platform-stereo/) distinguishes co-located Base and left optical axes. `../sources/platform.html` and `../sources/manifest.json` preserve source identities.
- Pinned official standalone/frontier Xacro files were read, not used to overwrite Kalibr: [standalone](https://github.com/ori-drs/frontier_description/blob/0b039393334a7767e24a711bd05816bfc7f7485a/urdf/frontier_standalone.urdf.xacro), [device](https://github.com/ori-drs/frontier_description/blob/0b039393334a7767e24a711bd05816bfc7f7485a/urdf/frontier.urdf.xacro).

The live official ground-truth page still names RS_C1 for stereo and Base for multicamera. This provenance ambiguity and the absence of a CSV-exporter binding are NOT resolved by this pilot. Mathematically valid matrices and small errors cannot certify the exported file axes. No Optical experiment was run.
