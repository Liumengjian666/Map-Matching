# Pre-run build failure retained

The first build of tools/shadow.cpp failed before any real align:

`shadow.cpp:55: error: 'begin' was not declared in this scope`

Root cause: range-for over Eigen::Matrix<double,6,1> assumes iterable begin/end support absent from installed Eigen3.3.7. Both spectrum and vector output loops were changed to explicit indices0..5, the same compatible serialization pattern used by frozen R7. No mathematics, threshold or input changed. Full rebuild is the regression check for this compilation error. It is a research-driver integration error, not NDT algorithm failure or a discarded real-data run.
