# Independent DCReg-adapted report, with full sensor support end time.
std_msgs/Header header
uint64 scan_end_ns
bool valid
bool translation_degenerate
bool rotation_degenerate
float64[3] translation_eigenvalues
float64[3] rotation_eigenvalues
float64[9] translation_axes
float64[9] rotation_axes
