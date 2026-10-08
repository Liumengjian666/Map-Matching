// Frozen C++ numerical predicate, including float subtraction/norm and Quaternionf carrier.
#include "p9_r4_frozen_geometry.hpp"

extern "C" int p9_r4_oracle_distances(const char* nominal_xyzq, const char* candidate16,
                                      double* output) {
  try {
    const auto a = r4_frozen_geometry::parsePose(nominal_xyzq);
    const auto b = r4_frozen_geometry::parseMatrix16(candidate16);
    output[0] = r4_frozen_geometry::translationDistance(a, b);
    output[1] = r4_frozen_geometry::rotationDistanceDeg(a, b);
    return std::isfinite(output[0]) && std::isfinite(output[1]) ? 0 : 1;
  } catch (...) { return 1; }
}

#ifdef P9_R4_ORACLE_GEOMETRY_TEST_MAIN
#include <iostream>
int main() {
  double result[2];
  if (p9_r4_oracle_distances("0;0;0;0;0;0;1",
      "1;0;0;0.12;0;1;0;0.16;0;0;1;0;0;0;0;1", result) ||
      result[0] != static_cast<double>(0.2f) || result[0] <= 0.2 || result[1] > 1e-12) return 1;
  const auto p = r4_frozen_geometry::parsePose("5;-8;3;0.1;-0.2;0.3;0.9");
  std::ostringstream text; text << std::setprecision(17);
  for (int i=0; i<16; ++i) { if(i) text << ';'; text << p(i/4,i%4); }
  if (p9_r4_oracle_distances("5;-8;3;0.1;-0.2;0.3;0.9", text.str().c_str(), result) ||
      result[0] != 0 || result[1] > 1e-12) return 1;
  std::cout << "P9_R4_FROZEN_ORACLE_FLOAT_BOUNDARY_SELF_TEST=PASS\n";
}
#endif
