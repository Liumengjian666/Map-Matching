#include "corrected_ndt.hpp"
#include <iostream>
void check(bool value) { if(!value) throw std::runtime_error("grid regression failed"); }
int main() {
  screen::Cloud::Ptr cloud(new screen::Cloud);
  for(int x=0;x<3;++x) for(int y=0;y<3;++y) for(int z=0;z<3;++z)
    cloud->push_back(pcl::PointXYZ(.1*x,.1*y,.1*z));
  screen::finish(cloud);
  screen::Ndt archived;
  archived.configureTarget(cloud);
  check(archived.getResolution()==.8f);
  bool rejected=false;
  try {screen::requireCorrectedGrid(archived);} catch(const std::runtime_error&) {rejected=true;}
  check(rejected); // Checking only getResolution would incorrectly pass R4.
  screen::CorrectedNdt corrected;
  corrected.configureTarget(cloud);
  screen::requireCorrectedGrid(corrected);
  corrected.setInputSource(cloud);
  screen::requireCorrectedGrid(corrected);
  check(corrected.getStepSize()==archived.getStepSize());
  check(corrected.getTransformationEpsilon()==archived.getTransformationEpsilon());
  check(corrected.getMaximumIterations()==archived.getMaximumIterations());
  check(corrected.getOulierRatio()==archived.getOulierRatio());
  // No align in this unit test; no synthetic pose truth or old experiment rerun.
  std::cout<<"Actual XYZ Gaussian leaf 0.8f PASS; old getter-only false-positive rejected; other parameters unchanged\n";
}
