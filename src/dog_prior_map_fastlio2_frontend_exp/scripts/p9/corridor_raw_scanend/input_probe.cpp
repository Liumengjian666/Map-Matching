// Admission diagnostic, not a baseline runner. Never aligns NDT or propagates
// a real scan from an unaccepted state. Identity poses below are test fixtures.
#include "dog_prior_map_fastlio2_frontend_exp/p7_replay_io.hpp"
#include "dog_prior_map_fastlio2_frontend_exp/fastlio2_frontend.hpp"
#include <iostream>
#include <stdexcept>

namespace p = dog_prior_map_fastlio2_frontend_exp;
int main(int argc, char** argv) {
  try {
    if(argc!=2)throw std::runtime_error("usage: EXTRACTED_INPUT_DIRECTORY");
    const std::string dir=argv[1];
    const auto imu=p::readP7Imu(dir+"/imu.csv");
    const auto scans=p::readP7TimedScans(dir+"/filter_scans.csv",dir+"/raw_timed_scan_index.csv");
    uint64_t points=0;
    for(const auto& scan:scans)points+=p::readP7PackedTimedCloud(dir+"/raw_timed_points.bin",scan).size();
    std::cout<<"P7_READER_PARITY=PASS scans="<<scans.size()<<" points="<<points<<" imu="<<imu.size()<<'\n';
    p::RuntimeParameters parameters;
    if(imu.size()<static_cast<size_t>(parameters.static_init_samples))throw std::runtime_error("short IMU");
    p::P7ImuVector samples(imu.begin(),imu.begin()+parameters.static_init_samples);
    p::FastLio2IkfomFrontend frontend(parameters);
    p::Pose3d identity_fixture;
    std::string reason;
    const bool accepted=frontend.initializeStatic(samples,identity_fixture,identity_fixture,&reason);
    std::cout<<"P7_STATIC_VARIANCE_ADMISSION="<<(accepted?"PASS_NOT_SUFFICIENT_FOR_REAL_INITIALIZATION":"FAIL")
      <<" reason="<<reason<<" samples="<<parameters.static_init_samples<<'\n'
      <<"NDT_CALLS=0 PROPAGATED_REAL_SCANS=0 GT_LOADED=NO\n";
    return 0; // A diagnosed scientific input blocker is an expected probe outcome.
  }catch(const std::exception& e){std::cerr<<"P7_INPUT_PROBE_ERROR="<<e.what()<<'\n';return 1;}
}
