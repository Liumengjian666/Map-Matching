#include "dog_prior_map_localization/visual_aided_initial_guess.hpp"
#include <fstream>
#include <iostream>
#include <stdexcept>
#include <unistd.h>
int main() {
  char name[] = "/tmp/p17_test_XXXXXX";
  const int fd = mkstemp(name); if (fd < 0) return 1; close(fd);
  auto write = [&](const std::string& row) {
    std::ofstream f(name);
    f << "scan_ns,prev_scan_ns,ref_image_ns,cur_image_ns,valid,dx,dy,dz,depth_max_point_ns,max_imu_ns\n" << row << '\n';
  };
  try {
    dog_prior_map_localization::VisualAidedInitialGuess adapter;
    write("1100000000,1000000000,990000000,1090000000,1,0.1,0.2,0.3,980000000,1100000000");
    adapter.load(name);
    Eigen::Matrix4d previous=Eigen::Matrix4d::Identity(), guess=previous;
    previous(0,3)=2; std::string reason;
    if (!adapter.apply(1000000000,1100000000,previous,guess,reason) ||
        std::abs(guess(0,3)-2.1)>1e-12 || guess.block<3,3>(0,0)!=Eigen::Matrix3d::Identity())
      throw std::runtime_error("valid translation/rotation test");
    const auto before=guess;
    if (adapter.apply(1,1100000000,previous,guess,reason) || guess!=before)
      throw std::runtime_error("fallback changed guess");
    write("1100000000,1000000000,990000000,1110000000,1,0.1,0.2,0.3,980000000,1100000000");
    bool rejected=false;try {adapter.load(name);}catch(const std::exception&){rejected=true;}
    if (!rejected) throw std::runtime_error("future image accepted");
    write("1100000000,1000000000,990000000,1090000000,1,nan,0.2,0.3,980000000,1100000000");
    rejected=false;try {adapter.load(name);}catch(const std::exception&){rejected=true;}
    if (!rejected) throw std::runtime_error("nan accepted");
    write("1100000000,1000000000,990000000,1090000000,1,0.1,0.2,0.3,100000000,1100000000");
    rejected=false;try {adapter.load(name);}catch(const std::exception&){rejected=true;}
    if (!rejected) throw std::runtime_error("stale depth accepted");
    unlink(name);std::cout << "VISUAL_INITIAL_GUESS_CONTRACT_PASS\n";return 0;
  } catch(const std::exception& e) {unlink(name);std::cerr << e.what()<<'\n';return 1;}
}
