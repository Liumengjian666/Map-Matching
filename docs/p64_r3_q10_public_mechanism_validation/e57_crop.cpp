// Read bounded blocks through libE57Format. The E57 scan pose is applied once.
#include <E57SimpleReader.h>
#include <Eigen/Geometry>
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <stdexcept>
#include <vector>

int main(int argc, char** argv) {
  try {
    if (argc != 8) throw std::runtime_error("INPUT.e57 OUTPUT.xyzf CX CY CZ RADIUS MAX_RECORDS(0=all)");
    const Eigen::Vector3d center(std::stod(argv[3]),std::stod(argv[4]),std::stod(argv[5]));
    const double radius=std::stod(argv[6]);
    const uint64_t cap=std::stoull(argv[7]);
    e57::Reader file(argv[1],e57::ReaderOptions{});
    std::ofstream out(argv[2],std::ios::binary);
    if (!out) throw std::runtime_error("bad output");
    uint64_t read=0, emit=0;
    Eigen::Vector3d low=Eigen::Vector3d::Constant(std::numeric_limits<double>::infinity());
    Eigen::Vector3d high=-low;
    constexpr size_t block=65536;
    std::vector<double> x(block),y(block),z(block);
    std::vector<int8_t> flags(block);
    for (int64_t scan=0;scan<file.GetData3DCount();++scan) {
      e57::Data3D metadata;
      if (!file.ReadData3D(scan,metadata)) throw std::runtime_error("scan metadata error");
      const auto& p=metadata.pose;
      const Eigen::Quaterniond q(p.rotation.w,p.rotation.x,p.rotation.y,p.rotation.z);
      if (std::abs(q.norm()-1)>.00001) throw std::runtime_error("non-rotational E57 pose");
      const Eigen::Vector3d t(p.translation.x,p.translation.y,p.translation.z);
      std::cerr << std::setprecision(12) << "scan " << scan << " count " << metadata.pointCount
          << " pose_xyzw " << q.coeffs().transpose() << " translation " << t.transpose() << '\n';
      e57::Data3DPointsDouble data;
      data.cartesianX=x.data(); data.cartesianY=y.data(); data.cartesianZ=z.data();
      if (metadata.pointFields.cartesianInvalidStateField) data.cartesianInvalidState=flags.data();
      auto rdr=file.SetUpData3DPointsData(scan,block,data);
      while (!cap || read<cap) {
        const auto n=rdr.read();
        if (!n) break;
        for (size_t i=0;i<n && (!cap || read<cap);++i,++read) {
          if (data.cartesianInvalidState && flags[i]!=0) continue;
          const Eigen::Vector3d xyz(t+q.normalized()*Eigen::Vector3d(x[i],y[i],z[i]));
          if (!xyz.allFinite()) continue;
          low=low.cwiseMin(xyz);
          high=high.cwiseMax(xyz);
          if ((xyz-center).cwiseAbs().maxCoeff()>radius) continue;
          const Eigen::Vector3f f=xyz.cast<float>();
          out.write(reinterpret_cast<const char*>(f.data()),sizeof(float)*3);
          ++emit;
        }
      }
      rdr.close();
      if (cap && read>=cap) break;
    }
    if (!out) throw std::runtime_error("incomplete ROI");
    std::cout << "read " << read << " retained " << emit << " low " << low.transpose()
              << " high " << high.transpose() << '\n';
  } catch(const e57::E57Exception& e) {
    std::cerr << "E57:" << e.what() << " " << e.context() << '\n';
    return 2;
  } catch(const std::exception& e) {
    std::cerr << e.what() << '\n';
    return 1;
  }
}
