// Decode all official E57 scan records with local/world audit samples.
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
    if (argc != 4) throw std::runtime_error("INPUT.e57 OUTPUT.xyzf AUDIT.tsv");
    std::ofstream audit(argv[3]);
    if (!audit) throw std::runtime_error("bad audit output");
    audit << std::setprecision(17);
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
      Eigen::Vector3d local_low=Eigen::Vector3d::Constant(std::numeric_limits<double>::infinity());
      Eigen::Vector3d local_high=-local_low,world_low=local_low,world_high=local_high;
      uint64_t read_scan=0, emit_scan=0;
      audit << "POSE\t" << scan << '\t' << metadata.pointCount << '\t'
            << q.x() << '\t' << q.y() << '\t' << q.z() << '\t' << q.w()
            << '\t' << t.x() << '\t' << t.y() << '\t' << t.z() << '\n';
      auto rdr=file.SetUpData3DPointsData(scan,block,data);
      while (true) {
        const auto n=rdr.read();
        if (!n) break;
        for (size_t i=0;i<n;++i,++read,++read_scan) {
          if (data.cartesianInvalidState && flags[i]!=0) continue;
          const Eigen::Vector3d point(x[i],y[i],z[i]);
          const Eigen::Vector3d xyz(t+q*point);
          if (!xyz.allFinite()) continue;
          low=low.cwiseMin(xyz);
          high=high.cwiseMax(xyz);
          local_low=local_low.cwiseMin(point);
          local_high=local_high.cwiseMax(point);
          world_low=world_low.cwiseMin(xyz);
          world_high=world_high.cwiseMax(xyz);
          if (emit_scan == 0 || read_scan % 1000000 == 0)
            audit << "SAMPLE\t" << scan << '\t' << read_scan << '\t' << emit
                  << '\t' << point.x() << '\t' << point.y() << '\t' << point.z()
                  << '\t' << xyz.x() << '\t' << xyz.y() << '\t' << xyz.z() << '\n';
          const Eigen::Vector3f f=xyz.cast<float>();
          out.write(reinterpret_cast<const char*>(f.data()),sizeof(float)*3);
          ++emit; ++emit_scan;
        }
      }
      rdr.close();
      if (read_scan != static_cast<uint64_t>(metadata.pointCount)) throw std::runtime_error("scan count mismatch");
      audit << "END_SCAN\t" << scan << '\t' << read_scan << '\t' << emit_scan;
      for (auto point : {local_low,local_high,world_low,world_high})
        audit << '\t' << point.x() << '\t' << point.y() << '\t' << point.z();
      audit << '\n';
    }
    if (!out || !audit) throw std::runtime_error("incomplete map or audit");
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
