#include "scoring.hpp"
#include <pcl/io/pcd_io.h>
#include <chrono>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <stdexcept>

namespace fs = std::filesystem;
using q1::Cloud;
using q1::Pose;
using Clock = std::chrono::steady_clock;
double elapsed(Clock::time_point start) {
  return std::chrono::duration<double>(Clock::now() - start).count();
}
Cloud::Ptr load(const std::string& path) {
  Cloud::Ptr cloud(new Cloud);
  if (pcl::io::loadPCDFile(path, *cloud) != 0 || cloud->empty()) {
    throw std::runtime_error("invalid cloud: " + path);
  }
  return cloud;
}
std::ofstream output(const fs::path& path) {
  if (fs::exists(path)) throw std::runtime_error("would overwrite " + path.string());
  std::ofstream out(path, std::ios::binary);
  out.exceptions(std::ios::badbit | std::ios::failbit);
  return out;
}
template<class T>
void write(std::ostream& out, const T* data, std::size_t length) {
  out.write(reinterpret_cast<const char*>(data), sizeof(T) * length);
}
void xyz(const fs::path& path, const Cloud& cloud) {
  auto out = output(path);
  for (const auto& p : cloud) {
    const float point[] = {p.x, p.y, p.z};
    write(out, point, 3);
  }
}
q1::Mat6 info(const q1::Point& p, const q1::PointScore& score, const Pose& t, q1::Scorer& scorer) {
  Eigen::Vector3d v = p.getVector3fMap().cast<double>();
  Eigen::Matrix3d cross;
  cross << 0, -v.z(), v.y(), v.z(), 0, -v.x(), -v.y(), v.x(), 0;
  const Eigen::Matrix3d r = t.topLeftCorner<3,3>().cast<double>();
  Eigen::Matrix<double,3,6> j;
  j << r, -r * cross / 5.0;
  Eigen::Matrix3d w = Eigen::Matrix3d::Zero();
  for (const auto& cell : score.neighborhoods) {
    w += cell.score * scorer.cells().at(cell.cell).getInverseCov();
  }
  return j.transpose() * w * j;
}
int main(int argc, char** argv) {
  try {
    if (argc != 3) throw std::runtime_error("score_fixed PLAN OUTDIR");
    std::ifstream plan(argv[1]);
    std::string frame, map_path, source_path;
    int k, j;
    if (!(plan >> frame >> map_path >> source_path >> k >> j)) throw std::runtime_error("empty plan");
    const fs::path dir(argv[2]);
    if (!fs::create_directory(dir)) throw std::runtime_error("output must be new");
    const auto start = Clock::now();
    const auto map = load(map_path);
    const auto target = q1::downsample(map);
    const auto source = q1::downsample(load(source_path));
    q1::Scorer scorer(target);
    const auto load_time = elapsed(start);
    if (j == 0) {
      xyz(dir / "map.bin", *map);
      xyz(dir / "target.bin", *target);
      xyz(dir / "source.bin", *source);
    }
    auto log = output(dir / "timing.csv");
    log << "frame,k,j,n,total,associated,score_s,oracle_s,info_write_s,load_model_s\n";
    log << std::setprecision(17);
    const auto expected_frame = frame, expected_map = map_path, expected_source = source_path;
    do {
      if (frame != expected_frame || map_path != expected_map || source_path != expected_source)
        throw std::runtime_error("mixed-frame plan");
      Pose t;
      for (int a=0;a<4;++a)
        for (int b=0;b<4;++b)
          if (!(plan >> t(a,b))) throw std::runtime_error("malformed pose");
      auto tick = Clock::now();
      std::vector<q1::PointScore> details;
      auto score = scorer.fixed(*source, t, &details);
      const double score_time = elapsed(tick);
      tick = Clock::now();
      if (j == 0 && scorer.oracle(source,t,1) != score.native)
        throw std::runtime_error("original native scorer parity failed");
      const double oracle_time = elapsed(tick);
      tick = Clock::now();
      double total = 0;
      const auto id = std::to_string(k) + "_" + std::to_string(j);
      auto points = output(dir / (id + ".bin"));
      for (const auto& p : details) {
        const double values[] = {p.total, static_cast<double>(p.neighborhoods.size())};
        write(points, values, 2);
        total += p.total;
      }
      if (total != score.native) throw std::runtime_error("sum parity failed");
      if (j == 0) {
        auto result = output(dir / (id + "_info.bin"));
        for (std::size_t point = 0; point < details.size(); ++point) {
          const auto h = info((*source)[point], details[point], t, scorer);
          write(result, h.data(), 36);
        }
      }
      log << frame << ',' << k << ',' << j << ',' << details.size() << ',' << score.native << ','
          << score.associated << ',' << score_time << ',' << oracle_time << ',' << elapsed(tick) << ','
          << (k == 0 && j <= 1 ? load_time : 0) << '\n';
    } while (plan >> frame >> map_path >> source_path >> k >> j);
  } catch (const std::exception& e) {
    std::cerr << e.what() << '\n';
    return 1;
  }
}
