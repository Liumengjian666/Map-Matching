#include <hdl_localization/pose_estimator.hpp>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pclomp/ndt_omp.h>

#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <stdexcept>

namespace fs = std::filesystem;
using Point = pcl::PointXYZI;
using Cloud = pcl::PointCloud<Point>;
using Transform = Eigen::Matrix4f;

class CausalNdt : public pclomp::NormalDistributionsTransform<Point, Point> {
public:
  std::size_t supported(const Cloud& aligned) const {
    std::vector<TargetGridLeafConstPtr> neighbors;
    std::size_t count = 0;
    for (const auto& point : aligned) {
      count += target_cells_.getNeighborhoodAtPoint7(point, neighbors) != 0;
    }
    return count;
  }
};

Cloud::Ptr load(const fs::path& path) {
  Cloud::Ptr cloud(new Cloud), filtered(new Cloud);
  if (pcl::io::loadPCDFile(path.string(), *cloud) != 0) {
    throw std::runtime_error("cannot load " + path.string());
  }
  pcl::VoxelGrid<Point> filter;
  filter.setInputCloud(cloud);
  filter.setLeafSize(0.1f, 0.1f, 0.1f);
  filter.filter(*filtered);
  if (filtered->empty()) {
    throw std::runtime_error("empty point cloud");
  }
  return filtered;
}

bool read_imu(std::istream& in, std::uint64_t& ns, Eigen::Vector3f& a, Eigen::Vector3f& w) {
  return static_cast<bool>(in >> ns >> a.x() >> a.y() >> a.z() >> w.x() >> w.y() >> w.z());
}

void write_pose(std::ostream& stream, const Transform& pose) {
  for (int r = 0; r < 4; ++r) {
    for (int c = 0; c < 4; ++c) {
      stream << ',' << pose(r, c);
    }
  }
}

int main(int argc, char** argv) {
  try {
    if (argc != 4) {
      throw std::runtime_error("usage: protocol_b_window INPUT_DIRECTORY DIFF_MAP OUTPUT_CSV");
    }
    const fs::path directory = argv[1];
    if (fs::exists(argv[3])) {
      throw std::runtime_error("output already exists");
    }
    std::ifstream anchor(directory/"anchor.tsv"), scans(directory/"scans.tsv"), imu(directory/"imu.tsv");
    Eigen::Vector3f seed;
    float yaw_degrees;
    if (!(anchor >> seed.x() >> seed.y() >> seed.z() >> yaw_degrees) || !scans || !imu) {
      throw std::runtime_error("missing frozen sensor or anchor input");
    }
    Eigen::Quaternionf orientation(Eigen::AngleAxisf(yaw_degrees * 0.017453292519943295f,
                                                     Eigen::Vector3f::UnitZ()));
    CausalNdt* ndt = new CausalNdt;
    pcl::Registration<Point, Point>::Ptr native(ndt);
    ndt->setNumThreads(1);
    ndt->setResolution(1.0f);
    ndt->setTransformationEpsilon(0.01);
    ndt->setNeighborhoodSearchMethod(pclomp::DIRECT7);
    const auto target = load(argv[2]);
    ndt->setInputTarget(target);
    hdl_localization::PoseEstimator tracker(native, seed, orientation, 2.0);

    std::ofstream out(argv[3]);
    out << std::setprecision(10)
        << "frame,stamp_ns,imu_samples,active_imu_samples,latest_imu_ns,iterations,converged,probability,source,supported";
    for (const auto* kind : {"pred", "ndt", "state"}) {
      for (int r = 0; r < 4; ++r) {
        for (int c = 0; c < 4; ++c) {
          out << ',' << kind << r << c;
        }
      }
    }
    out << '\n';
    Eigen::Vector3f acc, gyro;
    std::uint64_t stamp_ns, imu_ns;
    std::uint64_t first_scan_ns = 0;
    std::uint64_t last_imu = 0;
    bool have_imu = read_imu(imu, imu_ns, acc, gyro);
    std::string file;
    int frame = 0;
    while (scans >> stamp_ns >> file) {
      if (frame >= 72 || (frame && stamp_ns <= first_scan_ns)) {
        throw std::runtime_error("window size or timing contract violated");
      }
      if (!frame) first_scan_ns = stamp_ns;
      unsigned steps = 0, active = 0;
      while (have_imu && imu_ns <= stamp_ns) {
        if (imu_ns <= last_imu || imu_ns <= first_scan_ns) {
          throw std::runtime_error("noncausal IMU order");
        }
        ros::Time t;
        t.fromNSec(imu_ns);
        tracker.predict(t, acc, gyro);
        active += last_imu && imu_ns - first_scan_ns >= 2'000'000'000ULL;
        ++steps;
        last_imu = imu_ns;
        have_imu = read_imu(imu, imu_ns, acc, gyro);
      }
      const auto source = load(directory/file);
      ros::Time stamp;
      stamp.fromNSec(stamp_ns);
      const Transform predicted = tracker.matrix();
      const auto aligned = tracker.correct(stamp, source);
      const Transform final_ndt = ndt->getFinalTransformation();
      const Transform state = tracker.matrix();
      const auto support = ndt->supported(*aligned);
      out << frame << ',' << stamp_ns << ',' << steps << ',' << active << ',' << last_imu << ','
          << ndt->getFinalNumIteration() << ',' << ndt->hasConverged() << ','
          << ndt->getTransformationProbability() << ',' << source->size() << ',' << support;
      write_pose(out, predicted);
      write_pose(out, final_ndt);
      write_pose(out, state);
      out << '\n';
      if (support == 0 || !predicted.allFinite() || !final_ndt.allFinite() || !state.allFinite()) {
        throw std::runtime_error("invalid causal localization");
      }
      ++frame;
    }
    if (frame != 72 || have_imu || !imu.eof() || !scans.eof()) {
      throw std::runtime_error("incomplete frozen window");
    }
    return 0;
  } catch (const std::exception& exception) {
    std::cerr << exception.what() << '\n';
    return 1;
  }
}
