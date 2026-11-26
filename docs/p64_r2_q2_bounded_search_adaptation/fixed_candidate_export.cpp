#include "scoring.hpp"
#include <pcl/io/pcd_io.h>

#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <stdexcept>
#include <string>

namespace {
namespace fs = std::filesystem;
using q1::Cloud;
using q1::Pose;
using q1::Scorer;

std::ofstream openOutput(const fs::path& path) {
  if (fs::exists(path)) {
    throw std::runtime_error("refusing to replace " + path.string());
  }
  std::ofstream stream(path);
  stream.exceptions(std::ios::failbit | std::ios::badbit);
  stream << std::setprecision(17);
  return stream;
}

Cloud::Ptr load(const std::string& path) {
  Cloud::Ptr cloud(new Cloud);
  if (pcl::io::loadPCDFile(path, *cloud) != 0 || cloud->empty()) {
    throw std::runtime_error("unreadable cloud " + path);
  }
  return cloud;
}

void writeXYZ(const fs::path& path, const Cloud& cloud) {
  auto out = openOutput(path);
  for (const auto& point : cloud) {
    const float xyz[] = {point.x, point.y, point.z};
    out.write(reinterpret_cast<const char*>(xyz), sizeof(xyz));
  }
}

void writeCells(const fs::path& path, Scorer& scorer) {
  auto out = openOutput(path);
  out << "cell_id,points,mx,my,mz,C00,C01,C02,C10,C11,C12,C20,C21,C22\n";
  for (const auto& [id, cell] : scorer.cells()) {
    if (cell.getPointCount() < q1::associationMinimum) {
      continue;
    }
    out << id << ',' << cell.getPointCount();
    const auto center = cell.getMean();
    const auto covariance = cell.getCov();
    for (int axis = 0; axis < 3; ++axis) {
      out << ',' << center(axis);
    }
    for (int row = 0; row < 3; ++row) {
      for (int column = 0; column < 3; ++column) {
        out << ',' << covariance(row, column);
      }
    }
    out << '\n';
  }
}

void exportPose(const fs::path& output, Scorer& scorer, const Cloud::ConstPtr& source,
                const Pose& pose, const std::string& id, std::ostream& summary) {
  std::vector<q1::PointScore> points;
  const auto result = scorer.fixed(*source, pose, &points);
  const double native = scorer.oracle(source, pose, 1);
  if (result.native != native || points.size() != source->size()) {
    throw std::runtime_error("fixed scorer does not match original native objective");
  }
  summary << id << ',' << result.native << ',' << result.native / result.total << ','
          << result.associated << ',' << result.total << ',' << result.park << ','
          << result.parkAssociated << '\n';

  auto out = openOutput(output / (id + "_points.csv"));
  auto cells = openOutput(output / (id + "_contributions.csv"));
  out << "point_id,x_L,y_L,z_L,x_M,y_M,z_M,associated_cells,native_reward\n";
  cells << "point_id,cell_id,mahalanobis_squared,native_reward\n";
  for (std::size_t i = 0; i < points.size(); ++i) {
    const auto& in = (*source)[i];
    const auto& scored = points[i];
    out << i << ',' << in.x << ',' << in.y << ',' << in.z << ','
        << scored.transformed.x << ',' << scored.transformed.y << ',' << scored.transformed.z << ','
        << scored.neighborhoods.size() << ',' << scored.total << '\n';
    for (const auto& neighbor : scored.neighborhoods) {
      cells << i << ',' << neighbor.cell << ',' << neighbor.mahalanobis << ',' << neighbor.score << '\n';
    }
  }
}
}  // namespace

int main(int argc, char** argv) {
  try {
    if (argc != 3) {
      throw std::runtime_error("usage: fixed_candidate_export plan.tsv exclusive_output_dir");
    }
    std::ifstream input(argv[1]);
    input.exceptions(std::ios::badbit);
    if (!input.is_open()) {
      throw std::runtime_error("unable to read candidate plan");
    }
    const fs::path output(argv[2]);
    if (!fs::create_directory(output)) {
      throw std::runtime_error("output directory already exists");
    }
    auto summary = openOutput(output / "score_summary.csv");
    summary << "candidate_id,native_sum,native_mean,associated,source_points,park_log,park_associated\n";
    std::string id, frame, map, scan;
    int processed = 0;
    while (input >> id >> frame >> map >> scan) {
      Pose pose;
      for (int row = 0; row < 4; ++row) {
        for (int column = 0; column < 4; ++column) {
          if (!(input >> pose(row, column))) {
            throw std::runtime_error("invalid pose within candidate plan");
          }
        }
      }
      const auto source = q1::downsample(load(scan));
      const auto target = q1::downsample(load(map));
      Scorer scorer(target);
      const auto sourceOutput = output / (frame + "_filtered_source_xyz.bin");
      if (!fs::exists(sourceOutput)) {
        writeXYZ(sourceOutput, *source);
      }
      if (!fs::exists(output / "filtered_map_xyz.bin")) {
        writeXYZ(output / "filtered_map_xyz.bin", *target);
        writeCells(output / "native_cells_world.csv", scorer);
      }
      exportPose(output, scorer, source, pose, id, summary);
      ++processed;
    }
    if (processed != 5) {
      throw std::runtime_error("expected exactly five fixed poses");
    }
  } catch (const std::exception& error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
