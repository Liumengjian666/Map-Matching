#include "scoring.hpp"
#include <pcl/io/pcd_io.h>

#include <chrono>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <stdexcept>
#include <sys/resource.h>

using namespace q1;
namespace fs = std::filesystem;
using Clock = std::chrono::steady_clock;
constexpr double pi = 3.14159265358979323846;
constexpr double tolerance = 1e-6;

struct Frame {
  std::string id;
  fs::path map;
  fs::path scan;
  Pose predicted;
  Pose historical;
};

void require(bool okay, const std::string& message) {
  if (!okay) {
    throw std::runtime_error(message);
  }
}

bool readPose(std::istream& input, Pose& pose) {
  for (int r = 0; r < 4; ++r) {
    for (int c = 0; c < 4; ++c) {
      if (!(input >> pose(r,c))) return false;
    }
  }
  return pose.allFinite();
}

std::vector<Frame> readInput(const fs::path& path) {
  std::ifstream input(path);
  require(input.good(), "cannot read online input");
  std::vector<Frame> frames;
  Frame frame;
  while (input >> frame.id >> frame.map >> frame.scan) {
    require(readPose(input, frame.predicted) && readPose(input, frame.historical), "invalid online pose");
    frames.push_back(frame);
  }
  require(input.eof() && !frames.empty() && frames.size() <= 12, "online input must have 1 to 12 frames");
  return frames;
}

Cloud::Ptr load(const fs::path& path) {
  Cloud::Ptr cloud(new Cloud);
  require(pcl::io::loadPCDFile(path.string(), *cloud) == 0 && !cloud->empty(), "cannot load " + path.string());
  return cloud;
}

std::ofstream output(const fs::path& path, const std::string& header) {
  require(!fs::exists(path), "will not overwrite " + path.string());
  std::ofstream stream(path);
  require(stream.good(), "cannot write " + path.string());
  if (!header.empty()) stream << header << '\n';
  stream << std::setprecision(17);
  return stream;
}

std::string poseHeader() {
  return ",T00,T01,T02,T03,T10,T11,T12,T13,T20,T21,T22,T23,T30,T31,T32,T33";
}

void writePose(std::ostream& out, const Pose& p) {
  for (int r = 0; r < 4; ++r) {
    for (int c = 0; c < 4; ++c) out << ',' << p(r,c);
  }
}

Pose offset(const Pose& center, double dx, double dy, double drot) {
  Pose result = center;
  const float heading = std::atan2(center(1, 0), center(0, 0));
  result(0, 3) += std::cos(heading)*dx - std::sin(heading)*dy;
  result(1, 3) += std::sin(heading)*dx + std::cos(heading)*dy;
  result.block<3, 3>(0, 0) = Eigen::AngleAxisf(drot*pi/180.0, Eigen::Vector3f::UnitZ()) *
                             center.block<3, 3>(0, 0);
  return result;
}

void writeCloud(const fs::path& path, const Cloud& cloud) {
  auto stream = output(path, "");
  for (const auto& p : cloud) {
    const float xyz[3] = {p.x, p.y, p.z};
    stream.write(reinterpret_cast<const char*>(xyz), sizeof(xyz));
  }
}

void membership(const fs::path& path, const Cloud& raw, const Cloud& down) {
  std::map<Key, std::vector<std::size_t>> members;
  for (std::size_t i = 0; i < raw.size(); ++i) {
    members[voxel(raw[i], 10.0f)].push_back(i);
  }
  require(members.size() == down.size(), "source voxel membership size mismatch");
  auto out = output(path, "point_id,raw_point_id");
  std::size_t i = 0;
  for (const auto& entry : members) {
    Eigen::Vector3d mean = Eigen::Vector3d::Zero();
    for (auto id : entry.second) {
      mean += raw[id].getVector3fMap().cast<double>();
      out << i << ',' << id << '\n';
    }
    require((mean / entry.second.size() - down[i].getVector3fMap().cast<double>()).norm() < 0.002,
            "source voxel membership does not reproduce centroid order");
    ++i;
  }
}

void writeCells(const fs::path& path, Scorer& scorer) {
  auto out = output(path, "cell,count,mx,my,mz,c00,c01,c02,c10,c11,c12,c20,c21,c22");
  for (const auto& [id, leaf] : scorer.cells()) {
    if (leaf.getPointCount() < associationMinimum) continue;
    out << id << ',' << leaf.getPointCount();
    for (int j = 0; j < 3; ++j) out << ',' << leaf.getMean()[j];
    for (int j = 0; j < 3; ++j) {
      for (int k = 0; k < 3; ++k) out << ',' << leaf.getCov()(j,k);
    }
    out << '\n';
  }
}

void writeDetails(std::ostream& pts, std::ostream& contributions, const std::string& id,
                  const std::string& label, const std::vector<PointScore>& details) {
  for (std::size_t i = 0; i < details.size(); ++i) {
    const auto& p = details[i];
    pts << id << ',' << label << ',' << i << ',' << p.transformed.x << ',' << p.transformed.y
        << ',' << p.transformed.z << ',' << p.total << ',' << p.parkLog << ','
        << p.parkAssociated << ',' << p.neighborhoods.size() << ',' << details.size() << '\n';
    for (const auto& n : p.neighborhoods) {
      contributions << id << ',' << label << ',' << i << ',' << n.cell << ',' << n.mahalanobis
                    << ',' << n.score << '\n';
    }
  }
}

std::size_t independentSupport(Scorer& ndt, const Cloud& scan, const Pose& pose) {
  // Reconstruct occupancy from valid leaves by their mean, which lies within its voxel.
  std::map<Key, bool> occupied;
  for (const auto& [id, leaf] : ndt.cells()) {
    if (leaf.getPointCount() >= associationMinimum) {
      Point centroid;
      centroid.getVector3fMap() = leaf.getMean().cast<float>();
      occupied[voxel(centroid, 1)] = true;
    }
  }
  constexpr Key neighbors[] = {{0,0,0},{0,0,1},{0,0,-1},{0,1,0},{0,-1,0},{1,0,0},{-1,0,0}};
  Cloud transformed;
  pcl::transformPointCloud(scan, transformed, pose);
  std::size_t count = 0;
  for (const auto& p : transformed) {
    const auto key = voxel(p, 1);
    // The native code bounds-checks each queried offset voxel, including cells
    // whose central voxel lies just outside the map.
    count += std::any_of(std::begin(neighbors), std::end(neighbors), [&](const Key& d) {
      return occupied.count({key[0]+d[0], key[1]+d[1], key[2]+d[2]}) > 0;
    });
  }
  return count;
}

void qualify(const Frame& frame, Scorer& ndt, const Cloud::ConstPtr& source,
             const Cloud& target, std::ostream& out, std::ostream& pts, std::ostream& contrib) {
  auto poses = std::vector<std::pair<std::string, Pose>>{{"prediction", frame.predicted},
                                                        {"historical", frame.historical}};
  const Eigen::Vector3f anchor = target[target.size()/2].getVector3fMap();
  for (int axis = 0; axis < 3; ++axis) {
    for (int sign : {-1, 1}) {
      Pose translated = frame.historical;
      translated(axis,3) += sign * 0.10f;
      poses.push_back({"translation_" + std::to_string(axis) + "_" + std::to_string(sign), translated});
      Pose rotated = frame.historical;
      rotated.block<3,3>(0,0) = Eigen::AngleAxisf(sign * pi/180, Eigen::Vector3f::Unit(axis)) *
                               frame.historical.block<3,3>(0,0);
      poses.push_back({"rotation_" + std::to_string(axis) + "_" + std::to_string(sign), rotated});
    }
  }
  Pose wrong = frame.historical;
  wrong.block<3,3>(0,0) = Eigen::AngleAxisf(pi/2, Eigen::Vector3f::UnitZ()) * wrong.block<3,3>(0,0);
  wrong.block<3,1>(0,3) = anchor + Eigen::Vector3f(2,0,0);
  poses.push_back({"wrong_control", wrong});
  Pose empty = Pose::Identity();
  empty.block<3,1>(0,3) = anchor + Eigen::Vector3f::Constant(100000);
  poses.push_back({"no_association", empty});
  bool foundPartial = false;
  for (int attempt = 0; attempt < 63 && !foundPartial; ++attempt) {
    Pose partial = wrong;
    partial(attempt % 3,3) += (attempt / 3 - 10) * 0.5f;
    const auto s = ndt.fixed(*source, partial);
    if (s.associated > 0 && s.associated < s.total) {
      poses.push_back({"partial_association", partial});
      foundPartial = true;
    }
  }
  require(foundPartial, frame.id + " partial association missing");

  for (const auto& [label, pose] : poses) {
    std::vector<PointScore> details;
    const auto s = ndt.fixed(*source, pose, &details);
    const auto again = ndt.fixed(*source, pose);
    double reconstructed = 0;
    std::size_t counted = 0;
    for (const auto& p : details) {
      reconstructed += p.total;
      counted += !p.neighborhoods.empty();
    }
    const double one = ndt.oracle(source, pose, 1);
    const double four = ndt.oracle(source, pose, 4);
    const auto independentlyCounted = independentSupport(ndt, *source, pose);
    const double absolute = std::abs(one - reconstructed);
    const double relative = absolute/std::max(std::abs(one), 1e-12);
    const bool passed = relative <= tolerance && independentlyCounted == counted && counted == s.associated &&
        reconstructed == s.native && s.native == again.native && s.park == again.park && one == four &&
        (label != "no_association" || (s.associated == 0 && s.native == 0));
    out << frame.id << ',' << label << ',' << one << ',' << reconstructed << ',' << absolute << ','
        << relative << ',' << one/s.total << ',' << four << ',' << counted << ',' << independentlyCounted
        << ',' << s.total << ',' << (s.native == again.native) << ',' << passed;
    writePose(out, pose);
    out << '\n';
    out.flush();
    writeDetails(pts, contrib, frame.id, label, details);
    require(passed, frame.id + " score consistency failed at " + label);
  }
}

void grid(const Frame& frame, Scorer& ndt, const Cloud& source, std::ostream& out) {
  struct Trial { Pose pose; Score score; int x,y,yaw; };
  std::vector<Trial> trials;
  trials.reserve(4851);
  double maximum = -std::numeric_limits<double>::infinity();
  for (int x = -10; x <= 10; ++x) {
    for (int y = -10; y <= 10; ++y) {
      for (int a = -5; a <= 5; ++a) {
        const Pose t = offset(frame.historical, x * 0.2, y * 0.2, a);
        const auto s = ndt.fixed(source, t);
        require(std::isfinite(s.native) && std::isfinite(s.park), "non-finite grid");
        trials.push_back({t, s, x, y, a});
        maximum = std::max(maximum, s.park);
      }
    }
  }
  double denominator = 0;
  for (const auto& trial : trials) denominator += std::exp(alpha*(trial.score.park - maximum));
  require(trials.size() == 4851 && denominator >= 1, "invalid Park mass");
  for (std::size_t i = 0; i < trials.size(); ++i) {
    const auto& t = trials[i];
    const auto& s = t.score;
    out << frame.id << ',' << i << ',' << t.x+10 << ',' << t.y+10 << ',' << t.yaw+5 << ','
        << t.x*0.2 << ',' << t.y*0.2 << ',' << t.yaw << ',' << s.native/s.total << ',' << s.park << ','
        << s.associated << ',' << s.total << ',' << std::exp(alpha*(s.park - maximum))/denominator;
    writePose(out, t.pose);
    out << ',' << s.parkAssociated << '\n';
  }
}

struct Seed {
  int index;
  int rank;
  std::string kind;
  Pose pose;
};

auto readSeeds(const fs::path& path) {
  std::ifstream input(path);
  require(input.good(), "cannot read fit seeds");
  std::map<std::string, std::vector<Seed>> result;
  std::string id;
  Seed seed;
  while (input >> id >> seed.index >> seed.rank >> seed.kind) {
    require(readPose(input, seed.pose), "invalid fit seed");
    result[id].push_back(seed);
    require(result[id].size() <= 4, "fit budget exceeded");
  }
  require(input.eof(), "malformed fit seeds");
  return result;
}

void fit(const Frame& frame, Scorer& ndt, const Cloud::ConstPtr& source, const std::vector<Seed>& seeds,
         std::ostream& out, std::ostream& curvature, std::ostream& points, std::ostream& contributions) {
  ndt.setInputSource(source);
  ndt.setNumThreads(1);
  for (const auto& seed : seeds) {
    Cloud aligned;
    ndt.align(aligned, seed.pose);
    const Pose pose = ndt.getFinalTransformation();
    require(pose.allFinite(), "non-finite fit output");
    std::vector<PointScore> details;
    const auto s = ndt.fixed(*source, pose, &details);
    out << frame.id << ',' << seed.index << ',' << seed.rank << ',' << seed.kind << ','
        << ndt.hasConverged() << ',' << ndt.getFinalNumIteration() << ',' << s.native/s.total << ','
        << s.associated << ',' << s.total;
    writePose(out, pose);
    out << ',' << ndt.getTransformationProbability() << '\n';
    writeDetails(points, contributions, frame.id, "fit_" + std::to_string(seed.index), details);
    writeDetails(points, contributions, frame.id, "seed_" + std::to_string(seed.index),
                 (ndt.fixed(*source, seed.pose, &details), details));
    for (const auto* trial : {&seed.pose, &pose}) {
      Mat6 h;
      ndt.oracle(source, *trial, 1, &h);
      curvature << frame.id << ',' << seed.index << ',' << (trial == &pose ? "fit" : "grid");
      for (int i = 0; i < 6; ++i) {
        for (int j = 0; j < 6; ++j) curvature << ',' << h(i,j);
      }
      curvature << '\n';
    }
  }
}

int main(int argc, char** argv) {
  try {
    require(argc >= 4, "usage: qualify MODE online.tsv OUTDIR [fit_seeds.tsv]");
    const std::string mode = argv[1];
    const auto frames = readInput(argv[2]);
    const fs::path dir = argv[3];
    require(mode == "gate" || mode == "smoke" || mode == "grid" || mode == "fit", "unknown mode");
    require(!fs::exists(dir), "output directory already exists");
    fs::create_directories(dir);
    auto runtime = output(dir/"runtime_breakdown.csv", "sample_id,mode,load_preprocess_s,scorer_map_build_s,evaluation_s,max_rss_kib");
    if (mode != "gate" && mode != "fit") {
      auto out = output(dir/"grid.csv", "sample_id,index,ix,iy,ia,dx,dy,dyaw,native,park,associated,total,park_mass"+poseHeader()+",park_associated");
      for (const auto& frame : frames) {
        if (mode == "smoke" && frame.id != "H309") continue;
        if (mode == "grid" && frame.id == "H309") continue;
        const auto start = Clock::now();
        const auto target = downsample(load(frame.map));
        const auto source = downsample(load(frame.scan));
        const auto loadEnd = Clock::now();
        Scorer scorer(target);
        const auto modelEnd = Clock::now();
        grid(frame, scorer, *source, out);
        out.flush();
        rusage info;
        getrusage(RUSAGE_SELF, &info);
        runtime << frame.id << ',' << mode << ',' << std::chrono::duration<double>(loadEnd-start).count() << ','
                << std::chrono::duration<double>(modelEnd-loadEnd).count() << ','
                << std::chrono::duration<double>(Clock::now()-modelEnd).count() << ',' << info.ru_maxrss << '\n';
        runtime.flush();
      }
      return 0;
    }

    const auto seeds = mode == "fit" && argc == 5 ? readSeeds(argv[4]) : std::map<std::string, std::vector<Seed>>{};
    require(mode != "fit" || seeds.size() == frames.size(), "fit seed set incomplete");
    auto pts = output(dir/"point_scores.csv", "sample_id,pose_id,point_id,x,y,z,native_contribution,park_log,park_associated,neighbor_count,normalization_denominator");
    auto contrib = output(dir/"voxel_contributions.csv", "sample_id,pose_id,point_id,cell,mahalanobis_squared,native_contribution");
    auto out = mode == "gate" ? output(dir/"native_score_consistency.csv", "sample_id,pose_id,S_native,S_reconstructed,absolute,relative,native_reported_units,oracle_four_threads,associated,independent_associated,total,repeat_exact,pass"+poseHeader())
        : output(dir/"fit.csv", "sample_id,index,rank,seed_kind,reported_converged,iterations,native,associated,total"+poseHeader()+",native_optimizer_probability");
    std::string hessianHeader = "sample_id,index,stage";
    for (int i=0;i<6;++i) for (int j=0;j<6;++j) hessianHeader += ",h"+std::to_string(i)+std::to_string(j);
    auto curvature = mode == "fit" ? output(dir/"curvature.csv", hessianHeader) : std::ofstream();
    for (const auto& frame : frames) {
      const auto start = Clock::now();
      const auto rawSource = load(frame.scan);
      const auto target = downsample(load(frame.map));
      const auto source = downsample(rawSource);
      const auto loadEnd = Clock::now();
      Scorer scorer(target);
      const auto modelEnd = Clock::now();
      if (mode == "gate") {
        const auto prefix = dir/frame.id;
        writeCloud(prefix.string()+"_source_xyz.bin", *source);
        writeCloud(prefix.string()+"_target_xyz.bin", *target);
        membership(prefix.string()+"_membership.csv", *rawSource, *source);
        if (frame.id == "H309" || frame.id == "O164") {
          writeCells(dir/("cells_"+frame.id.substr(0,1)+".csv"), scorer);
        }
        qualify(frame, scorer, source, *target, out, pts, contrib);
      } else {
        fit(frame, scorer, source, seeds.at(frame.id), out, curvature, pts, contrib);
      }
      rusage info;
      getrusage(RUSAGE_SELF, &info);
      runtime << frame.id << ',' << mode << ',' << std::chrono::duration<double>(loadEnd-start).count() << ','
              << std::chrono::duration<double>(modelEnd-loadEnd).count() << ','
              << std::chrono::duration<double>(Clock::now()-modelEnd).count() << ',' << info.ru_maxrss << '\n';
      runtime.flush();
    }
    return 0;
  } catch (const std::exception& e) {
    std::cerr << e.what() << '\n';
    return 2;
  }
}
