#include "spatial_split.hpp"

#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/conversions.h>
#include <boost/property_tree/json_parser.hpp>
#include <openssl/sha.h>
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <map>
#include <numeric>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

using Point = pcl::PointXYZI;
using Cloud = pcl::PointCloud<Point>;
using Tree = boost::property_tree::ptree;
using Row = std::map<std::string, std::string>;

void require(bool condition, const std::string& message) {
  if (!condition) throw std::runtime_error(message);
}

std::vector<std::string> csv_fields(const std::string& line) {
  std::vector<std::string> fields;
  std::stringstream input(line);
  std::string field;
  while (std::getline(input, field, ',')) fields.push_back(field);
  return fields;
}

std::vector<Row> read_csv(const std::string& path) {
  std::ifstream input(path);
  require(input.good(), "cannot open CSV: " + path);
  std::string line;
  require(static_cast<bool>(std::getline(input, line)), "empty CSV: " + path);
  const auto header = csv_fields(line);
  std::vector<Row> rows;
  while (std::getline(input, line)) {
    if (line.empty()) continue;
    const auto values = csv_fields(line);
    require(values.size() == header.size(), "CSV column count mismatch: " + path);
    Row row;
    for (size_t i = 0; i < header.size(); ++i) row.emplace(header[i], values[i]);
    rows.push_back(std::move(row));
  }
  return rows;
}

const std::string& field(const Row& row, const std::string& name) {
  const auto it = row.find(name);
  if (it == row.end()) throw std::runtime_error("missing CSV field: " + name);
  return it->second;
}

std::string xyz_sha(const Cloud& cloud) {
  std::vector<float> xyz;
  xyz.reserve(cloud.size() * 3);
  for (const auto& point : cloud.points) {
    xyz.push_back(point.x); xyz.push_back(point.y); xyz.push_back(point.z);
  }
  unsigned char digest[SHA256_DIGEST_LENGTH];
  SHA256(reinterpret_cast<const unsigned char*>(xyz.data()), xyz.size() * sizeof(float), digest);
  std::ostringstream output;
  for (unsigned char byte : digest) output << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(byte);
  return output.str();
}

std::string file_sha(const std::string& path) {
  std::ifstream input(path, std::ios::binary);
  require(input.good(), "cannot hash file: " + path);
  SHA256_CTX context; SHA256_Init(&context);
  char buffer[1 << 16];
  while (input) {
    input.read(buffer, sizeof(buffer));
    const auto count = input.gcount();
    if (count > 0) SHA256_Update(&context, buffer, static_cast<size_t>(count));
  }
  unsigned char digest[SHA256_DIGEST_LENGTH]; SHA256_Final(digest, &context);
  std::ostringstream output;
  for (unsigned char byte : digest) output << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(byte);
  return output.str();
}

Cloud::Ptr load_voxel(const std::string& path, size_t expected_raw_count) {
  pcl::PCLPointCloud2 blob;
  require(pcl::io::loadPCDFile(path, blob) == 0, "PCD load failed: " + path);
  require(blob.width * blob.height == expected_raw_count, "raw point count mismatch: " + path);
  Cloud::Ptr raw(new Cloud);
  pcl::fromPCLPointCloud2(blob, *raw);
  for (auto& point : raw->points) point.intensity = 0.0f;
  pcl::VoxelGrid<Point> voxel;
  voxel.setLeafSize(0.1f, 0.1f, 0.1f);
  voxel.setInputCloud(raw);
  Cloud::Ptr filtered(new Cloud);
  voxel.filter(*filtered);
  return filtered;
}

double quantile(std::vector<double> values, double probability) {
  require(!values.empty(), "cannot summarize empty residual distribution");
  std::sort(values.begin(), values.end());
  const double index = probability * static_cast<double>(values.size() - 1);
  const size_t low = static_cast<size_t>(std::floor(index));
  const size_t high = static_cast<size_t>(std::ceil(index));
  const double fraction = index - static_cast<double>(low);
  return values[low] * (1.0 - fraction) + values[high] * fraction;
}

struct CandidateResiduals {
  std::vector<double> distances_squared;
  uint64_t supported_points = 0;
  uint64_t supported_blocks = 0;
};

struct BlockResiduals {
  std::vector<double> prior;
  std::vector<double> ndt;
  uint64_t prior_support = 0;
  uint64_t ndt_support = 0;
};

Eigen::Matrix4f matrix_from_row(const Row& row, const std::string& prefix) {
  Eigen::Matrix4f result;
  for (int r = 0; r < 4; ++r) for (int c = 0; c < 4; ++c)
    result(r,c) = std::stof(field(row, prefix + std::to_string(r) + std::to_string(c)));
  require(result.allFinite(), "non-finite candidate transform");
  return result;
}

int main(int argc, char** argv) {
  try {
    require(argc == 7, "usage: score_heldout RUN_DIR INPUT_FREEZE_LOCAL.json R10_SOURCES.csv MAP_PCD PARTITION_SUPPORT.csv OUTPUT_DIR");
    const std::string run_dir = argv[1], freeze_path = argv[2], source_contract_path = argv[3];
    const std::string map_path = argv[4], partition_support_path = argv[5], output_dir = argv[6];
    Tree receipt; boost::property_tree::read_json(run_dir + "/RUN_RECEIPT.json", receipt);
    require(receipt.get<std::string>("reason") == "COMPLETED_2375" &&
            receipt.get<size_t>("completed") == 2375 && receipt.get<bool>("test_GT_access") == false,
            "B run is incomplete or has invalid reference-access status");
    Tree seal; boost::property_tree::read_json(run_dir + "/ONLINE_SEAL.json", seal);
    const auto& sealed_files = seal.get_child("files");
    for (const auto* name : {"ONLINE_RESULTS.csv", "NATIVE_NDT.csv", "SPATIAL_SPLIT.csv"}) {
      const std::string path = run_dir + "/" + name;
      require(file_sha(path) == sealed_files.get<std::string>(name), std::string("run seal mismatch: ") + name);
    }

    Tree freeze; boost::property_tree::read_json(freeze_path, freeze);
    require(freeze.get<std::string>("map_path") == map_path, "map path differs from frozen input");
    Cloud::Ptr raw_target(new Cloud);
    require(pcl::io::loadPCDFile(map_path, *raw_target) == 0, "map PCD load failed");
    require(raw_target->size() == freeze.get<size_t>("map_points"), "raw map count differs from freeze");
    pcl::VoxelGrid<Point> map_voxel;
    map_voxel.setLeafSize(0.1f, 0.1f, 0.1f);
    map_voxel.setInputCloud(raw_target);
    Cloud::Ptr target(new Cloud);
    map_voxel.filter(*target);
    require(target->size() == freeze.get<size_t>("target_points"), "target point count differs from freeze");
    require(xyz_sha(*target) == freeze.get<std::string>("target_xyz_sha256"), "target XYZ SHA differs from freeze");
    pcl::KdTreeFLANN<Point> tree;
    tree.setInputCloud(target);

    const auto sources = read_csv(source_contract_path);
    const auto partition_rows = read_csv(partition_support_path);
    const auto ndt_rows = read_csv(run_dir + "/NATIVE_NDT.csv");
    const auto split_rows = read_csv(run_dir + "/SPATIAL_SPLIT.csv");
    const auto online_rows = read_csv(run_dir + "/ONLINE_RESULTS.csv");
    require(sources.size() == 2375 && partition_rows.size() == 2375 && ndt_rows.size() == 2375 &&
            split_rows.size() == 2375 && online_rows.size() == 2375, "expected 2375 sealed rows in every input");

    const std::string frame_path=output_dir+"/SCORES.csv", block_path=output_dir+"/BLOCK_SCORES.csv";
    std::ifstream old_frame(frame_path),old_block(block_path);
    require(!old_frame.good()&&!old_block.good(),"refusing to overwrite prior shadow score files");
    std::ofstream frame_output(frame_path), block_output(block_path);
    require(frame_output.good() && block_output.good(), "cannot create score output files");
    frame_output << "raw_ordinal,stamp_ns,validation_points,validation_blocks,prior_support_points,ndt_support_points,prior_support_blocks,ndt_support_blocks,prior_mean_d2,prior_q10_d2,prior_q50_d2,prior_q90_d2,prior_q95_d2,prior_max_d2,ndt_mean_d2,ndt_q10_d2,ndt_q50_d2,ndt_q90_d2,ndt_q95_d2,ndt_max_d2,prior_median_block_d2,ndt_median_block_d2,delta_ndt_minus_prior_m2,blocks_prefer_ndt,blocks_prefer_prior,blocks_tied,shadow_preference\n";
    block_output << "raw_ordinal,stamp_ns,block_x,block_y,block_z,validation_points,prior_median_d2,ndt_median_d2,delta_ndt_minus_prior_m2,prior_support_points,ndt_support_points\n";

    const auto& frames = freeze.get_child("frames");
    auto source_it = sources.begin(); auto partition_it = partition_rows.begin();
    auto ndt_it = ndt_rows.begin(); auto split_it = split_rows.begin(); auto online_it = online_rows.begin();
    size_t frame_count = 0;
    for (const auto& frame_entry : frames) {
      const auto& frame = frame_entry.second;
      const int ordinal = frame.get<int>("raw_ordinal");
      const uint64_t stamp = frame.get<uint64_t>("stamp_ns");
      const Row& source_row = *source_it++; const Row& part_row = *partition_it++;
      const Row& ndt_row = *ndt_it++; const Row& split_row = *split_it++; const Row& online_row = *online_it++;
      require(std::stoi(field(source_row,"raw_ordinal")) == ordinal && std::stoull(field(source_row,"stamp_ns")) == stamp,
              "frozen source order/timestamp mismatch");
      require(file_sha(frame.get<std::string>("path")) == frame.get<std::string>("sha256"),
              "raw scan file SHA differs from frozen input identity");
      require(std::stoi(field(part_row,"raw_ordinal")) == ordinal && std::stoull(field(part_row,"stamp_ns")) == stamp,
              "partition support order/timestamp mismatch");
      require(std::stoull(field(ndt_row,"stamp_ns")) == stamp && std::stoull(field(split_row,"stamp_ns")) == stamp &&
              std::stoull(field(online_row,"stamp_ns")) == stamp, "sealed B scan timestamp mismatch");

      Cloud::Ptr source = load_voxel(frame.get<std::string>("path"), frame.get<size_t>("points"));
      require(source->size() == std::stoull(field(source_row,"source_points")) && xyz_sha(*source) == field(source_row,"source_xyz_sha256"),
              "R10 source preprocessing identity mismatch");
      require(source->size() == frame.get<size_t>("source_points") && xyz_sha(*source) == frame.get<std::string>("source_xyz_sha256"),
              "frozen per-frame source identity mismatch");
      auto partition = p63r14::split_cloud(*source);
      require(partition.counts.registration_points == std::stoull(field(split_row,"registration_points")) &&
              partition.counts.validation_points == std::stoull(field(split_row,"validation_points")) &&
              partition.counts.registration_blocks == std::stoull(field(split_row,"registration_blocks")) &&
              partition.counts.validation_blocks == std::stoull(field(split_row,"validation_blocks")),
              "runtime split support differs from deterministic offline split");
      require(xyz_sha(*partition.registration) == field(split_row,"registration_xyz_sha256") &&
              xyz_sha(*partition.validation) == field(split_row,"validation_xyz_sha256"),
              "runtime/offline split XYZ hash mismatch");

      const Eigen::Matrix4f prior = matrix_from_row(ndt_row, "guess_");
      const Eigen::Matrix4f ndt = matrix_from_row(ndt_row, "final_");
      std::map<p63r14::BlockKey, BlockResiduals> block_residuals;
      CandidateResiduals prior_totals, ndt_totals;
      std::vector<int> nearest_index(1); std::vector<float> squared_distance(1);
      for (const auto& point : source->points) {
        const auto block = p63r14::block_for(point.x, point.y, point.z);
        if ((p63r14::block_hash(block) & 1ULL) == 0ULL) continue;
        Point prior_point, ndt_point;
        const Eigen::Vector4f xyz(point.x, point.y, point.z, 1.0f);
        const Eigen::Vector4f prior_xyz = prior * xyz, ndt_xyz = ndt * xyz;
        prior_point.x=prior_xyz.x();prior_point.y=prior_xyz.y();prior_point.z=prior_xyz.z();prior_point.intensity=0.0f;
        ndt_point.x=ndt_xyz.x();ndt_point.y=ndt_xyz.y();ndt_point.z=ndt_xyz.z();ndt_point.intensity=0.0f;
        require(tree.nearestKSearch(prior_point,1,nearest_index,squared_distance)==1,"prior nearest-neighbor query failed");
        const double prior_d2=squared_distance[0];prior_totals.distances_squared.push_back(prior_d2);
        auto& block_data=block_residuals[block];block_data.prior.push_back(prior_d2);
        if(prior_d2<0.25){++prior_totals.supported_points;++block_data.prior_support;}
        require(tree.nearestKSearch(ndt_point,1,nearest_index,squared_distance)==1,"NDT nearest-neighbor query failed");
        const double ndt_d2=squared_distance[0];ndt_totals.distances_squared.push_back(ndt_d2);
        block_data.ndt.push_back(ndt_d2);
        if(ndt_d2<0.25){++ndt_totals.supported_points;++block_data.ndt_support;}
      }
      require(prior_totals.distances_squared.size()==partition.counts.validation_points &&
              ndt_totals.distances_squared.size()==partition.counts.validation_points,
              "validation point support count mismatch");

      std::vector<double> prior_block_scores, ndt_block_scores;
      uint64_t blocks_prefer_ndt=0, blocks_prefer_prior=0, blocks_tied=0;
      for(const auto& entry:block_residuals){
        const double prior_median=quantile(entry.second.prior,0.5), ndt_median=quantile(entry.second.ndt,0.5);
        const double delta=ndt_median-prior_median;
        prior_block_scores.push_back(prior_median);ndt_block_scores.push_back(ndt_median);
        if(delta<0)++blocks_prefer_ndt;else if(delta>0)++blocks_prefer_prior;else ++blocks_tied;
        block_output<<ordinal<<','<<stamp<<','<<entry.first.x<<','<<entry.first.y<<','<<entry.first.z<<','
                    <<entry.second.prior.size()<<','<<std::setprecision(12)<<prior_median<<','<<ndt_median<<','<<delta<<','
                    <<entry.second.prior_support<<','<<entry.second.ndt_support<<'\n';
        prior_totals.supported_blocks+=entry.second.prior_support>0;
        ndt_totals.supported_blocks+=entry.second.ndt_support>0;
      }
      std::string preference="AMBIGUOUS";
      if(block_residuals.size()>=2&&blocks_tied==0&&blocks_prefer_ndt==block_residuals.size())preference="PREFER_NDT";
      if(block_residuals.size()>=2&&blocks_tied==0&&blocks_prefer_prior==block_residuals.size())preference="PREFER_PRIOR";
      const auto mean=[](const std::vector<double>& v){return std::accumulate(v.begin(),v.end(),0.0)/v.size();};
      frame_output<<ordinal<<','<<stamp<<','<<partition.counts.validation_points<<','<<block_residuals.size()<<','
                  <<prior_totals.supported_points<<','<<ndt_totals.supported_points<<','<<prior_totals.supported_blocks<<','<<ndt_totals.supported_blocks<<','
                  <<std::setprecision(12)<<mean(prior_totals.distances_squared)<<','
                  <<quantile(prior_totals.distances_squared,.10)<<','<<quantile(prior_totals.distances_squared,.50)<<','
                  <<quantile(prior_totals.distances_squared,.90)<<','<<quantile(prior_totals.distances_squared,.95)<<','
                  <<*std::max_element(prior_totals.distances_squared.begin(),prior_totals.distances_squared.end())<<','
                  <<mean(ndt_totals.distances_squared)<<','<<quantile(ndt_totals.distances_squared,.10)<<','
                  <<quantile(ndt_totals.distances_squared,.50)<<','<<quantile(ndt_totals.distances_squared,.90)<<','
                  <<quantile(ndt_totals.distances_squared,.95)<<','<<*std::max_element(ndt_totals.distances_squared.begin(),ndt_totals.distances_squared.end())<<','
                  <<quantile(prior_block_scores,.50)<<','<<quantile(ndt_block_scores,.50)<<','
                  <<quantile(ndt_block_scores,.50)-quantile(prior_block_scores,.50)<<','
                  <<blocks_prefer_ndt<<','<<blocks_prefer_prior<<','<<blocks_tied<<','<<preference<<'\n';
      ++frame_count;
    }
    frame_output.flush();block_output.flush();
    require(frame_count==2375,"not all frozen frames scored");
    std::cout<<"PASS heldout shadow scores sealed inputs only; frames="<<frame_count<<" align=0 reference_access=0\n";
    return 0;
  } catch(const std::exception& error) {
    std::cerr<<"SHADOW_SCORE_BLOCKED "<<error.what()<<'\n';
    return 2;
  }
}
