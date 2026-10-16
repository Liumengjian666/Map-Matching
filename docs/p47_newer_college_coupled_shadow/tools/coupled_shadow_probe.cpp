#include "dog_prior_map_fastlio2_frontend_exp/observable_pcl_ndt.hpp"
#include "dog_prior_map_fastlio2_frontend_exp/coupled_ndt_anchor.hpp"
#include "dog_prior_map_fastlio2_frontend_exp/coupled_ndt_weak_refinement.hpp"

#include <openssl/evp.h>
#include <pcl/io/pcd_io.h>

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <map>
#include <memory>
#include <set>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

namespace
{
using namespace dog_prior_map_fastlio2_frontend_exp;
using Cloud = pcl::PointCloud<pcl::PointXYZ>;
using Ndt = ObservablePclNdt<pcl::PointXYZ>;
using Clock = std::chrono::steady_clock;

struct Table
{
  std::vector<std::string> header;
  std::map<std::string, std::size_t> column;
  std::vector<std::vector<std::string>> rows;
  std::string get(const std::vector<std::string> &row, const std::string &key) const
  {
    auto it = column.find(key);
    if (it == column.end() || it->second >= row.size())
      throw std::runtime_error("missing CSV field: " + key);
    return row[it->second];
  }
};

std::vector<std::string> splitCsv(const std::string &line)
{
  std::vector<std::string> fields;
  std::stringstream in(line);
  std::string field;
  while (std::getline(in, field, ',')) fields.push_back(field);
  return fields;
}

Table readCsv(const std::string &path)
{
  std::ifstream in(path.c_str());
  if (!in) throw std::runtime_error("cannot open CSV: " + path);
  Table table;
  std::string line;
  if (!std::getline(in, line)) throw std::runtime_error("empty CSV: " + path);
  table.header = splitCsv(line);
  for (std::size_t i = 0; i < table.header.size(); ++i) table.column.emplace(table.header[i], i);
  while (std::getline(in, line))
  {
    if (line.empty()) continue;
    auto row = splitCsv(line);
    if (row.size() != table.header.size()) throw std::runtime_error("malformed CSV: " + path);
    table.rows.push_back(std::move(row));
  }
  return table;
}

std::string sha256File(const std::string &path)
{
  std::ifstream input(path.c_str(), std::ios::binary);
  if (!input) throw std::runtime_error("cannot open for SHA256: " + path);
  EVP_MD_CTX *ctx = EVP_MD_CTX_new();
  if (!ctx) throw std::runtime_error("cannot allocate SHA256 context");
  bool ok = EVP_DigestInit_ex(ctx, EVP_sha256(), nullptr) == 1;
  std::array<char, 1 << 20> buffer{};
  while (ok && input.good())
  {
    input.read(buffer.data(), static_cast<std::streamsize>(buffer.size()));
    const auto count = input.gcount();
    if (count > 0 && EVP_DigestUpdate(ctx, buffer.data(), static_cast<std::size_t>(count)) != 1) ok = false;
  }
  unsigned char digest[EVP_MAX_MD_SIZE]{};
  unsigned int length = 0;
  if (ok) ok = EVP_DigestFinal_ex(ctx, digest, &length) == 1 && length == 32;
  EVP_MD_CTX_free(ctx);
  if (!ok) throw std::runtime_error("SHA256 failed: " + path);
  static const char hex[] = "0123456789abcdef";
  std::string output;
  output.reserve(64);
  for (unsigned int i = 0; i < length; ++i)
  { output.push_back(hex[digest[i] >> 4]); output.push_back(hex[digest[i] & 15]); }
  return output;
}

uint64_t stableCloudHash(const Cloud::ConstPtr &cloud)
{
  constexpr uint64_t kOffset = 1469598103934665603ULL;
  constexpr uint64_t kPrime = 1099511628211ULL;
  uint64_t hash = kOffset;
  auto byte = [&](uint8_t b) { hash ^= b; hash *= kPrime; };
  auto u32 = [&](uint32_t v) { for (int s = 0; s < 32; s += 8) byte(static_cast<uint8_t>((v >> s) & 255U)); };
  u32(cloud->width); u32(cloud->height); u32(cloud->is_dense ? 1U : 0U); u32(static_cast<uint32_t>(cloud->size()));
  for (const auto &p : cloud->points)
  {
    uint32_t bits = 0;
    std::memcpy(&bits, &p.x, sizeof(bits)); u32(bits);
    std::memcpy(&bits, &p.y, sizeof(bits)); u32(bits);
    std::memcpy(&bits, &p.z, sizeof(bits)); u32(bits);
  }
  return hash;
}

std::string hex64(uint64_t x)
{
  std::ostringstream s; s << std::hex << std::setw(16) << std::setfill('0') << x; return s.str();
}

struct Tile
{
  std::string path, sha256;
  std::size_t points = 0;
};

std::map<std::string, Tile> readTileCatalog(const Table &table)
{
  std::map<std::string, Tile> tiles;
  for (const auto &row : table.rows)
  {
    Tile t;
    t.path = table.get(row, "tile_path");
    t.sha256 = table.get(row, "sha256");
    t.points = static_cast<std::size_t>(std::stoull(table.get(row, "point_count")));
    const auto id = table.get(row, "tile_id");
    if (!tiles.emplace(id, t).second) throw std::runtime_error("duplicate tile catalog ID: " + id);
  }
  return tiles;
}

Cloud::Ptr loadTarget(const std::string &id_text, std::size_t expected_points,
                      const std::map<std::string, Tile> &catalog, std::string *target_hash)
{
  std::vector<std::string> ids;
  std::stringstream in(id_text);
  std::string id;
  while (std::getline(in, id, ';')) if (!id.empty()) ids.push_back(id);
  if (ids.empty()) throw std::runtime_error("P46 target has no active tile IDs");
  if (!std::is_sorted(ids.begin(), ids.end()) ||
      std::adjacent_find(ids.begin(), ids.end()) != ids.end())
    throw std::runtime_error("P46 active tile IDs are not a unique lexicographically sorted cache key");
  Cloud::Ptr target(new Cloud());
  for (const auto &tile_id : ids)
  {
    auto found = catalog.find(tile_id);
    if (found == catalog.end()) throw std::runtime_error("active P46 tile absent from catalog: " + tile_id);
    const Tile &tile = found->second;
    if (sha256File(tile.path) != tile.sha256)
      throw std::runtime_error("P45 tile SHA mismatch: " + tile_id);
    Cloud tile_cloud;
    if (pcl::io::loadPCDFile(tile.path, tile_cloud) != 0 || tile_cloud.size() != tile.points)
      throw std::runtime_error("P45 tile load/count mismatch: " + tile_id);
    *target += tile_cloud;
  }
  target->height = 1;
  target->width = static_cast<uint32_t>(target->size());
  target->is_dense = true;
  if (target->size() != expected_points)
    throw std::runtime_error("reassembled Local Target count differs from P46 diagnostics: expected=" +
                             std::to_string(expected_points) + " actual=" + std::to_string(target->size()));
  *target_hash = hex64(stableCloudHash(target));
  return target;
}

Eigen::Matrix4f poseFrom(const Table &table, const std::vector<std::string> &row,
                         const std::string &prefix)
{
  const Eigen::Vector3f p(static_cast<float>(std::stod(table.get(row, prefix + "_tx"))),
                          static_cast<float>(std::stod(table.get(row, prefix + "_ty"))),
                          static_cast<float>(std::stod(table.get(row, prefix + "_tz"))));
  Eigen::Quaternionf q(static_cast<float>(std::stod(table.get(row, prefix + "_qw"))),
                       static_cast<float>(std::stod(table.get(row, prefix + "_qx"))),
                       static_cast<float>(std::stod(table.get(row, prefix + "_qy"))),
                       static_cast<float>(std::stod(table.get(row, prefix + "_qz"))));
  if (!p.allFinite() || !q.coeffs().allFinite() || q.norm() < 1e-8f)
    throw std::runtime_error("invalid logged pose in sample manifest");
  q.normalize();
  Eigen::Matrix4f pose = Eigen::Matrix4f::Identity();
  pose.block<3, 3>(0, 0) = q.toRotationMatrix();
  pose.block<3, 1>(0, 3) = p;
  return pose;
}

bool validRigid(const Eigen::Matrix4f &T)
{
  if (!T.allFinite() || (T.row(3) - Eigen::RowVector4f(0, 0, 0, 1)).norm() > 1e-5f) return false;
  const Eigen::Matrix3f R = T.block<3, 3>(0, 0);
  return (R.transpose() * R - Eigen::Matrix3f::Identity()).norm() < 1e-3f &&
      std::abs(R.determinant() - 1.0f) < 1e-3f;
}

double translationDistance(const Eigen::Matrix4f &a, const Eigen::Matrix4f &b)
{ return (a.block<3, 1>(0, 3) - b.block<3, 1>(0, 3)).cast<double>().norm(); }

double rotationDistanceDeg(const Eigen::Matrix4f &a, const Eigen::Matrix4f &b)
{
  Eigen::Quaterniond qa(a.block<3, 3>(0, 0).cast<double>()), qb(b.block<3, 3>(0, 0).cast<double>());
  return qa.normalized().angularDistance(qb.normalized()) * 180.0 / std::acos(-1.0);
}

struct NominalReplayCheck
{
  double align_ms = 0.0;
  double cpu_ms = 0.0;
  double translation_error_m = 0.0;
  double rotation_error_deg = 0.0;
  double fitness_error = 0.0;
  int iterations = 0;
  double replay_fitness = 0.0;
  bool converged = false;
  bool parity_pass = false;
};

NominalReplayCheck primeNdtAtFrozenNominal(Ndt &ndt, const Cloud::Ptr &source,
    const Eigen::Matrix4f &initial, const Eigen::Matrix4f &frozen_nominal,
    int frozen_iterations, double frozen_fitness, const std::string &frame)
{
  if (!source || source->empty())
    throw std::runtime_error("cannot prime PCL derivative state with an empty source: frame " + frame);
  ndt.setInputSource(source);
  Cloud aligned;
  const auto wall_start = Clock::now();
  const std::clock_t cpu_start = std::clock();
  ndt.align(aligned, initial);
  NominalReplayCheck check;
  check.align_ms = std::chrono::duration<double, std::milli>(Clock::now() - wall_start).count();
  check.cpu_ms = 1000.0 * static_cast<double>(std::clock() - cpu_start) / CLOCKS_PER_SEC;
  check.iterations = ndt.getFinalNumIteration();
  check.converged = ndt.hasConverged();
  const Eigen::Matrix4f replay = ndt.getFinalTransformation();
  check.translation_error_m = translationDistance(replay, frozen_nominal);
  check.rotation_error_deg = rotationDistanceDeg(replay, frozen_nominal);
  check.replay_fitness = ndt.getFitnessScore();
  check.fitness_error = std::abs(check.replay_fitness - frozen_fitness);

  // Native PCL computeDerivatives() relies on state initialized by align().
  // A replay is only accepted as a derivative-state primer if it reproduces
  // the frozen P46 CONTROL nominal tightly enough not to substitute a new
  // reference pose or silently change the frame's objective evaluation.
  constexpr double kTranslationToleranceM = 1e-3;
  constexpr double kRotationToleranceDeg = 1e-2;
  constexpr double kFitnessTolerance = 1e-5;
  check.parity_pass = check.converged && replay.allFinite() && validRigid(replay) &&
      check.iterations == frozen_iterations && check.translation_error_m <= kTranslationToleranceM &&
      check.rotation_error_deg <= kRotationToleranceDeg && std::isfinite(check.replay_fitness) &&
      check.fitness_error <= kFitnessTolerance;
  return check;
}

std::size_t tileIdCount(const std::string &ids)
{
  if (ids.empty()) return 0;
  return static_cast<std::size_t>(1 + std::count(ids.begin(), ids.end(), ';'));
}

std::string vectorText(const Eigen::VectorXd &value)
{
  std::ostringstream out;
  out << std::setprecision(17);
  for (Eigen::Index i = 0; i < value.size(); ++i)
  {
    if (i) out << ';';
    out << value(i);
  }
  return out.str();
}

}  // namespace

int main(int argc, char **argv)
{
  if (argc != 10)
  {
    std::cerr << "usage: p47_coupled_shadow_probe SAMPLE_MANIFEST SOURCE_PARITY TILE_CATALOG "
                 "SOURCE_DIR DEGENERACY_CSV PAIRED_CSV CANDIDATE_CSV RUNTIME_CSV NOMINAL_PARITY_CSV\n";
    return 2;
  }
  try
  {
    for (int i = 5; i <= 9; ++i)
      if (std::ifstream(argv[i])) throw std::runtime_error("refusing to overwrite output CSV: " + std::string(argv[i]));
    const Table manifest = readCsv(argv[1]);
    const Table parity = readCsv(argv[2]);
    const Table catalog_table = readCsv(argv[3]);
    const auto tiles = readTileCatalog(catalog_table);
    if (manifest.rows.empty() || parity.rows.size() != manifest.rows.size())
      throw std::runtime_error("source parity row count differs from frozen sample set");
    for (std::size_t i = 0; i < parity.rows.size(); ++i)
      if (parity.get(parity.rows[i], "parity") != "PASS" ||
          parity.get(parity.rows[i], "frame_index") != manifest.get(manifest.rows[i], "frame_index"))
        throw std::runtime_error("P46 source hash parity not passed for every fixed sample");

    std::ofstream degeneracy(argv[5]), paired(argv[6]), candidate(argv[7]), runtime(argv[8]);
    std::ofstream nominal_parity(argv[9]);
    if (!degeneracy || !paired || !candidate || !runtime || !nominal_parity)
      throw std::runtime_error("cannot create P47 output CSV");
    degeneracy << "frame_index,scan_ns,source_hash,target_tile_ids,target_points,target_hash,nominal_fitness,ndt_iterations,"
                  "initial_to_nominal_translation_m,initial_to_nominal_rotation_deg,jet_valid,score_sum,mean_energy,"
                  "gradient_norm,eigenvalues,weak_dimension,lambda_min_max_ratio,lambda1_lambda0_ratio,"
                  "W00,W10,W20,W30,W40,W50,W01,W11,W21,W31,W41,W51,jet_ms,jet_calls,nominal_tx,nominal_ty,nominal_tz,nominal_qx,nominal_qy,nominal_qz,nominal_qw\n";
    paired << "frame_index,scan_ns,source_hash,target_tile_ids,target_points,target_hash,anchor_kind,triggered,"
              "weak_dimension,nominal_score_sum,weak_score_sum,coupled_score_sum,nominal_objective,weak_objective,"
              "coupled_objective,weak_eta,delta_u,strong_eta,delta_v,nominal_cross_norm,strong_cross_norm,"
              "strong_gradient_norm,strong_selected,weak_candidate_valid,coupled_candidate_valid,"
              "solver_status,solver_condition,solver_residual,strong_damping,fallback_reason,"
              "weak_candidate_tx,weak_candidate_ty,weak_candidate_tz,weak_candidate_qx,weak_candidate_qy,weak_candidate_qz,weak_candidate_qw,"
              "coupled_candidate_tx,coupled_candidate_ty,coupled_candidate_tz,coupled_candidate_qx,coupled_candidate_qy,coupled_candidate_qz,coupled_candidate_qw,"
              "weak_only_vs_nominal_score,coupled_vs_weak_score,coupled_vs_nominal_score,weak_status,coupled_status\n";
    candidate << "frame_index,scan_ns,arm,status,triggered,attempted,recommended,weak_dimension,"
                 "innovation_translation_m,innovation_rotation_deg,anchor_age_s,weak_eta,delta_u,strong_eta,delta_v,"
                 "nominal_cross_norm,displaced_cross_norm,nominal_strong_gradient,displaced_strong_gradient,"
                 "strong_selected,solver_status,solver_condition,solver_residual,strong_damping,"
                 "candidate_score,candidate_objective,candidate_finite,candidate_rigid,"
                 "candidate_tx,candidate_ty,candidate_tz,candidate_qx,candidate_qy,candidate_qz,candidate_qw\n";
    runtime << "frame_index,scan_ns,stage,jet_calls,value_calls,align_calls,total_ms,cpu_ms,jet_ms,value_ms,solve_ms,align_ms,"
               "source_hash,target_hash\n";
    nominal_parity << "frame_index,scan_ns,stage,converged,frozen_iterations,replay_iterations,frozen_fitness,replay_fitness,"
                      "translation_error_m,rotation_error_deg,fitness_error,translation_tolerance_m,"
                      "rotation_tolerance_deg,fitness_tolerance,pass\n";
    degeneracy << std::setprecision(17);
    paired << std::setprecision(17);
    runtime << std::setprecision(17);

    std::vector<std::size_t> pair_eligible;
    std::size_t total_align_calls = 0;
    std::string active_ids;
    Cloud::Ptr active_target;
    std::string active_target_hash;
    std::unique_ptr<Ndt> ndt;
    CoupledNdtBackend backend;
    for (std::size_t i = 0; i < manifest.rows.size(); ++i)
    {
      const auto &row = manifest.rows[i];
      const std::string frame = manifest.get(row, "frame_index");
      const std::string source_hash = parity.get(parity.rows[i], "reconstructed_hash");
      const std::string ids = manifest.get(row, "active_tile_ids");
      const std::size_t target_points = static_cast<std::size_t>(std::stoull(manifest.get(row, "target_points")));
      if (!ndt || ids != active_ids)
      {
        active_target = loadTarget(ids, target_points, tiles, &active_target_hash);
        ndt.reset(new Ndt());
        ndt->setResolution(0.8f);
        ndt->setStepSize(0.08);
        ndt->setTransformationEpsilon(0.001);
        ndt->setMaximumIterations(40);
        ndt->setOulierRatio(0.55);
        ndt->setInputTarget(active_target);
        active_ids = ids;
        std::cout << "P47_LOCAL_TARGET_REBUILT tile_count=" << tileIdCount(ids)
                  << " points=" << active_target->size() << " hash=" << active_target_hash << '\n';
      }
      Cloud::Ptr source(new Cloud());
      const std::string source_path = std::string(argv[4]) + "/source_frame_" + frame + ".pcd";
      if (pcl::io::loadPCDFile(source_path, *source) != 0 || source->size() != 1400 ||
          hex64(stableCloudHash(source)) != source_hash)
        throw std::runtime_error("saved source PCD does not preserve parity/hash: frame " + frame);

      const Eigen::Matrix4f nominal = poseFrom(manifest, row, "nominal");
      const Eigen::Matrix4f prediction = poseFrom(manifest, row, "initial");
      if (!validRigid(nominal) || !validRigid(prediction))
        throw std::runtime_error("P46 nominal or initial pose is not rigid at frame " + frame);
      const auto prime = primeNdtAtFrozenNominal(*ndt, source, prediction, nominal,
          std::stoi(manifest.get(row, "ndt_iterations")),
          std::stod(manifest.get(row, "ndt_fitness")), frame);
      ++total_align_calls;
      nominal_parity << frame << ',' << manifest.get(row, "scan_ns") << ",hessian," << prime.converged << ','
          << manifest.get(row, "ndt_iterations") << ',' << prime.iterations << ','
          << manifest.get(row, "ndt_fitness") << ',' << prime.replay_fitness << ','
          << prime.translation_error_m << ',' << prime.rotation_error_deg << ',' << prime.fitness_error
          << ",0.001,0.01,1e-5," << prime.parity_pass << '\n';
      nominal_parity.flush();
      runtime << frame << ',' << manifest.get(row, "scan_ns") << ",p46_nominal_parity_align,0,0,1,"
              << prime.align_ms << ',' << prime.cpu_ms << ",0,0,0," << prime.align_ms << ','
              << source_hash << ',' << active_target_hash << '\n';
      runtime.flush();
      if (!prime.parity_pass)
      {
        std::ostringstream error;
        error << std::setprecision(12) << "P46 nominal replay parity failed at frame " << frame
              << ": converged=" << prime.converged << " iterations=" << prime.iterations
              << '/' << manifest.get(row, "ndt_iterations") << " translation_error_m="
              << prime.translation_error_m << " rotation_error_deg=" << prime.rotation_error_deg
              << " fitness_error=" << prime.fitness_error;
        throw std::runtime_error(error.str());
      }
      std::size_t frame_jet_calls = 0;
      double frame_native_jet_ms = 0.0;
      backend.jet = [&](const Eigen::Matrix4f &pose, const CoupledVector6 &params) {
        ++frame_jet_calls;
        const auto start = Clock::now();
        const CoupledNativeJet value = ndt->nativeJet(source, pose, params);
        frame_native_jet_ms += std::chrono::duration<double, std::milli>(Clock::now() - start).count();
        return value;
      };
      backend.score = [&](const Eigen::Matrix4f &pose) { return ndt->dynamicScore(source, pose); };

      const auto jet_start = Clock::now();
      const CoupledLocalJet model = coupledNominalJet(nominal, static_cast<double>(source->size()), backend, 0.8);
      const double total_jet_ms = std::chrono::duration<double, std::milli>(Clock::now() - jet_start).count();
      Eigen::SelfAdjointEigenSolver<CoupledMatrix6> eig;
      bool jet_valid = model.valid;
      if (jet_valid) { eig.compute(model.H); jet_valid = eig.info() == Eigen::Success && eig.eigenvalues().allFinite() && eig.eigenvectors().allFinite(); }
      const int weak_dimension = jet_valid && eig.eigenvalues().minCoeff() > 0.0 ?
          (eig.eigenvalues()(1) / eig.eigenvalues()(0) >= 2.0 ? 1 : 2) : 0;
      const double ratio = jet_valid && eig.eigenvalues()(5) > 0.0 ? eig.eigenvalues()(0) / eig.eigenvalues()(5) : NAN;
      const double split_ratio = weak_dimension > 0 ? eig.eigenvalues()(1) / eig.eigenvalues()(0) : NAN;
      const double initial_delta_t = translationDistance(nominal, prediction);
      const double initial_delta_r = rotationDistanceDeg(nominal, prediction);
      const std::string stamp = manifest.get(row, "scan_ns");
      const double mean_energy = model.valid ? -model.score / static_cast<double>(source->size()) : NAN;
      std::ostringstream eigen_text, W_text;
      if (jet_valid)
      {
        for (int j = 0; j < 6; ++j) { if (j) eigen_text << ';'; eigen_text << eig.eigenvalues()(j); }
        for (int c = 0; c < 2; ++c) for (int r = 0; r < 6; ++r)
        { if (c || r) W_text << ';'; W_text << eig.eigenvectors()(r, c); }
      }
      const Eigen::Quaternionf qn(nominal.block<3, 3>(0, 0));
      degeneracy << frame << ',' << stamp << ',' << source_hash << ',' << ids << ',' << target_points << ','
          << active_target_hash << ',' << manifest.get(row, "ndt_fitness") << ',' << manifest.get(row, "ndt_iterations") << ','
          << initial_delta_t << ',' << initial_delta_r << ',' << (jet_valid ? 1 : 0) << ','
          << (model.valid ? model.score : NAN) << ',' << mean_energy << ',' << (model.valid ? model.gradient.norm() : NAN) << ','
          << '"' << eigen_text.str() << '"' << ',' << weak_dimension << ',' << ratio << ',' << split_ratio;
      if (jet_valid)
        for (int c = 0; c < 2; ++c) for (int r = 0; r < 6; ++r) degeneracy << ',' << eig.eigenvectors()(r, c);
      else for (int j = 0; j < 12; ++j) degeneracy << ",nan";
      degeneracy << ',' << total_jet_ms << ',' << frame_jet_calls << ',' << nominal(0,3) << ',' << nominal(1,3) << ',' << nominal(2,3) << ','
          << qn.x() << ',' << qn.y() << ',' << qn.z() << ',' << qn.w() << '\n';
      runtime << frame << ',' << stamp << ",hessian," << frame_jet_calls << ",0,0," << total_jet_ms << ','
              << frame_native_jet_ms << ',' << frame_native_jet_ms << ",0,0,0,"
              << source_hash << ',' << active_target_hash << '\n';
      degeneracy.flush();
      runtime.flush();

      const bool triggered = initial_delta_t > 0.12 || initial_delta_r > 3.0;
      if (jet_valid && weak_dimension > 0 && triggered) pair_eligible.push_back(i);
    }

    std::vector<std::size_t> pairs;
    const std::size_t count = std::min<std::size_t>(20, pair_eligible.size());
    if (count == 1) pairs.push_back(pair_eligible.front());
    else if (count > 1)
      for (std::size_t i = 0; i < count; ++i)
      {
        const std::size_t position = static_cast<std::size_t>(std::llround(
            static_cast<double>(i) * static_cast<double>(pair_eligible.size() - 1) / static_cast<double>(count - 1)));
        if (pairs.empty() || pairs.back() != pair_eligible[position]) pairs.push_back(pair_eligible[position]);
      }
    std::set<std::size_t> pair_set(pairs.begin(), pairs.end());

    const auto write_candidate = [&](const std::string &frame, const std::string &stamp,
                                     const std::string &arm, const WeakCoupledResult &result) {
      const Eigen::Matrix4f candidate_pose = result.recommended ? result.candidate : result.nominal;
      const double candidate_score = result.recommended ? result.candidate_score : result.nominal_score;
      const double candidate_objective = result.recommended ? result.candidate_objective : result.nominal_objective;
      const bool finite = candidate_pose.allFinite();
      const bool rigid = finite && validRigid(candidate_pose);
      Eigen::Quaternionf q = Eigen::Quaternionf::Identity();
      if (rigid) { q = Eigen::Quaternionf(candidate_pose.block<3, 3>(0, 0)); q.normalize(); }
      candidate << frame << ',' << stamp << ',' << arm << ',' << result.status << ','
          << (result.triggered ? 1 : 0) << ',' << (result.attempted ? 1 : 0) << ','
          << (result.recommended ? 1 : 0) << ',' << result.weak_dimension << ','
          << result.innovation_translation_m << ',' << result.innovation_rotation_deg << ','
          << result.anchor_age_s << ',' << '"' << vectorText(result.weak_eta) << '"' << ','
          << '"' << vectorText(result.delta_u) << '"' << ','
          << '"' << vectorText(result.strong_eta) << '"' << ','
          << '"' << vectorText(result.delta_v) << '"' << ','
          << result.nominal_cross_norm << ',' << result.displaced_cross_norm << ','
          << result.nominal_strong_gradient << ',' << result.displaced_strong_gradient << ','
          << (result.strong_selected ? 1 : 0) << ',' << result.strong_status << ','
          << result.strong_condition << ',' << result.strong_solve_residual << ','
          << result.strong_damping << ',' << candidate_score << ','
          << candidate_objective << ',' << (finite ? 1 : 0) << ',' << (rigid ? 1 : 0) << ',';
      if (rigid)
        candidate << candidate_pose(0,3) << ',' << candidate_pose(1,3) << ',' << candidate_pose(2,3) << ','
                  << q.x() << ',' << q.y() << ',' << q.z() << ',' << q.w() << '\n';
      else candidate << "nan,nan,nan,nan,nan,nan,nan\n";
    };

    // The paired shadow reuses the existing R6 local refinement kernel. P46 CONTROL
    // did not request the production EKF-anchor stream, so its logged causal
    // CONTROL initial_guess is frozen as a zero-age diagnostic reference only.
    active_ids.clear(); ndt.reset(); active_target.reset();
    for (const std::size_t i : pairs)
    {
      const auto &row = manifest.rows[i];
      const std::string frame = manifest.get(row, "frame_index");
      const std::string stamp_text = manifest.get(row, "scan_ns");
      const uint64_t stamp_ns = std::stoull(stamp_text);
      const std::string ids = manifest.get(row, "active_tile_ids");
      const std::size_t target_points = static_cast<std::size_t>(std::stoull(manifest.get(row, "target_points")));
      const std::string source_hash = parity.get(parity.rows[i], "reconstructed_hash");
      if (!ndt || ids != active_ids)
      {
        active_target = loadTarget(ids, target_points, tiles, &active_target_hash);
        ndt.reset(new Ndt());
        ndt->setResolution(0.8f); ndt->setStepSize(0.08);
        ndt->setTransformationEpsilon(0.001); ndt->setMaximumIterations(40);
        ndt->setOulierRatio(0.55); ndt->setInputTarget(active_target); active_ids = ids;
      }
      Cloud::Ptr source(new Cloud());
      const std::string source_path = std::string(argv[4]) + "/source_frame_" + frame + ".pcd";
      if (pcl::io::loadPCDFile(source_path, *source) != 0 || source->size() != 1400 ||
          hex64(stableCloudHash(source)) != source_hash)
        throw std::runtime_error("paired source hash invalid at frame " + frame);
      const Eigen::Matrix4f nominal = poseFrom(manifest, row, "nominal");
      const Eigen::Matrix4f prediction = poseFrom(manifest, row, "initial");
      const auto prime = primeNdtAtFrozenNominal(*ndt, source, prediction, nominal,
          std::stoi(manifest.get(row, "ndt_iterations")),
          std::stod(manifest.get(row, "ndt_fitness")), frame);
      ++total_align_calls;
      nominal_parity << frame << ',' << stamp_text << ",paired," << prime.converged << ','
          << manifest.get(row, "ndt_iterations") << ',' << prime.iterations << ','
          << manifest.get(row, "ndt_fitness") << ',' << prime.replay_fitness << ','
          << prime.translation_error_m << ',' << prime.rotation_error_deg << ',' << prime.fitness_error
          << ",0.001,0.01,1e-5," << prime.parity_pass << '\n';
      nominal_parity.flush();
      runtime << frame << ',' << stamp_text << ",p46_nominal_parity_align,0,0,1,"
              << prime.align_ms << ',' << prime.cpu_ms << ",0,0,0," << prime.align_ms << ','
              << source_hash << ',' << active_target_hash << '\n';
      runtime.flush();
      if (!prime.parity_pass)
      {
        std::ostringstream error;
        error << std::setprecision(12) << "P46 nominal replay parity failed at paired frame " << frame
              << ": converged=" << prime.converged << " iterations=" << prime.iterations
              << '/' << manifest.get(row, "ndt_iterations") << " translation_error_m="
              << prime.translation_error_m << " rotation_error_deg=" << prime.rotation_error_deg
              << " fitness_error=" << prime.fitness_error;
        throw std::runtime_error(error.str());
      }
      backend.jet = [&](const Eigen::Matrix4f &pose, const CoupledVector6 &params) {
        return ndt->nativeJet(source, pose, params);
      };
      backend.score = [&](const Eigen::Matrix4f &pose) { return ndt->dynamicScore(source, pose); };

      CoupledAnchorState anchor;
      anchor.origin = prediction.cast<double>();
      anchor.prediction = prediction.cast<double>();
      anchor.origin_stamp_ns = stamp_ns;
      anchor.propagated_stamp_ns = stamp_ns;
      anchor.valid = true;
      anchor.frozen = false;
      anchor.status = "P46_CONTROL_INITIAL_GUESS_SHADOW_REFERENCE_NOT_PRODUCTION_EKF_ANCHOR";
      WeakCoupledConfig weak_config;
      weak_config.coupled = false;
      WeakCoupledConfig coupled_config;
      coupled_config.coupled = true;
      const auto weak = runWeakCoupledRefinement(nominal, prediction, stamp_ns,
          source->size(), true, anchor, backend, weak_config);
      const auto coupled = runWeakCoupledRefinement(nominal, prediction, stamp_ns,
          source->size(), true, anchor, backend, coupled_config);
      const bool weak_valid = validRigid(weak.candidate) && weak.candidate.allFinite();
      const bool coupled_valid = validRigid(coupled.candidate) && coupled.candidate.allFinite();
      auto pose_fields = [&](const Eigen::Matrix4f &T) {
        Eigen::Quaternionf q(T.block<3, 3>(0, 0)); q.normalize();
        std::ostringstream out; out << T(0,3) << ',' << T(1,3) << ',' << T(2,3) << ','
            << q.x() << ',' << q.y() << ',' << q.z() << ',' << q.w(); return out.str();
      };
      const double weak_score = weak.recommended ? weak.candidate_score : weak.nominal_score;
      const double coupled_score = coupled.recommended ? coupled.candidate_score : coupled.nominal_score;
      const double weak_objective = weak.recommended ? weak.candidate_objective : weak.nominal_objective;
      const double coupled_objective = coupled.recommended ? coupled.candidate_objective : coupled.nominal_objective;
      paired << frame << ',' << stamp_text << ',' << source_hash << ',' << ids << ',' << target_points << ','
          << active_target_hash << ",P46_CONTROL_INITIAL_GUESS_DIAGNOSTIC_REFERENCE," << (coupled.triggered ? 1 : 0) << ','
          << coupled.weak_dimension << ',' << coupled.nominal_score << ',' << weak_score << ',' << coupled_score << ','
          << weak.nominal_objective << ',' << weak_objective << ',' << coupled_objective << ','
          << '"' << coupled.weak_eta.transpose() << '"' << ',' << '"' << coupled.delta_u.transpose() << '"' << ','
          << '"' << coupled.strong_eta.transpose() << '"' << ',' << '"' << coupled.delta_v.transpose() << '"' << ','
          << coupled.nominal_cross_norm << ',' << coupled.displaced_cross_norm << ','
          << coupled.displaced_strong_gradient << ',' << (coupled.strong_selected ? 1 : 0) << ','
          << (weak_valid ? 1 : 0) << ',' << (coupled_valid ? 1 : 0) << ','
          << coupled.strong_status << ',' << coupled.strong_condition << ',' << coupled.strong_solve_residual << ','
          << coupled.strong_damping << ',' << coupled.status << ':' << coupled.strong_status << ','
          << pose_fields(weak.candidate) << ',' << pose_fields(coupled.candidate) << ','
          << (weak_score - weak.nominal_score) << ',' << (coupled_score - weak_score) << ','
          << (coupled_score - weak.nominal_score) << ',' << weak.status << ',' << coupled.status << '\n';
      write_candidate(frame, stamp_text, "weak_only", weak);
      write_candidate(frame, stamp_text, "coupled", coupled);
      runtime << frame << ',' << stamp_text << ",weak_only," << weak.jet_calls << ',' << weak.value_calls
          << ",0," << weak.total_ms << ',' << weak.cpu_ms << ',' << weak.jet_ms << ',' << weak.value_ms << ','
          << weak.solve_ms << ",0," << source_hash << ',' << active_target_hash << '\n';
      runtime << frame << ',' << stamp_text << ",coupled," << coupled.jet_calls << ',' << coupled.value_calls
          << ",0," << coupled.total_ms << ',' << coupled.cpu_ms << ',' << coupled.jet_ms << ',' << coupled.value_ms << ','
          << coupled.solve_ms << ",0," << source_hash << ',' << active_target_hash << '\n';
      paired.flush();
      candidate.flush();
      runtime.flush();
      std::cout << "P47_SHADOW frame=" << frame << " status=" << coupled.status
                << " dim=" << coupled.weak_dimension << " strong_selected=" << coupled.strong_selected
                << " dv=" << coupled.strong_eta.norm() << '\n';
    }
    std::cout << "P47_PROBE_COMPLETE samples=" << manifest.rows.size()
              << " pair_eligible=" << pair_eligible.size() << " paired=" << pairs.size()
              << " candidate_rows=" << 2 * pairs.size()
              << " nominal_parity_aligns=" << total_align_calls << " gt_read=false\n";
    return 0;
  }
  catch (const std::exception &e)
  {
    std::cerr << "P47_SHADOW_PROBE_FAILED: " << e.what() << '\n';
    return 1;
  }
}
