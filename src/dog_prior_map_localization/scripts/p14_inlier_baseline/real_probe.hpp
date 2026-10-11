#pragma once
#include "probe_io.hpp"
#include <inlier_core/matcher.hpp>
#include <inlier_core/verify.hpp>
#include <ctime>
#include <iostream>
#include <sstream>

namespace p14 {
struct Input { int64_t id; std::string stamp, path; };
inline std::vector<Input> readInputs(const std::string& path) {
  std::ifstream f(path);
  if (!f) throw std::runtime_error("Missing frozen input list: " + path);
  std::string line;
  std::getline(f, line);
  if (line != "id,stamp,path,sha256") throw std::runtime_error("Input list schema mismatch");
  std::vector<Input> inputs;
  std::set<int64_t> ids;
  while (std::getline(f, line)) {
    std::istringstream row(line);
    std::string id, stamp, file, hash;
    std::getline(row, id, ','); std::getline(row, stamp, ',');
    std::getline(row, file, ','); std::getline(row, hash);
    Input input{std::stoll(id), stamp, file};
    if (file.empty() || stamp.empty() || hash.size() != 64 || !ids.insert(input.id).second)
      throw std::runtime_error("Invalid frozen input row");
    inputs.push_back(input);
  }
  return inputs;
}
struct Encoded { inlier::Keypoints kp; inlier::Tokens tokens; };
struct Frame {
  int raw = 0, finite = 0, zeros = 0, prepared = 0, above_ground = 0;
  double load_ms = 0, prep_ms = 0, encode_ms = 0;
  double raw_z_min = 0, raw_z_max = 0, aligned_z_min = 0, aligned_z_max = 0;
  Encoded encoded;
};
inline Frame encode(const std::string& file, inlier::Encoder& encoder) {
  Frame f;
  auto tick = Clock::now();
  Points raw = load(file);
  f.load_ms = ms(tick);
  f.raw = static_cast<int>(raw.rows());
  f.raw_z_min = std::numeric_limits<double>::infinity();
  f.raw_z_max = -std::numeric_limits<double>::infinity();
  for (Eigen::Index i = 0; i < raw.rows(); ++i) {
    if (!raw.row(i).allFinite()) continue;
    ++f.finite;
    if (raw.row(i).isZero(0)) ++f.zeros;
    f.raw_z_min = std::min(f.raw_z_min, raw(i, 2));
    f.raw_z_max = std::max(f.raw_z_max, raw(i, 2));
  }
  tick = Clock::now();
  Points points = preprocess(raw);
  f.prep_ms = ms(tick);
  f.prepared = static_cast<int>(points.rows());
  if (points.rows() >= 10) {  // author's encode_frame empty-input policy
    tick = Clock::now();
    auto pair = encoder.Encode(points);
    f.encode_ms = ms(tick);
    f.encoded = {std::move(pair.first), std::move(pair.second)};
  } else f.encoded.kp.p.resize(0, 3);
  validate(f.encoded.kp, f.encoded.tokens);
  f.aligned_z_min = std::numeric_limits<double>::infinity();
  f.aligned_z_max = -std::numeric_limits<double>::infinity();
  for (Eigen::Index i = 0; i < points.rows(); ++i) {
    double z = (f.encoded.kp.T_ground.topLeftCorner<3,3>() * points.row(i).transpose()
                + f.encoded.kp.T_ground.topRightCorner<3,1>()).z();
    f.aligned_z_min = std::min(f.aligned_z_min, z);
    f.aligned_z_max = std::max(f.aligned_z_max, z);
    f.above_ground += (z >= 0 && z < 20);
  }
  return f;
}
inline void frameFields(std::ostream& out, const Frame& f) {
  out << f.raw << ',' << f.finite << ',' << f.zeros << ',' << f.prepared << ','
      << f.encoded.kp.p.rows() << ',' << f.encoded.tokens.token_id.size() << ','
      << activeRows(f.encoded.tokens) << ',' << f.raw_z_min << ',' << f.raw_z_max << ','
      << f.aligned_z_min << ',' << f.aligned_z_max << ',' << f.above_ground << ','
      << f.encoded.kp.T_ground(2, 2) << ',' << f.encoded.kp.T_ground(2, 3) << ','
      << f.load_ms << ',' << f.prep_ms << ',' << f.encode_ms;
}
inline std::string frameHeader() {
  return "raw_points,finite_points,zero_points,prepared_points,keypoints,tokens,active_height_rows,"
         "raw_z_min,raw_z_max,aligned_z_min,aligned_z_max,above_ground_points,ground_Rzz,ground_tz,load_ms,preprocess_ms,encode_ms";
}
inline int positive(const std::vector<double>& scores) {
  int n = 0;
  for (double s : scores) {
    if (!std::isfinite(s)) throw std::runtime_error("Nonfinite retrieval score");
    n += (s > 0);
  }
  return n;
}
inline void real(const std::string& directory) {
  // The caller freezes/hashes lists before launch; refuse to overwrite runs.
  std::ifstream existing(directory + "/queries_result.csv");
  if (existing.good()) throw std::runtime_error("Real experiment already exists; no selective rerun");
  const auto refs = readInputs(directory + "/references.csv");
  const auto queries = readInputs(directory + "/queries.csv");
  if (refs.size() != 389 || queries.size() != 24) throw std::runtime_error("Frozen input counts mismatch");
  auto reference_csv = output(directory + "/reference_encoding.csv");
  reference_csv << "id,stamp," << frameHeader() << '\n';
  auto cache = output(directory + "/reference_tokens_keypoints.bin");
  cache.write("P14DB001", 8);
  const uint64_t n_refs = refs.size();
  cache.write(reinterpret_cast<const char*>(&n_refs), 8);
  inlier::InLiERConfig grid;
  inlier::Encoder encoder(grid);
  std::vector<Encoded> database;
  database.reserve(refs.size());
  const auto db_tick = Clock::now();
  const long before_database = rssKiB();
  const long before_database_peak = peakRssKiB();
  size_t tokens_total = 0;
  for (const auto& input : refs) {
    if (input.id != static_cast<int64_t>(database.size()))
      throw std::runtime_error("Reference IDs must be frozen sequential indices");
    Frame frame = encode(input.path, encoder);
    reference_csv << input.id << ',' << input.stamp << ',';
    frameFields(reference_csv, frame); reference_csv << '\n';
    tokens_total += frame.encoded.tokens.token_id.size();
    uint64_t count = frame.encoded.tokens.token_id.size();
    cache.write(reinterpret_cast<const char*>(&input.id), 8);
    cache.write(reinterpret_cast<const char*>(&count), 8);
    cache.write(reinterpret_cast<const char*>(frame.encoded.kp.T_ground.data()), 128);
    writePoints(cache, frame.encoded.kp.p);
    cache.write(reinterpret_cast<const char*>(frame.encoded.tokens.token_id.data()), count * 8);
    database.push_back(std::move(frame.encoded));
    if (database.size() % 50 == 0) std::cout << "REFERENCE_ENCODED " << database.size() << std::endl;
  }
  cache.close(); reference_csv.close();
  const double database_ms = ms(db_tick);
  const long before_matcher = rssKiB();
  const long database_peak = peakRssKiB();
  auto finalize_tick = Clock::now();
  inlier::Matcher matcher(grid);
  matcher.Reserve(refs.size());
  for (size_t i = 0; i < refs.size(); ++i) matcher.Add(refs[i].id, database[i].tokens.token_id);
  matcher.Finalize();
  const double index_ms = ms(finalize_tick);
  const long after_matcher = rssKiB();
  auto resources = output(directory + "/resources.csv");
  resources << "stage,wall_ms,rss_kib,peak_rss_kib,tokens_total\n";
  resources << "BEFORE_DATABASE,0," << before_database << ',' << before_database_peak << ",0\n";
  resources << "REFERENCE_DB," << database_ms << ',' << before_matcher << ',' << database_peak << ',' << tokens_total << '\n';
  resources << "MATCHER_INDEX," << index_ms << ',' << after_matcher << ',' << peakRssKiB() << ',' << tokens_total << '\n';

  auto qout = output(directory + "/queries_result.csv");
  qout << "id,stamp," << frameHeader() << ",mint_count,mint_positive,beam_count,beam_positive,verify_calls,verify_success,legal_relative_poses,"
          "mint_ms,beam_ms,verify_ms,compute_ms,end_to_end_ms,cpu_ms,peak_rss_kib\n";
  auto candidates = output(directory + "/candidates.csv");
  candidates << "query_id,rank,reference_id,mint_score,beam_score,yaw_radians,azimuth_shift,verify_success,fail_stage,correspondences,"
                "ransac_inliers_found,keypoint_inliers,inlier_rmse,rigid_success_pose,verify_ms";
  for (int i = 0; i < 16; ++i) candidates << ",T" << i;
  candidates << '\n';
  auto mintout = output(directory + "/mint_candidates.csv");
  mintout << "query_id,rank,reference_id,score\n";
  int usable_six = 0, executed = 0;
  for (const auto& input : queries) {
    if (executed == 6 && usable_six == 0) {
      std::cout << "STOP_NO_USABLE_CANDIDATE_IN_FIRST_SIX" << std::endl;
      break;
    }
    auto query_tick = Clock::now();
    const auto cpu_tick = std::clock();
    Frame frame = encode(input.path, encoder);
    const auto& q = frame.encoded;
    auto tick = Clock::now();
    auto mint = matcher.Shortlist(q.tokens.token_id, mintConfig(), -1, -1);
    double mint_ms = ms(tick);
    tick = Clock::now();
    auto beam = matcher.BeamScore(q.tokens.token_id, mint.ids, beamConfig(), -1, -1);
    double beam_ms = ms(tick);
    const int mp = positive(mint.scores), bp = positive(beam.scores);
    if (executed < 6 && !q.tokens.token_id.empty() && mp > 0 && bp > 0) ++usable_six;
    std::vector<inlier::VerifyResult> verified;
    std::vector<double> times;
    double verify_ms = 0;
    int success = 0, legal = 0;
    for (size_t i = 0; i < beam.ids.size(); ++i) {
      // IDs are independently frozen sequential reference indices. Author
      // relative Verify uses sensor-frame keypoints, never mapping-state poses.
      const auto& db = database.at(static_cast<size_t>(beam.ids[i]));
      tick = Clock::now();
      auto v = inlier::Verify(q.tokens.token_id, q.kp, db.tokens.token_id, db.kp,
                             beam.best_shifts[i], grid, verifyConfig());
      double t = ms(tick); verify_ms += t;
      success += v.success;
      const bool valid = v.success && rigid(v.T_sensor) && std::isfinite(v.inlier_rmse);
      legal += valid;
      if (v.success && !valid) throw std::runtime_error("Author Verify returned invalid successful pose");
      verified.push_back(v); times.push_back(t);
    }
    const double elapsed_ms = ms(query_tick);
    const double cpu_ms = 1000.0 * (std::clock() - cpu_tick) / CLOCKS_PER_SEC;
    const double compute_ms = frame.prep_ms + frame.encode_ms + mint_ms + beam_ms + verify_ms;
    qout << input.id << ',' << input.stamp << ','; frameFields(qout, frame);
    qout << ',' << mint.ids.size() << ',' << mp << ',' << beam.ids.size() << ',' << bp << ','
         << verified.size() << ',' << success << ',' << legal << ',' << mint_ms << ',' << beam_ms << ','
         << verify_ms << ',' << compute_ms << ',' << elapsed_ms << ',' << cpu_ms << ',' << peakRssKiB() << '\n';
    for (size_t i = 0; i < mint.ids.size(); ++i)
      mintout << input.id << ',' << i+1 << ',' << mint.ids[i] << ',' << mint.scores[i] << '\n';
    for (size_t i = 0; i < verified.size(); ++i) {
      const auto& v = verified[i];
      const auto mit = std::find(mint.ids.begin(), mint.ids.end(), beam.ids[i]);
      const size_t mi = static_cast<size_t>(mit - mint.ids.begin());
      candidates << input.id << ',' << i+1 << ',' << beam.ids[i] << ',' << mint.scores.at(mi) << ','
                 << beam.scores[i] << ',' << beam.yaw_estimates[i] << ',' << beam.best_shifts[i] << ','
                 << v.success << ',' << v.fail_stage << ',' << v.n_correspondences << ','
                 << v.ransac_inliers_found << ',' << v.n_keypoint_inliers << ',' << v.inlier_rmse << ','
                 << (v.success && rigid(v.T_sensor)) << ',' << times[i];
      // Failed identity is saved for upstream fidelity but explicitly marked
      // invalid; it is not an estimated transform.
      for (int r = 0; r < 4; ++r) for (int c = 0; c < 4; ++c) candidates << ',' << v.T_sensor(r,c);
      candidates << '\n';
    }
    qout.flush(); candidates.flush(); mintout.flush();
    ++executed;
    std::cout << "QUERY " << input.id << " tokens=" << q.tokens.token_id.size()
              << " MINT_positive=" << mp << " BEAM_positive=" << bp
              << " Verify=" << success << '/' << verified.size()
              << " wall_ms=" << elapsed_ms << std::endl;
  }
  resources << "QUERY_COMPLETE," << ms(db_tick) << ',' << rssKiB() << ',' << peakRssKiB() << ',' << tokens_total << '\n';
  auto status = output(directory + "/run_status.csv");
  status << "status,references,queries,first_six_usable,full_cloud_registration_calls\n";
  status << (executed == 24 ? "COMPLETE" : "STOP_NO_USABLE_CANDIDATE") << ",389," << executed << ',' << usable_six << ",0\n";
}
}  // namespace p14
