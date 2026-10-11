#include "probe_io.hpp"
#include "real_probe.hpp"
#include <inlier_core/matcher.hpp>
#include <inlier_core/verify.hpp>
#include <Eigen/Geometry>
#include <iostream>

static void smoke(const std::string& fixture, const std::string& result) {
  auto start = p14::Clock::now();
  inlier::InLiERConfig grid;
  inlier::Encoder encoder(grid);
  const auto points = p14::preprocess(p14::readPoints(fixture));
  const auto encoded = encoder.Encode(points);
  const auto& kp = encoded.first;
  const auto& tokens = encoded.second;
  p14::validate(kp, tokens);
  inlier::Matcher matcher(grid);
  matcher.Add(17, tokens.token_id);
  matcher.Finalize();
  const auto shortlist = matcher.Shortlist(tokens.token_id, p14::mintConfig(), -1, -1);
  const auto beam = matcher.BeamScore(tokens.token_id, shortlist.ids, p14::beamConfig(), -1, -1);
  if (shortlist.ids.empty() || shortlist.scores.at(0) <= 0 || beam.ids.empty() || beam.scores.at(0) <= 0)
    throw std::runtime_error("Author API smoke has no positive candidate");
  const auto identity = inlier::Verify(tokens.token_id, kp, tokens.token_id, kp,
                                      beam.best_shifts.at(0), grid, p14::verifyConfig());
  // Independent frame-direction unit test, not a real-data pose prior:
  // construct query sensor keypoints with known query->reference SE(3).
  Eigen::Matrix4d expected = Eigen::Matrix4d::Identity();
  expected.topLeftCorner<3,3>() = (Eigen::AngleAxisd(.4, Eigen::Vector3d::UnitZ()) *
                                  Eigen::AngleAxisd(.1, Eigen::Vector3d::UnitX())).toRotationMatrix();
  expected.topRightCorner<3,1>() << 1.2, -.7, .3;
  // The author's first-index token correspondence is many-to-one. Preserve
  // raw smoke self-match diagnostic, but use unique (h,r,a) correspondence
  // keys for the exact frame-direction test instead of demanding an exact
  // pose from aliased synthetic tokens.
  std::set<std::tuple<int,int,int>> keys;
  std::vector<Eigen::Index> selected;
  inlier::Tokens unit_tokens;
  for (size_t i = 0; i < tokens.token_id.size(); ++i) {
    auto bins = inlier::UnpackToken(tokens.token_id[i], 20, 7, 60);
    if (keys.emplace(bins.hb, bins.rb, bins.ab).second) {
      selected.push_back(i); unit_tokens.token_id.push_back(tokens.token_id[i]);
    }
  }
  inlier::Keypoints unit_db;
  unit_db.T_ground = kp.T_ground;
  unit_db.p.resize(selected.size(), 3);
  for (size_t i = 0; i < selected.size(); ++i) unit_db.p.row(i) = kp.p.row(selected[i]);
  inlier::Keypoints query;
  query.T_ground = unit_db.T_ground * expected;
  query.p.resize(unit_db.p.rows(), 3);
  for (Eigen::Index i = 0; i < unit_db.p.rows(); ++i)
    query.p.row(i) = (expected.topLeftCorner<3,3>().transpose() *
                     (unit_db.p.row(i).transpose() - expected.topRightCorner<3,1>())).transpose();
  const auto directed = inlier::Verify(unit_tokens.token_id, query, unit_tokens.token_id, unit_db, 0, grid, p14::verifyConfig());
  const auto repeated = inlier::Verify(unit_tokens.token_id, query, unit_tokens.token_id, unit_db, 0, grid, p14::verifyConfig());
  const double identity_error = (identity.T_sensor - Eigen::Matrix4d::Identity()).norm();
  const double direction_error = (directed.T_sensor - expected).norm();
  std::cout << "SMOKE_DIAGNOSTIC keypoints=" << kp.p.rows() << " tokens=" << tokens.token_id.size()
            << " identity_success=" << identity.success << " identity_fail_stage=" << identity.fail_stage
            << " direction_success=" << directed.success << " direction_fail_stage=" << directed.fail_stage
            << " identity_error=" << identity_error << " direction_error=" << direction_error << std::endl;
  if (!identity.success || !directed.success || !p14::rigid(identity.T_sensor) ||
      !p14::rigid(directed.T_sensor) || direction_error > 1e-8 || !repeated.success ||
      !(directed.T_sensor.array() == repeated.T_sensor.array()).all())
    throw std::runtime_error("Author Verify smoke / transform direction failed");
  auto out = p14::output(result);
  out << "status,points,keypoints,tokens,mint_score,beam_score,identity_error,direction_error,wall_ms,peak_rss_kib\n";
  out << "PASS," << points.rows() << ',' << kp.p.rows() << ',' << tokens.token_id.size() << ','
      << shortlist.scores.at(0) << ',' << beam.scores.at(0) << ',' << identity_error << ','
      << direction_error << ',' << p14::ms(start) << ',' << p14::peakRssKiB() << '\n';
}

int main(int argc, char** argv) {
  try {
    if (argc == 4 && std::string(argv[1]) == "--smoke") smoke(argv[2], argv[3]);
    else if (argc == 3 && std::string(argv[1]) == "--real") p14::real(argv[2]);
    else if (argc == 4 && std::string(argv[1]) == "--preprocess") {
      auto out = p14::output(argv[3]);
      p14::writePoints(out, p14::preprocess(p14::load(argv[2])));
    } else throw std::runtime_error("Usage: inlier_baseline --smoke FIXTURE_F64 OUTPUT_CSV | --preprocess PCD OUTPUT_F64 | --real FROZEN_DIRECTORY");
    return 0;
  } catch (const std::exception& e) {
    std::cerr << "P14_ERROR: " << e.what() << '\n';
    return 1;
  }
}
