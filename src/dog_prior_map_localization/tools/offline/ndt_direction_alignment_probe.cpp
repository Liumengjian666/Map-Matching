#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <numeric>
#include <sstream>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <vector>

#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <pcl/common/transforms.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/registration/ndt.h>

namespace
{
using PointT = pcl::PointXYZ;
using Cloud = pcl::PointCloud<PointT>;
using CloudPtr = Cloud::Ptr;
constexpr double kPi = 3.1415926535897932384626433832795;
const char *kScoreSemantics =
    "pcl_ndt_alignment_probability_objective_relative_log_profile_higher_is_better";

std::vector<std::string> splitCsv(const std::string &line)
{
  std::vector<std::string> fields;
  std::string field;
  std::stringstream stream(line);
  while (std::getline(stream, field, ',')) fields.push_back(field);
  if (!line.empty() && line.back() == ',') fields.emplace_back();
  return fields;
}

std::string trim(std::string value)
{
  while (!value.empty() && (value.back() == '\r' || value.back() == ' ' || value.back() == '\t')) value.pop_back();
  size_t first = 0;
  while (first < value.size() && (value[first] == ' ' || value[first] == '\t')) ++first;
  return value.substr(first);
}

double number(const std::unordered_map<std::string, std::string> &row, const std::string &key)
{
  const auto it = row.find(key);
  if (it == row.end() || it->second.empty() || it->second == "nan")
    return std::numeric_limits<double>::quiet_NaN();
  return std::stod(it->second);
}

std::vector<std::unordered_map<std::string, std::string>> readCsv(const std::string &path)
{
  std::ifstream input(path);
  if (!input.is_open()) throw std::runtime_error("failed to open CSV: " + path);
  std::string line;
  if (!std::getline(input, line)) throw std::runtime_error("empty CSV: " + path);
  const auto header = splitCsv(line);
  std::vector<std::unordered_map<std::string, std::string>> rows;
  while (std::getline(input, line))
  {
    if (line.empty()) continue;
    const auto fields = splitCsv(line);
    if (fields.size() != header.size()) throw std::runtime_error("CSV field count mismatch: " + path);
    std::unordered_map<std::string, std::string> row;
    for (size_t i = 0; i < header.size(); ++i) row.emplace(trim(header[i]), trim(fields[i]));
    rows.push_back(std::move(row));
  }
  return rows;
}

void finalizeCloud(const CloudPtr &cloud)
{
  cloud->width = static_cast<uint32_t>(cloud->points.size());
  cloud->height = 1;
  cloud->is_dense = true;
}

CloudPtr voxelDown(const CloudPtr &cloud, double leaf, double z_leaf, int max_points)
{
  CloudPtr down(new Cloud());
  if (leaf > 0.01)
  {
    pcl::VoxelGrid<PointT> voxel;
    voxel.setLeafSize(static_cast<float>(leaf), static_cast<float>(leaf), static_cast<float>(z_leaf));
    voxel.setInputCloud(cloud);
    voxel.filter(*down);
  }
  else *down = *cloud;
  if (max_points > 0 && static_cast<int>(down->size()) > max_points)
  {
    CloudPtr sampled(new Cloud());
    sampled->reserve(static_cast<size_t>(max_points));
    const double step = static_cast<double>(down->size() - 1) /
        static_cast<double>(std::max(max_points - 1, 1));
    for (int i = 0; i < max_points; ++i)
      sampled->push_back(down->points[static_cast<size_t>(std::round(i * step))]);
    finalizeCloud(sampled);
    return sampled;
  }
  finalizeCloud(down);
  return down;
}

CloudPtr preprocess(const CloudPtr &raw)
{
  CloudPtr filtered(new Cloud());
  filtered->reserve(raw->size());
  for (const auto &point : raw->points)
  {
    if (!std::isfinite(point.x) || !std::isfinite(point.y) || !std::isfinite(point.z)) continue;
    const double range = std::sqrt(static_cast<double>(point.x) * point.x +
                                   static_cast<double>(point.y) * point.y +
                                   static_cast<double>(point.z) * point.z);
    if (range < 0.5 || range > 80.0) continue;
    filtered->push_back(point);
  }
  finalizeCloud(filtered);
  return voxelDown(filtered, 0.25, 0.25, 1400);
}

class ProbeNdt : public pcl::NormalDistributionsTransform<PointT, PointT>
{
public:
  using Base = pcl::NormalDistributionsTransform<PointT, PointT>;
  using Base::setInputSource;
  using Base::setInputTarget;
  using Base::setResolution;

  double scoreAtPose(const Eigen::Matrix4f &pose)
  {
    if (!this->input_ || this->input_->empty()) return std::numeric_limits<double>::quiet_NaN();
    const double resolution = this->getResolution();
    const double outlier_ratio = this->getOulierRatio();
    const double c1 = 10.0 * (1.0 - outlier_ratio);
    const double c2 = outlier_ratio / std::pow(resolution, 3.0);
    const double d3 = -std::log(c2);
    this->gauss_d1_ = -std::log(c1 + c2) - d3;
    this->gauss_d2_ = -2.0 * std::log((-std::log(c1 * std::exp(-0.5) + c2) - d3) / this->gauss_d1_);
    this->point_gradient_.setZero();
    this->point_gradient_.block<3, 3>(0, 0).setIdentity();
    this->point_hessian_.setZero();
    Cloud transformed;
    pcl::transformPointCloud(*this->input_, transformed, pose);
    Eigen::Transform<float, 3, Eigen::Affine, Eigen::ColMajor> affine;
    affine.matrix() = pose;
    const Eigen::Vector3f euler = affine.rotation().eulerAngles(0, 1, 2);
    Eigen::Matrix<double, 6, 1> parameters;
    parameters << pose(0, 3), pose(1, 3), pose(2, 3), euler.x(), euler.y(), euler.z();
    Eigen::Matrix<double, 6, 1> gradient;
    Eigen::Matrix<double, 6, 6> hessian;
    return this->computeDerivatives(gradient, hessian, transformed, parameters, true);
  }
};

Eigen::Matrix4f poseFromRow(const std::unordered_map<std::string, std::string> &row)
{
  Eigen::Quaternionf q(static_cast<float>(number(row, "final_used_qw")),
                       static_cast<float>(number(row, "final_used_qx")),
                       static_cast<float>(number(row, "final_used_qy")),
                       static_cast<float>(number(row, "final_used_qz")));
  if (!q.coeffs().allFinite() || q.norm() < 1e-6f) throw std::runtime_error("invalid final_used quaternion");
  q.normalize();
  Eigen::Matrix4f pose = Eigen::Matrix4f::Identity();
  pose.block<3, 3>(0, 0) = q.toRotationMatrix();
  pose(0, 3) = static_cast<float>(number(row, "final_used_tx"));
  pose(1, 3) = static_cast<float>(number(row, "final_used_ty"));
  pose(2, 3) = static_cast<float>(number(row, "final_used_tz"));
  return pose;
}

struct Direction
{
  double lambda = std::numeric_limits<double>::quiet_NaN();
  Eigen::Vector3d vector = Eigen::Vector3d::Zero();
};

Direction schurDirection(const std::unordered_map<std::string, std::string> &row,
                         const std::string &type, int index)
{
  Direction result;
  result.lambda = number(row, "schur_" + type + "_lambda_" + std::to_string(index));
  result.vector.x() = number(row, "schur_" + type + "_v" + std::to_string(index) + "_x");
  result.vector.y() = number(row, "schur_" + type + "_v" + std::to_string(index) + "_y");
  result.vector.z() = number(row, "schur_" + type + "_v" + std::to_string(index) + "_z");
  // Stage3A.3 runtime CSVs contain the weakest Schur vector but not the
  // complete v1/v2 eigenbasis.  Keep index 0 usable for magnitude/1D
  // diagnostics and leave unavailable higher-basis directions invalid rather
  // than fabricating an orthogonal completion.
  if ((!result.vector.allFinite() || result.vector.norm() < 1e-8) && index == 0)
  {
    result.vector.x() = number(row, "schur_" + type + "_weak_x");
    result.vector.y() = number(row, "schur_" + type + "_weak_y");
    result.vector.z() = number(row, "schur_" + type + "_weak_z");
  }
  const double norm = result.vector.norm();
  if (!std::isfinite(result.lambda))
    throw std::runtime_error("invalid Schur eigenvector for " + type + " index " + std::to_string(index));
  if (!result.vector.allFinite() || norm < 1e-8) return result;
  result.vector /= norm;
  return result;
}

Eigen::Matrix4f perturbPose(const Eigen::Matrix4f &center, const Eigen::Vector3d &delta,
                            const std::string &type)
{
  Eigen::Matrix4f pose = center;
  if (type == "translation")
  {
    pose.block<3, 1>(0, 3) += delta.cast<float>();
  }
  else
  {
    const double angle = delta.norm();
    if (angle > 1e-12)
    {
      const Eigen::AngleAxisd rotation(angle, delta / angle);
      const Eigen::Matrix3d center_rotation = center.block<3, 3>(0, 0).cast<double>();
      pose.block<3, 3>(0, 0) = (rotation.toRotationMatrix() * center_rotation).cast<float>();
    }
  }
  return pose;
}

struct SurfaceFit
{
  Eigen::Matrix3d hessian = Eigen::Matrix3d::Constant(std::numeric_limits<double>::quiet_NaN());
  Eigen::Vector3d covariance_eigenvalues = Eigen::Vector3d::Constant(std::numeric_limits<double>::quiet_NaN());
  Eigen::Vector3d stddev_eigenvalues = Eigen::Vector3d::Constant(std::numeric_limits<double>::quiet_NaN());
  Eigen::Matrix3d sigma_vectors = Eigen::Matrix3d::Constant(std::numeric_limits<double>::quiet_NaN());
  double fit_rmse = std::numeric_limits<double>::quiet_NaN();
  double r2 = std::numeric_limits<double>::quiet_NaN();
  bool covariance_valid = false;
};

SurfaceFit fitLocalSurface(const std::vector<Eigen::Vector3d> &deltas,
                           const std::vector<double> &scores)
{
  SurfaceFit result;
  if (deltas.size() != scores.size() || deltas.size() < 10) return result;
  size_t center_index = 0;
  double center_norm = std::numeric_limits<double>::infinity();
  for (size_t i = 0; i < deltas.size(); ++i)
  {
    const double norm = deltas[i].squaredNorm();
    if (norm < center_norm) { center_norm = norm; center_index = i; }
  }
  const double center_score = scores[center_index];
  if (!(center_score > 0.0) || !std::isfinite(center_score)) return result;
  Eigen::MatrixXd design(static_cast<Eigen::Index>(deltas.size()), 10);
  Eigen::VectorXd observation(static_cast<Eigen::Index>(deltas.size()));
  for (size_t i = 0; i < deltas.size(); ++i)
  {
    const Eigen::Vector3d &d = deltas[i];
    design.row(static_cast<Eigen::Index>(i)) << 1.0, d.x(), d.y(), d.z(),
        -0.5 * d.x() * d.x(), -d.x() * d.y(), -d.x() * d.z(),
        -0.5 * d.y() * d.y(), -d.y() * d.z(), -0.5 * d.z() * d.z();
    observation(static_cast<Eigen::Index>(i)) =
        std::log(std::max(scores[i], std::numeric_limits<double>::min())) - std::log(center_score);
  }
  const Eigen::VectorXd coefficients = design.colPivHouseholderQr().solve(observation);
  const Eigen::VectorXd residual = design * coefficients - observation;
  result.fit_rmse = std::sqrt(residual.squaredNorm() / static_cast<double>(deltas.size()));
  const double mean = observation.mean();
  const double total = (observation.array() - mean).square().sum();
  result.r2 = total > 1e-15 ? 1.0 - residual.squaredNorm() / total : std::numeric_limits<double>::quiet_NaN();
  result.hessian << coefficients(4), coefficients(5), coefficients(6),
                    coefficients(5), coefficients(7), coefficients(8),
                    coefficients(6), coefficients(8), coefficients(9);
  Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> solver(result.hessian);
  if (solver.info() != Eigen::Success) return result;
  const Eigen::Vector3d h_eigenvalues = solver.eigenvalues();
  const double scale = std::max(h_eigenvalues.cwiseAbs().maxCoeff(), 1e-12);
  result.covariance_valid = h_eigenvalues.minCoeff() > 1e-8 * scale;
  for (int i = 0; i < 3; ++i)
  {
    // Eigen returns h0 <= h1 <= h2.  Keep the uncertainty basis semantic
    // index aligned with the Schur basis: index 0 is the flattest curvature
    // (largest 1/H proxy uncertainty), and index 2 is the strongest one.
    const int h_index = i;
    result.covariance_eigenvalues(i) = 1.0 / h_eigenvalues(h_index);
    result.stddev_eigenvalues(i) = std::sqrt(result.covariance_eigenvalues(i));
    result.sigma_vectors.col(i) = solver.eigenvectors().col(h_index);
  }
  return result;
}

double angleDegrees(const Eigen::Vector3d &a, const Eigen::Vector3d &b)
{
  if (!a.allFinite() || !b.allFinite() || a.norm() < 1e-10 || b.norm() < 1e-10)
    return std::numeric_limits<double>::quiet_NaN();
  const double cosine = std::max(-1.0, std::min(1.0, std::abs(a.normalized().dot(b.normalized()))));
  return std::acos(cosine) * 180.0 / kPi;
}

struct BestAssignment
{
  std::array<int, 3> permutation{{0, 1, 2}};
  double max_angle_deg = std::numeric_limits<double>::quiet_NaN();
  double mean_angle_deg = std::numeric_limits<double>::quiet_NaN();
};

BestAssignment bestAssignment(const std::vector<Direction> &schur, const SurfaceFit &fit)
{
  BestAssignment result;
  double best_score = -1.0;
  std::array<int, 3> permutation{{0, 1, 2}};
  do
  {
    double score = 0.0;
    std::array<double, 3> angles{};
    for (int i = 0; i < 3; ++i)
    {
      const double angle = angleDegrees(schur[i].vector, fit.sigma_vectors.col(permutation[i]));
      angles[static_cast<size_t>(i)] = angle;
      if (!std::isfinite(angle)) { score = -1.0; break; }
      score += std::cos(angle * kPi / 180.0);
    }
    if (score > best_score)
    {
      best_score = score;
      result.permutation = permutation;
      result.max_angle_deg = *std::max_element(angles.begin(), angles.end());
      result.mean_angle_deg = std::accumulate(angles.begin(), angles.end(), 0.0) / 3.0;
    }
  } while (std::next_permutation(permutation.begin(), permutation.end()));
  return result;
}

void writeValue(std::ofstream &out, double value)
{
  if (std::isfinite(value)) out << std::setprecision(17) << value;
  else out << "nan";
}

}  // namespace

int main(int argc, char **argv)
{
  try
  {
    if (argc < 4)
      throw std::runtime_error("usage: ndt_direction_alignment_probe <combined_frames.csv> <map.pcd> <output_dir>");
    const std::string frames_path = argv[1];
    const std::string map_path = argv[2];
    const std::string output_dir = argv[3];
    const auto frames = readCsv(frames_path);
    if (frames.empty()) throw std::runtime_error("combined frame CSV is empty");

    CloudPtr raw_map(new Cloud());
    if (pcl::io::loadPCDFile(map_path, *raw_map) != 0) throw std::runtime_error("failed to load map");
    CloudPtr finite_map(new Cloud());
    for (const auto &point : raw_map->points)
      if (std::isfinite(point.x) && std::isfinite(point.y) && std::isfinite(point.z)) finite_map->push_back(point);
    finalizeCloud(finite_map);
    CloudPtr map_cloud = voxelDown(finite_map, 0.15, 0.15, 0);
    CloudPtr target_cloud = voxelDown(map_cloud, 0.15, 0.15, 0);
    if (target_cloud->size() != 459154U)
      throw std::runtime_error("target size mismatch: " + std::to_string(target_cloud->size()));

    ProbeNdt ndt;
    ndt.setResolution(0.8);
    ndt.setInputTarget(target_cloud);
    std::ofstream surface_output(output_dir + "/local_score_surfaces.csv");
    std::ofstream eigen_output(output_dir + "/covariance_eigenvalues.csv");
    std::ofstream alignment_output(output_dir + "/direction_alignment.csv");
    std::ofstream performance_output(output_dir + "/probe_performance.csv");
    if (!surface_output.is_open() || !eigen_output.is_open() || !alignment_output.is_open() || !performance_output.is_open())
      throw std::runtime_error("failed to open Stage3A.6 outputs in: " + output_dir);

    surface_output << "frame_index,target_rel,t_rel,profile_type,dx,dy,dz,delta_unit,raw_ndt_score,log_score,score_semantics\n";
    eigen_output << "frame_index,target_rel,t_rel,profile_type,eigen_index,h_eigenvalue,covariance_eigenvalue,stddev_eigenvalue,vx,vy,vz,covariance_valid,fit_rmse,r2\n";
    alignment_output << "frame_index,target_rel,t_rel,translation_angle0_deg,translation_angle1_deg,translation_angle2_deg,"
        "rotation_angle0_deg,rotation_angle1_deg,rotation_angle2_deg,"
        "translation_covariance_eigenvalue0,translation_covariance_eigenvalue1,translation_covariance_eigenvalue2,"
        "translation_stddev0,translation_stddev1,translation_stddev2,rotation_covariance_eigenvalue0,"
        "rotation_covariance_eigenvalue1,rotation_covariance_eigenvalue2,rotation_stddev0,rotation_stddev1,rotation_stddev2,"
        "translation_lambda0,translation_lambda1,translation_lambda2,"
        "rotation_lambda0,rotation_lambda1,rotation_lambda2,translation_covariance_valid,rotation_covariance_valid,"
        "translation_fit_rmse,rotation_fit_rmse,translation_r2,rotation_r2,reference_A_deviation_m,reference_B_deviation_m,"
        "reference_disagreement,translation_best_perm0,translation_best_perm1,translation_best_perm2,"
        "translation_best_max_angle_deg,translation_best_mean_angle_deg,rotation_best_perm0,rotation_best_perm1,"
        "rotation_best_perm2,rotation_best_max_angle_deg,rotation_best_mean_angle_deg,"
        "translation_vs0_x,translation_vs0_y,translation_vs0_z,translation_vs1_x,translation_vs1_y,translation_vs1_z,"
        "translation_vs2_x,translation_vs2_y,translation_vs2_z,translation_vu0_x,translation_vu0_y,translation_vu0_z,"
        "translation_vu1_x,translation_vu1_y,translation_vu1_z,translation_vu2_x,translation_vu2_y,translation_vu2_z,"
        "rotation_vs0_x,rotation_vs0_y,rotation_vs0_z,rotation_vs1_x,rotation_vs1_y,rotation_vs1_z,"
        "rotation_vs2_x,rotation_vs2_y,rotation_vs2_z,rotation_vu0_x,rotation_vu0_y,rotation_vu0_z,"
        "rotation_vu1_x,rotation_vu1_y,rotation_vu1_z,rotation_vu2_x,rotation_vu2_y,rotation_vu2_z\n";

    std::vector<double> cell_times;
    std::vector<double> frame_times;
    const std::array<std::string, 2> types{{"translation", "rotation"}};
    for (const auto &row : frames)
    {
      const auto frame_start = std::chrono::steady_clock::now();
      CloudPtr raw(new Cloud());
      if (pcl::io::loadPCDFile(row.at("raw_pcd_path"), *raw) != 0)
        throw std::runtime_error("failed to load scan: " + row.at("raw_pcd_path"));
      finalizeCloud(raw);
      CloudPtr source = preprocess(raw);
      ndt.setInputSource(source);
      const Eigen::Matrix4f center_pose = poseFromRow(row);
      SurfaceFit fits[2];
      std::vector<Direction> schur[2];

      for (size_t type_index = 0; type_index < types.size(); ++type_index)
      {
        const std::string &type = types[type_index];
        const double step = type == "translation" ? 0.1 : 1.0;
        const double half_range = type == "translation" ? 0.3 : 2.0;
        std::vector<double> grid;
        for (int i = -static_cast<int>(std::round(half_range / step));
             i <= static_cast<int>(std::round(half_range / step)); ++i)
          grid.push_back(static_cast<double>(i) * step);
        std::vector<Eigen::Vector3d> deltas;
        std::vector<double> scores;
        deltas.reserve(grid.size() * grid.size() * grid.size());
        scores.reserve(deltas.capacity());
        for (double dx : grid) for (double dy : grid) for (double dz : grid)
        {
          Eigen::Vector3d delta(dx, dy, dz);
          if (type == "rotation") delta *= kPi / 180.0;
          const auto score_start = std::chrono::steady_clock::now();
          const double score = ndt.scoreAtPose(perturbPose(center_pose, delta, type));
          cell_times.push_back(std::chrono::duration<double, std::micro>(
              std::chrono::steady_clock::now() - score_start).count());
          if (!(score > 0.0) || !std::isfinite(score)) throw std::runtime_error("invalid NDT score");
          deltas.push_back(delta);
          scores.push_back(score);
          surface_output << row.at("frame_index") << ',' << row.at("target_rel") << ',' << row.at("t_rel") << ',' << type << ',';
          writeValue(surface_output, dx); surface_output << ','; writeValue(surface_output, dy); surface_output << ','; writeValue(surface_output, dz);
          surface_output << ',' << (type == "translation" ? "m" : "deg") << ',';
          writeValue(surface_output, score); surface_output << ',';
          writeValue(surface_output, std::log(score)); surface_output << ',' << kScoreSemantics << '\n';
        }
        fits[type_index] = fitLocalSurface(deltas, scores);
        for (int i = 0; i < 3; ++i) schur[type_index].push_back(schurDirection(row, type, i));
        Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> h_solver(fits[type_index].hessian);
        const Eigen::Vector3d h_values = h_solver.info() == Eigen::Success ? h_solver.eigenvalues() : Eigen::Vector3d::Constant(std::numeric_limits<double>::quiet_NaN());
        for (int i = 0; i < 3; ++i)
        {
          eigen_output << row.at("frame_index") << ',' << row.at("target_rel") << ',' << row.at("t_rel") << ',' << type << ',' << i << ',';
          // Report the same weakest-to-strongest ordering as the fitted
          // covariance eigenvectors below: h0, h1, h2.
          writeValue(eigen_output, h_values(i)); eigen_output << ',';
          writeValue(eigen_output, fits[type_index].covariance_eigenvalues(i)); eigen_output << ',';
          writeValue(eigen_output, fits[type_index].stddev_eigenvalues(i)); eigen_output << ',';
          writeValue(eigen_output, fits[type_index].sigma_vectors(0, i)); eigen_output << ',';
          writeValue(eigen_output, fits[type_index].sigma_vectors(1, i)); eigen_output << ',';
          writeValue(eigen_output, fits[type_index].sigma_vectors(2, i)); eigen_output << ','
              << (fits[type_index].covariance_valid ? 1 : 0) << ',';
          writeValue(eigen_output, fits[type_index].fit_rmse); eigen_output << ',';
          writeValue(eigen_output, fits[type_index].r2); eigen_output << '\n';
        }
      }

      alignment_output << row.at("frame_index") << ',' << row.at("target_rel") << ',' << row.at("t_rel") << ',';
      for (size_t type_index = 0; type_index < types.size(); ++type_index)
      {
        const SurfaceFit &fit = fits[type_index];
        for (int i = 0; i < 3; ++i)
        {
          const double angle = fit.covariance_valid ? angleDegrees(schur[type_index][i].vector, fit.sigma_vectors.col(i)) : std::numeric_limits<double>::quiet_NaN();
          writeValue(alignment_output, angle); alignment_output << ',';
        }
      }
      for (int i = 0; i < 3; ++i) { writeValue(alignment_output, fits[0].covariance_eigenvalues(i)); alignment_output << ','; }
      for (int i = 0; i < 3; ++i) { writeValue(alignment_output, fits[0].stddev_eigenvalues(i)); alignment_output << ','; }
      for (int i = 0; i < 3; ++i) { writeValue(alignment_output, fits[1].covariance_eigenvalues(i)); alignment_output << ','; }
      for (int i = 0; i < 3; ++i) { writeValue(alignment_output, fits[1].stddev_eigenvalues(i)); alignment_output << ','; }
      for (size_t type_index = 0; type_index < types.size(); ++type_index)
        for (int i = 0; i < 3; ++i) { writeValue(alignment_output, schur[type_index][i].lambda); alignment_output << ','; }
      alignment_output << (fits[0].covariance_valid ? 1 : 0) << ',' << (fits[1].covariance_valid ? 1 : 0) << ',';
      writeValue(alignment_output, fits[0].fit_rmse); alignment_output << ','; writeValue(alignment_output, fits[1].fit_rmse); alignment_output << ',';
      writeValue(alignment_output, fits[0].r2); alignment_output << ','; writeValue(alignment_output, fits[1].r2); alignment_output << ',';
      writeValue(alignment_output, number(row, "reference_A_deviation_m")); alignment_output << ',';
      writeValue(alignment_output, number(row, "reference_B_deviation_m")); alignment_output << ',';
      alignment_output << (number(row, "reference_disagreement") > 0.5 ? 1 : 0) << ',';
      const BestAssignment translation_assignment = bestAssignment(schur[0], fits[0]);
      const BestAssignment rotation_assignment = bestAssignment(schur[1], fits[1]);
      alignment_output << translation_assignment.permutation[0] << ',' << translation_assignment.permutation[1] << ','
                       << translation_assignment.permutation[2] << ',';
      writeValue(alignment_output, translation_assignment.max_angle_deg); alignment_output << ',';
      writeValue(alignment_output, translation_assignment.mean_angle_deg); alignment_output << ',';
      alignment_output << rotation_assignment.permutation[0] << ',' << rotation_assignment.permutation[1] << ','
                       << rotation_assignment.permutation[2] << ',';
      writeValue(alignment_output, rotation_assignment.max_angle_deg); alignment_output << ',';
      writeValue(alignment_output, rotation_assignment.mean_angle_deg); alignment_output << ',';
      for (size_t type_index = 0; type_index < types.size(); ++type_index)
      {
        for (int i = 0; i < 3; ++i)
          for (int axis = 0; axis < 3; ++axis) { writeValue(alignment_output, schur[type_index][i].vector(axis)); alignment_output << ','; }
        for (int i = 0; i < 3; ++i)
          for (int axis = 0; axis < 3; ++axis)
          {
            writeValue(alignment_output, fits[type_index].sigma_vectors(axis, i));
            const bool last = type_index == types.size() - 1 && i == 2 && axis == 2;
            if (!last) alignment_output << ',';
          }
      }
      alignment_output << '\n';
      frame_times.push_back(std::chrono::duration<double, std::milli>(
          std::chrono::steady_clock::now() - frame_start).count());
    }
    auto mean = [](const std::vector<double> &values) {
      return values.empty() ? std::numeric_limits<double>::quiet_NaN() :
          std::accumulate(values.begin(), values.end(), 0.0) / values.size();
    };
    performance_output << "frames,score_cells,mean_cell_us,max_cell_us,mean_frame_ms,max_frame_ms,score_semantics\n";
    performance_output << frames.size() << ',' << cell_times.size() << ',';
    writeValue(performance_output, mean(cell_times)); performance_output << ',';
    writeValue(performance_output, cell_times.empty() ? std::numeric_limits<double>::quiet_NaN() : *std::max_element(cell_times.begin(), cell_times.end())); performance_output << ',';
    writeValue(performance_output, mean(frame_times)); performance_output << ',';
    writeValue(performance_output, frame_times.empty() ? std::numeric_limits<double>::quiet_NaN() : *std::max_element(frame_times.begin(), frame_times.end())); performance_output << ',' << kScoreSemantics << '\n';
    std::cout << "frames=" << frames.size() << " target_size=" << target_cloud->size() << " score_cells=" << cell_times.size() << '\n';
  }
  catch (const std::exception &error)
  {
    std::cerr << "ndt_direction_alignment_probe: " << error.what() << '\n';
    return 2;
  }
  return 0;
}
