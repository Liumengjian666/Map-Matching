#include <pcl/common/transforms.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/registration/ndt.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <Eigen/Cholesky>
#include <Eigen/Eigenvalues>
#include <Eigen/Geometry>
#include <Eigen/QR>
#include <Eigen/SVD>

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <map>
#include <set>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

namespace {
using Point = pcl::PointXYZ;
using Cloud = pcl::PointCloud<Point>;
using Ndt = pcl::NormalDistributionsTransform<Point, Point>;
using Vector3d = Eigen::Vector3d;
using Vector6d = Eigen::Matrix<double, 6, 1>;
using Matrix3d = Eigen::Matrix3d;
using Matrix6d = Eigen::Matrix<double, 6, 6>;
using Clock = std::chrono::steady_clock;
constexpr double kResolution = 0.8;
constexpr double kCoordinateValidationEpsilon = 1e-7;
const double kMaxSpatialJacobianCondition =
    1.0 / std::sqrt(std::numeric_limits<double>::epsilon());
constexpr const char* kExpectedMapSha =
    "2b571af236738a0664befacdc9c783246e991416a8915bcfebb9d2dc074e4570";

class AuditedNdt : public Ndt {
 public:
  double scoreHessian(Cloud& transformed, Vector6d& p, Matrix6d* hessian) {
    Vector6d gradient;
    Matrix6d local_hessian;
    const double score = computeDerivatives(gradient, local_hessian,
                                            transformed, p, true);
    if (hessian) *hessian = local_hessian;
    return score;
  }
};

struct CsvTable {
  std::vector<std::string> header;
  std::vector<std::vector<std::string>> rows;
  std::map<std::string, std::size_t> columns;

  std::size_t col(const std::string& name) const {
    const auto found = columns.find(name);
    if (found == columns.end()) throw std::runtime_error("missing CSV column: " + name);
    return found->second;
  }

  const std::string& get(const std::vector<std::string>& row,
                         const std::string& name) const {
    const std::size_t index = col(name);
    if (index >= row.size()) throw std::runtime_error("short CSV row at " + name);
    return row[index];
  }
};

std::vector<std::string> parseCsvLine(const std::string& line) {
  std::vector<std::string> fields;
  std::string field;
  bool quoted = false;
  for (std::size_t i = 0; i < line.size(); ++i) {
    const char c = line[i];
    if (quoted) {
      if (c == '"' && i + 1 < line.size() && line[i + 1] == '"') {
        field.push_back('"');
        ++i;
      } else if (c == '"') {
        quoted = false;
      } else {
        field.push_back(c);
      }
    } else if (c == ',') {
      fields.push_back(field);
      field.clear();
    } else if (c == '"' && field.empty()) {
      quoted = true;
    } else {
      field.push_back(c);
    }
  }
  if (quoted) throw std::runtime_error("unterminated CSV quote");
  fields.push_back(field);
  return fields;
}

CsvTable readCsv(const std::string& path) {
  std::ifstream input(path);
  if (!input) throw std::runtime_error("cannot open CSV: " + path);
  std::string line;
  if (!std::getline(input, line)) throw std::runtime_error("empty CSV: " + path);
  CsvTable table;
  table.header = parseCsvLine(line);
  for (std::size_t i = 0; i < table.header.size(); ++i)
    table.columns[table.header[i]] = i;
  while (std::getline(input, line)) {
    if (!line.empty()) table.rows.push_back(parseCsvLine(line));
  }
  return table;
}

std::string csvQuote(const std::string& value) {
  if (value.find_first_of(",\"\n\r") == std::string::npos) return value;
  std::string escaped = "\"";
  for (const char c : value) {
    if (c == '"') escaped.push_back('"');
    escaped.push_back(c);
  }
  escaped.push_back('"');
  return escaped;
}

void writeHeader(std::ofstream& out, const std::vector<std::string>& header) {
  for (std::size_t i = 0; i < header.size(); ++i) {
    if (i) out << ',';
    out << csvQuote(header[i]);
  }
  out << '\n';
}

void writeRow(std::ofstream& out, const std::vector<std::string>& row) {
  for (std::size_t i = 0; i < row.size(); ++i) {
    if (i) out << ',';
    out << csvQuote(row[i]);
  }
  out << '\n';
}

std::string number(double value) {
  if (!std::isfinite(value)) return "nan";
  std::ostringstream out;
  out << std::setprecision(15) << value;
  return out.str();
}

std::string integer(std::uint64_t value) {
  return std::to_string(value);
}

template <typename Derived>
std::string flatten(const Eigen::MatrixBase<Derived>& matrix) {
  std::ostringstream out;
  out << std::setprecision(15);
  bool first = true;
  for (Eigen::Index row = 0; row < matrix.rows(); ++row) {
    for (Eigen::Index col = 0; col < matrix.cols(); ++col) {
      if (!first) out << ';';
      first = false;
      out << matrix(row, col);
    }
  }
  return out.str();
}

std::string flattenVector(const Eigen::VectorXd& vector) {
  std::ostringstream out;
  out << std::setprecision(15);
  for (Eigen::Index i = 0; i < vector.size(); ++i) {
    if (i) out << ';';
    out << vector(i);
  }
  return out.str();
}

std::string flattenVector6(const Vector6d& vector) {
  return flattenVector(Eigen::VectorXd(vector));
}

std::uint64_t parseU64(const std::string& value) {
  return static_cast<std::uint64_t>(std::stoull(value));
}

double parseDouble(const std::string& value) { return std::stod(value); }

void finalize(const Cloud::Ptr& cloud, bool dense = true) {
  cloud->width = static_cast<std::uint32_t>(cloud->size());
  cloud->height = 1;
  cloud->is_dense = dense;
}

Cloud::Ptr transformCloud(const Cloud::Ptr& input, const Eigen::Matrix4f& pose) {
  Cloud::Ptr output(new Cloud);
  pcl::transformPointCloud(*input, *output, pose);
  finalize(output);
  return output;
}

Cloud::Ptr voxelDown(const Cloud::Ptr& input, double leaf, int cap) {
  Cloud::Ptr down(new Cloud);
  pcl::VoxelGrid<Point> voxel;
  voxel.setLeafSize(static_cast<float>(leaf), static_cast<float>(leaf),
                    static_cast<float>(leaf));
  voxel.setInputCloud(input);
  voxel.filter(*down);
  if (cap > 0 && static_cast<int>(down->size()) > cap) {
    Cloud::Ptr sampled(new Cloud);
    sampled->reserve(static_cast<std::size_t>(cap));
    const double increment = static_cast<double>(down->size() - 1) /
                             static_cast<double>(std::max(cap - 1, 1));
    for (int i = 0; i < cap; ++i)
      sampled->push_back(down->points[static_cast<std::size_t>(std::llround(i * increment))]);
    finalize(sampled);
    return sampled;
  }
  finalize(down);
  return down;
}

Cloud::Ptr loadTarget(const std::string& path) {
  Cloud::Ptr raw(new Cloud);
  if (pcl::io::loadPCDFile<Point>(path, *raw) != 0)
    throw std::runtime_error("cannot load frozen map: " + path);
  Cloud::Ptr finite(new Cloud);
  finite->reserve(raw->size());
  for (const Point& point : raw->points)
    if (std::isfinite(point.x) && std::isfinite(point.y) && std::isfinite(point.z))
      finite->push_back(point);
  finalize(finite);
  return voxelDown(voxelDown(finite, 0.15, 0), 0.15, 0);
}

Cloud::Ptr preprocessSource(const Cloud::Ptr& raw) {
  Cloud::Ptr filtered(new Cloud);
  filtered->reserve(raw->size());
  for (const Point& point : raw->points) {
    if (!std::isfinite(point.x) || !std::isfinite(point.y) ||
        !std::isfinite(point.z)) continue;
    const double range = std::sqrt(point.x * point.x + point.y * point.y +
                                   point.z * point.z);
    if (range < 0.5 || range > 80.0) continue;
    filtered->push_back(point);
  }
  finalize(filtered);
  return voxelDown(filtered, 0.25, 1400);
}

std::uint64_t sourceCloudHash(const Cloud::Ptr& cloud) {
  constexpr std::uint64_t kOffset = 1469598103934665603ULL;
  constexpr std::uint64_t kPrime = 1099511628211ULL;
  std::uint64_t hash = kOffset;
  auto mix = [&hash](std::uint8_t value) { hash = (hash ^ value) * kPrime; };
  auto mix32 = [&mix](std::uint32_t value) {
    for (int shift = 0; shift < 32; shift += 8)
      mix(static_cast<std::uint8_t>(value >> shift));
  };
  mix32(cloud->width);
  mix32(cloud->height);
  mix32(cloud->is_dense ? 1U : 0U);
  mix32(static_cast<std::uint32_t>(cloud->size()));
  for (const Point& point : cloud->points) {
    std::uint32_t bits;
    std::memcpy(&bits, &point.x, sizeof(bits)); mix32(bits);
    std::memcpy(&bits, &point.y, sizeof(bits)); mix32(bits);
    std::memcpy(&bits, &point.z, sizeof(bits)); mix32(bits);
  }
  return hash;
}

Vector6d poseVector(const Eigen::Matrix4f& pose) {
  const Eigen::Vector3d euler =
      pose.block<3, 3>(0, 0).cast<double>().eulerAngles(0, 1, 2);
  Vector6d result;
  result << pose(0, 3), pose(1, 3), pose(2, 3),
      euler.x(), euler.y(), euler.z();
  return result;
}

Eigen::Matrix4f poseFromVector(const Vector6d& pose) {
  const Eigen::AngleAxisf rx(static_cast<float>(pose(3)), Vector3d::UnitX().cast<float>());
  const Eigen::AngleAxisf ry(static_cast<float>(pose(4)), Vector3d::UnitY().cast<float>());
  const Eigen::AngleAxisf rz(static_cast<float>(pose(5)), Vector3d::UnitZ().cast<float>());
  Eigen::Matrix4f result = Eigen::Matrix4f::Identity();
  result.block<3, 3>(0, 0) = (rx * ry * rz).toRotationMatrix();
  result.block<3, 1>(0, 3) = pose.head<3>().cast<float>();
  return result;
}

Eigen::Matrix4f poseFromQuaternion(double x, double y, double z,
                                  double qx, double qy, double qz,
                                  double qw) {
  Eigen::Quaternionf q(static_cast<float>(qw), static_cast<float>(qx),
                       static_cast<float>(qy), static_cast<float>(qz));
  if (!q.coeffs().allFinite() || q.norm() < 1e-6f)
    throw std::runtime_error("invalid input quaternion");
  q.normalize();
  Eigen::Matrix4f pose = Eigen::Matrix4f::Identity();
  pose.block<3, 3>(0, 0) = q.toRotationMatrix();
  pose.block<3, 1>(0, 3) = Eigen::Vector3f(static_cast<float>(x),
      static_cast<float>(y), static_cast<float>(z));
  return pose;
}

struct SpatialJacobian {
  bool valid = false;
  Matrix3d value = Matrix3d::Constant(std::numeric_limits<double>::quiet_NaN());
  Matrix3d inverse = Matrix3d::Constant(std::numeric_limits<double>::quiet_NaN());
  double condition = std::numeric_limits<double>::infinity();
};

Eigen::Matrix3d eulerXyzRotation(const Vector3d& euler) {
  const Eigen::AngleAxisd rx(euler.x(), Vector3d::UnitX());
  const Eigen::AngleAxisd ry(euler.y(), Vector3d::UnitY());
  const Eigen::AngleAxisd rz(euler.z(), Vector3d::UnitZ());
  return (rx * ry * rz).toRotationMatrix();
}

SpatialJacobian makeSpatialJacobian(const Vector3d& euler) {
  SpatialJacobian result;
  const Eigen::AngleAxisd rx(euler.x(), Vector3d::UnitX());
  const Eigen::AngleAxisd ry(euler.y(), Vector3d::UnitY());
  result.value.col(0) = Vector3d::UnitX();
  result.value.col(1) = rx * Vector3d::UnitY();
  result.value.col(2) = rx * ry * Vector3d::UnitZ();
  if (!result.value.allFinite()) return result;

  Eigen::JacobiSVD<Matrix3d> svd(result.value);
  const Vector3d singular = svd.singularValues();
  if (!singular.allFinite() || singular.minCoeff() <= 0.0) return result;
  result.condition = singular.maxCoeff() / singular.minCoeff();
  if (!std::isfinite(result.condition) ||
      result.condition > kMaxSpatialJacobianCondition) return result;

  Eigen::ColPivHouseholderQR<Matrix3d> qr(result.value);
  if (qr.rank() != 3) return result;
  result.inverse = qr.solve(Matrix3d::Identity());
  const double residual = (result.value * result.inverse - Matrix3d::Identity()).norm();
  result.valid = result.inverse.allFinite() && std::isfinite(residual) &&
                 residual <= 1e-10;
  return result;
}

Vector3d rotationLog(const Matrix3d& rotation) {
  const Eigen::AngleAxisd angle_axis(rotation);
  return angle_axis.angle() * angle_axis.axis();
}

struct JacobianValidation {
  SpatialJacobian jacobian;
  std::array<Vector3d, 3> finite_difference;
  std::array<double, 3> errors{{std::numeric_limits<double>::infinity(),
                                std::numeric_limits<double>::infinity(),
                                std::numeric_limits<double>::infinity()}};
  bool pass = false;
};

JacobianValidation validateSpatialJacobian(const Vector3d& euler) {
  JacobianValidation result;
  result.jacobian = makeSpatialJacobian(euler);
  if (!result.jacobian.valid) return result;
  const Matrix3d base = eulerXyzRotation(euler);
  for (int axis = 0; axis < 3; ++axis) {
    Vector3d perturbed = euler;
    perturbed(axis) += kCoordinateValidationEpsilon;
    const Matrix3d relative = eulerXyzRotation(perturbed) * base.transpose();
    result.finite_difference[static_cast<std::size_t>(axis)] =
        rotationLog(relative) / kCoordinateValidationEpsilon;
    result.errors[static_cast<std::size_t>(axis)] =
        (result.finite_difference[static_cast<std::size_t>(axis)] -
         result.jacobian.value.col(axis)).norm();
  }
  const double max_error = *std::max_element(result.errors.begin(), result.errors.end());
  result.pass = std::isfinite(max_error) && max_error <= 1e-5;
  return result;
}

double rotationDifferenceDeg(const Eigen::Matrix4f& a, const Eigen::Matrix4f& b) {
  const Matrix3d delta = a.block<3, 3>(0, 0).cast<double>().transpose() *
                         b.block<3, 3>(0, 0).cast<double>();
  const double cosine = std::max(-1.0, std::min(1.0, (delta.trace() - 1.0) * 0.5));
  return std::acos(cosine) * 180.0 / M_PI;
}

double translationDifference(const Eigen::Matrix4f& a, const Eigen::Matrix4f& b) {
  return (a.block<3, 1>(0, 3) - b.block<3, 1>(0, 3)).norm();
}

struct FixtureExpectation {
  std::string name;
  std::array<bool, 3> weak_translation{{false, false, false}};
  std::array<bool, 3> weak_rotation{{false, false, false}};
};

int countAxes(const std::array<bool, 3>& axes) {
  return static_cast<int>(std::count(axes.begin(), axes.end(), true));
}

std::array<bool, 3> parseAxes(const std::string& text,
                              const std::array<std::string, 3>& names) {
  std::array<bool, 3> result{{false, false, false}};
  if (text.empty() || text == "none") return result;
  std::stringstream stream(text);
  std::string name;
  while (std::getline(stream, name, '|')) {
    if (name == "roll") name = "rotation_x";
    else if (name == "pitch") name = "rotation_y";
    else if (name == "yaw") name = "rotation_z";
    bool found = false;
    for (int i = 0; i < 3; ++i) {
      if (name == names[static_cast<std::size_t>(i)]) {
        result[static_cast<std::size_t>(i)] = true;
        found = true;
        break;
      }
    }
    if (!found) throw std::runtime_error("unknown expected physical axis: " + name);
  }
  return result;
}

std::vector<FixtureExpectation> readExpectations(const std::string& path) {
  const CsvTable table = readCsv(path);
  std::vector<FixtureExpectation> result;
  for (const auto& row : table.rows) {
    FixtureExpectation item;
    item.name = table.get(row, "fixture");
    item.weak_translation = parseAxes(table.get(row, "expected_weak_translation_axes"),
                                      {{"x", "y", "z"}});
    item.weak_rotation = parseAxes(table.get(row, "expected_weak_rotation_axes"),
                                   {{"rotation_x", "rotation_y", "rotation_z"}});
    result.push_back(item);
  }
  return result;
}

struct Perturbation {
  std::string id;
  Vector6d delta = Vector6d::Zero();
};

std::vector<Perturbation> readPerturbations(const std::string& path) {
  const CsvTable table = readCsv(path);
  std::vector<Perturbation> result;
  for (const auto& row : table.rows) {
    Perturbation p;
    p.id = table.get(row, "perturbation_id");
    p.delta << parseDouble(table.get(row, "tx_m")),
        parseDouble(table.get(row, "ty_m")), parseDouble(table.get(row, "tz_m")),
        parseDouble(table.get(row, "rx_deg")) * M_PI / 180.0,
        parseDouble(table.get(row, "ry_deg")) * M_PI / 180.0,
        parseDouble(table.get(row, "rz_deg")) * M_PI / 180.0;
    if (p.delta.head<3>().cwiseAbs().maxCoeff() > 0.1000001 ||
        p.delta.tail<3>().cwiseAbs().maxCoeff() > M_PI / 180.0 + 1e-12)
      throw std::runtime_error("pre-registered perturbation exceeds its bound");
    result.push_back(p);
  }
  if (result.size() != 20) throw std::runtime_error("expected exactly 20 perturbations");
  return result;
}

void appendGridPlane(Cloud::Ptr cloud, int axis, double fixed,
                     double first_min, double first_max,
                     double second_min, double second_max, double spacing) {
  for (double a = first_min; a <= first_max + 1e-9; a += spacing) {
    for (double b = second_min; b <= second_max + 1e-9; b += spacing) {
      if (axis == 2) cloud->push_back(Point(static_cast<float>(a), static_cast<float>(b), static_cast<float>(fixed)));
      else if (axis == 1) cloud->push_back(Point(static_cast<float>(a), static_cast<float>(fixed), static_cast<float>(b)));
      else cloud->push_back(Point(static_cast<float>(fixed), static_cast<float>(a), static_cast<float>(b)));
    }
  }
}

Cloud::Ptr makeFixture(const std::string& name) {
  Cloud::Ptr cloud(new Cloud);
  constexpr double s = 0.25;
  if (name == "RICH_CORNER") {
    appendGridPlane(cloud, 2, -2.0, -8.0, 8.0, -6.0, 6.0, s);
    appendGridPlane(cloud, 1, 3.0, -8.0, 4.0, -2.0, 4.0, s);
    appendGridPlane(cloud, 0, 4.0, -6.0, 3.0, -2.0, 4.0, s);
  } else if (name == "SINGLE_LARGE_PLANE") {
    appendGridPlane(cloud, 2, 0.0, -12.0, 12.0, -12.0, 12.0, s);
  } else if (name == "STRAIGHT_CORRIDOR") {
    appendGridPlane(cloud, 1, -2.0, -12.0, 12.0, -1.5, 2.0, s);
    appendGridPlane(cloud, 1, 2.0, -12.0, 12.0, -1.5, 2.0, s);
    appendGridPlane(cloud, 2, -1.5, -12.0, 12.0, -2.0, 2.0, s);
  } else if (name == "EXTRUDED_TUNNEL") {
    const int angular_samples = 64;
    for (double x = -12.0; x <= 12.0 + 1e-9; x += s) {
      for (int i = 0; i < angular_samples; ++i) {
        const double theta = 2.0 * M_PI * static_cast<double>(i) / angular_samples;
        cloud->push_back(Point(static_cast<float>(x),
            static_cast<float>(2.0 * std::cos(theta)),
            static_cast<float>(2.0 * std::sin(theta))));
      }
    }
  } else {
    throw std::runtime_error("unknown synthetic fixture: " + name);
  }
  finalize(cloud);
  if (cloud->size() < 100) throw std::runtime_error("synthetic fixture unexpectedly small");
  return cloud;
}

Vector6d truthPoseVector() {
  Vector6d truth;
  truth << 0.20, -0.15, 0.10,
      0.25 * M_PI / 180.0, -0.20 * M_PI / 180.0, 0.30 * M_PI / 180.0;
  return truth;
}

struct Timing {
  double hessian_ms = 0.0;
  double canonical_ms = 0.0;
  double rotation_transform_ms = 0.0;
  double normalization_ms = 0.0;
  double schur_ms = 0.0;
  double eigensolve_ms = 0.0;
  double physical_ms = 0.0;
  double total_ms = 0.0;
  int fallback_count = 0;
  int factor_failures = 0;
};

struct HessianOutput {
  bool finite = false;
  double score = std::numeric_limits<double>::quiet_NaN();
  double asymmetry = std::numeric_limits<double>::quiet_NaN();
  Matrix6d raw_score = Matrix6d::Constant(std::numeric_limits<double>::quiet_NaN());
  Matrix6d information = Matrix6d::Constant(std::numeric_limits<double>::quiet_NaN());
  Matrix6d physical_information = Matrix6d::Constant(std::numeric_limits<double>::quiet_NaN());
  Matrix6d normalized = Matrix6d::Constant(std::numeric_limits<double>::quiet_NaN());
  Matrix3d spatial_jacobian = Matrix3d::Constant(std::numeric_limits<double>::quiet_NaN());
  double jacobian_condition = std::numeric_limits<double>::infinity();
  bool coordinate_transform_valid = false;
  Timing timing;
};

HessianOutput extractAndNormalize(AuditedNdt& ndt, const Cloud::Ptr& source,
                                  const Eigen::Matrix4f& pose) {
  HessianOutput result;
  const auto total_start = Clock::now();
  const auto extract_start = Clock::now();
  Cloud transformed;
  pcl::transformPointCloud(*source, transformed, pose);
  Vector6d p = poseVector(pose);
  result.score = ndt.scoreHessian(transformed, p, &result.raw_score);
  result.timing.hessian_ms = std::chrono::duration<double, std::milli>(
      Clock::now() - extract_start).count();

  const auto canonical_start = Clock::now();
  const std::array<int, 6> permutation{{3, 4, 5, 0, 1, 2}};
  Matrix6d canonical;
  for (int r = 0; r < 6; ++r)
    for (int c = 0; c < 6; ++c)
      canonical(r, c) = result.raw_score(permutation[static_cast<std::size_t>(r)],
                                          permutation[static_cast<std::size_t>(c)]);
  result.asymmetry = (canonical - canonical.transpose()).norm();
  result.information = -0.5 * (canonical + canonical.transpose());
  result.timing.canonical_ms = std::chrono::duration<double, std::milli>(
      Clock::now() - canonical_start).count();

  const auto rotation_transform_start = Clock::now();
  const SpatialJacobian spatial = makeSpatialJacobian(p.tail<3>());
  result.spatial_jacobian = spatial.value;
  result.jacobian_condition = spatial.condition;
  if (spatial.valid) {
    Matrix6d euler_from_physical = Matrix6d::Identity();
    euler_from_physical.block<3, 3>(0, 0) = spatial.inverse;
    result.physical_information = euler_from_physical.transpose() *
        result.information * euler_from_physical;
    result.coordinate_transform_valid = result.physical_information.allFinite();
  }
  result.timing.rotation_transform_ms = std::chrono::duration<double, std::milli>(
      Clock::now() - rotation_transform_start).count();

  const auto normalization_start = Clock::now();
  Matrix6d physical_to_dimensionless = Matrix6d::Identity();
  physical_to_dimensionless(3, 3) = physical_to_dimensionless(4, 4) =
      physical_to_dimensionless(5, 5) = kResolution;
  if (result.coordinate_transform_valid)
    result.normalized = physical_to_dimensionless.transpose() *
        result.physical_information * physical_to_dimensionless;
  result.timing.normalization_ms = std::chrono::duration<double, std::milli>(
      Clock::now() - normalization_start).count();
  result.timing.total_ms = std::chrono::duration<double, std::milli>(
      Clock::now() - total_start).count();
  result.finite = std::isfinite(result.score) && result.raw_score.allFinite() &&
                  result.information.allFinite() && result.coordinate_transform_valid &&
                  result.normalized.allFinite();
  return result;
}

struct SolveResult {
  bool success = false;
  std::string method = "FAIL";
  double condition = std::numeric_limits<double>::infinity();
  Matrix3d x = Matrix3d::Zero();
};

double conditionEstimate(const Matrix3d& matrix) {
  Eigen::JacobiSVD<Matrix3d> svd(matrix);
  const Vector3d s = svd.singularValues();
  if (!s.allFinite() || s.minCoeff() <= 0.0)
    return std::numeric_limits<double>::infinity();
  return s.maxCoeff() / s.minCoeff();
}

SolveResult solveStable(const Matrix3d& matrix, const Matrix3d& rhs) {
  SolveResult result;
  result.condition = conditionEstimate(matrix);
  Eigen::LDLT<Matrix3d> ldlt;
  ldlt.compute(matrix);
  if (ldlt.info() == Eigen::Success && ldlt.vectorD().allFinite()) {
    const Vector3d diagonal = ldlt.vectorD().cwiseAbs();
    const double max_d = diagonal.maxCoeff();
    if (max_d > 0.0 && diagonal.minCoeff() > std::max(1e-14 * max_d, 1e-18)) {
      result.x = ldlt.solve(rhs);
      const double residual = (matrix * result.x - rhs).norm() /
                              std::max(rhs.norm(), 1e-18);
      if (result.x.allFinite() && residual <= 1e-8) {
        result.success = true;
        result.method = "LDLT";
        return result;
      }
    }
  }
  Eigen::ColPivHouseholderQR<Matrix3d> qr(matrix);
  qr.setThreshold(1e-12);
  if (qr.rank() == 3) {
    result.x = qr.solve(rhs);
    const double residual = (matrix * result.x - rhs).norm() /
                            std::max(rhs.norm(), 1e-18);
    if (result.x.allFinite() && residual <= 1e-8) {
      result.success = true;
      result.method = "QR";
      return result;
    }
  }
  return result;
}

struct Spectrum3 {
  bool finite = false;
  Vector3d values = Vector3d::Constant(std::numeric_limits<double>::quiet_NaN());
  Matrix3d vectors = Matrix3d::Constant(std::numeric_limits<double>::quiet_NaN());
};

Spectrum3 diagonalize(const Matrix3d& input) {
  Spectrum3 result;
  const Matrix3d symmetric = 0.5 * (input + input.transpose());
  Eigen::SelfAdjointEigenSolver<Matrix3d> solver(symmetric);
  if (solver.info() != Eigen::Success) return result;
  result.values = solver.eigenvalues();
  result.vectors = solver.eigenvectors();
  result.finite = result.values.allFinite() && result.vectors.allFinite();
  return result;
}

Spectrum3 diagonalize6(const Matrix6d& input,
                       Eigen::Matrix<double, 6, 1>* values,
                       Eigen::Matrix<double, 6, 6>* vectors) {
  const Matrix6d symmetric = 0.5 * (input + input.transpose());
  Eigen::SelfAdjointEigenSolver<Matrix6d> solver(symmetric);
  if (solver.info() != Eigen::Success) return Spectrum3();
  *values = solver.eigenvalues();
  *vectors = solver.eigenvectors();
  Spectrum3 marker;
  marker.finite = values->allFinite() && vectors->allFinite();
  return marker;
}

std::string axisName(int axis, bool rotation) {
  static const std::array<std::string, 3> trans{{"x", "y", "z"}};
  static const std::array<std::string, 3> rot{{"rotation_x", "rotation_y", "rotation_z"}};
  return rotation ? rot[static_cast<std::size_t>(axis)] :
                    trans[static_cast<std::size_t>(axis)];
}

void writeJacobianValidation(std::ofstream& output, const std::string& case_id,
                             const Vector3d& euler) {
  const JacobianValidation validation = validateSpatialJacobian(euler);
  for (int axis = 0; axis < 3; ++axis) {
    writeRow(output, {case_id, number(euler.x()), number(euler.y()), number(euler.z()),
        number(validation.jacobian.condition), axisName(axis, true),
        flatten(validation.jacobian.value.col(axis)),
        flatten(validation.finite_difference[static_cast<std::size_t>(axis)]),
        number(validation.errors[static_cast<std::size_t>(axis)]),
        validation.pass ? "1" : "0"});
  }
}

struct MethodComponent {
  std::string method;
  std::string component;
  bool finite = false;
  Vector3d values = Vector3d::Constant(std::numeric_limits<double>::quiet_NaN());
  Matrix3d vectors = Matrix3d::Constant(std::numeric_limits<double>::quiet_NaN());
  Matrix3d aligned_basis = Matrix3d::Constant(std::numeric_limits<double>::quiet_NaN());
  Vector3d aligned_values = Vector3d::Constant(std::numeric_limits<double>::quiet_NaN());
  std::array<int, 3> original_indices{{-1, -1, -1}};
  Eigen::Matrix<double, 6, 1> raw_values =
      Eigen::Matrix<double, 6, 1>::Constant(std::numeric_limits<double>::quiet_NaN());
  Matrix3d contribution = Matrix3d::Constant(std::numeric_limits<double>::quiet_NaN());
  Vector3d weakest = Vector3d::Constant(std::numeric_limits<double>::quiet_NaN());
  std::string weakest_axis = "UNAVAILABLE";
  std::string axis_to_mode = "UNAVAILABLE";
  double agreement = std::numeric_limits<double>::quiet_NaN();
  int expected_dim = 0;
  double selected_rotation_fraction = std::numeric_limits<double>::quiet_NaN();
};

Matrix3d expectedBasis(const std::array<bool, 3>& axes, int* dimension) {
  Matrix3d basis = Matrix3d::Zero();
  *dimension = 0;
  for (int i = 0; i < 3; ++i)
    if (axes[static_cast<std::size_t>(i)])
      basis.col((*dimension)++) = Vector3d::Unit(i);
  return basis;
}

double subspaceAgreement(const Matrix3d& expected, int dimension,
                         const Matrix3d& estimate, int estimate_dimension) {
  if (dimension == 0 || estimate_dimension < dimension)
    return std::numeric_limits<double>::quiet_NaN();
  Matrix3d overlap = Matrix3d::Zero();
  overlap.topLeftCorner(dimension, dimension) =
      expected.leftCols(dimension).transpose() * estimate.leftCols(dimension);
  Eigen::JacobiSVD<Matrix3d> svd(overlap);
  if (!svd.singularValues().allFinite())
    return std::numeric_limits<double>::quiet_NaN();
  return std::min(1.0, svd.singularValues()(dimension - 1));
}

bool alignEigenBasisToAxes(const Matrix3d& raw_basis, Matrix3d* aligned_basis,
                           std::array<int, 3>* original_indices) {
  aligned_basis->setZero();
  original_indices->fill(-1);
  std::array<bool, 3> used{{false, false, false}};
  for (int axis = 0; axis < 3; ++axis) {
    double best_score = -1.0;
    int best_index = -1;
    for (int candidate = 0; candidate < 3; ++candidate) {
      if (used[static_cast<std::size_t>(candidate)]) continue;
      const double score = std::abs(raw_basis(axis, candidate));
      if (score > best_score) {
        best_score = score;
        best_index = candidate;
      }
    }
    if (best_index < 0) return false;
    used[static_cast<std::size_t>(best_index)] = true;
    (*original_indices)[static_cast<std::size_t>(axis)] = best_index;
    aligned_basis->col(axis) = raw_basis.col(best_index);
    if ((*aligned_basis)(axis, axis) < 0.0)
      aligned_basis->col(axis) *= -1.0;
  }
  return aligned_basis->allFinite();
}

Matrix3d projectRawBasis(const Eigen::Matrix<double, 6, 6>& full_vectors,
                         bool rotation, int dimension, int* selected,
                         std::string* selected_modes) {
  Matrix3d basis = Matrix3d::Zero();
  *selected = 0;
  std::ostringstream mode_ids;
  const int offset = rotation ? 0 : 3;
  for (int col = 0; col < 6 && *selected < std::max(dimension, 1); ++col) {
    const double rot_energy = full_vectors.block<3, 1>(0, col).squaredNorm();
    const double trans_energy = full_vectors.block<3, 1>(3, col).squaredNorm();
    const bool dominates = rotation ? rot_energy >= trans_energy : trans_energy >= rot_energy;
    if (!dominates) continue;
    Vector3d projected = full_vectors.block<3, 1>(offset, col);
    for (int prior = 0; prior < *selected; ++prior)
      projected -= basis.col(prior).dot(projected) * basis.col(prior);
    const double norm = projected.norm();
    if (norm < 1e-10) continue;
    basis.col(*selected) = projected / norm;
    if (*selected) mode_ids << '|';
    mode_ids << col;
    ++(*selected);
  }
  *selected_modes = mode_ids.str();
  return basis;
}

MethodComponent characterize(const std::string& method, const std::string& component,
                             const Matrix3d& matrix,
                             const std::array<bool, 3>& expected_axes,
                             const Eigen::Matrix<double, 6, 1>* raw_values = nullptr,
                             const Eigen::Matrix<double, 6, 6>* raw_vectors = nullptr,
                             std::string* raw_selected_modes = nullptr) {
  MethodComponent result;
  result.method = method;
  result.component = component;
  result.expected_dim = countAxes(expected_axes);
  const bool rotation = component == "ROTATION";
  Matrix3d expected;
  int expected_dim = 0;
  expected = expectedBasis(expected_axes, &expected_dim);
  if (method == "RAW6") {
    if (!raw_values || !raw_vectors) return result;
    result.raw_values = *raw_values;
    int selected = 0;
    std::string selected_modes;
    result.vectors = projectRawBasis(*raw_vectors, rotation, expected_dim,
                                     &selected, &selected_modes);
    if (raw_selected_modes) *raw_selected_modes = selected_modes;
    // RAW6 is one coupled spectrum. Report the three smallest full-system
    // eigenvalues; component-projected directions are descriptive only.
    result.values = raw_values->head<3>();
    result.finite = raw_values->allFinite() && raw_vectors->allFinite() && selected > 0;
    result.agreement = subspaceAgreement(expected, expected_dim,
                                        result.vectors, selected);
  } else {
    const Spectrum3 spectrum = diagonalize(matrix);
    result.finite = spectrum.finite;
    if (!spectrum.finite) return result;
    result.values = spectrum.values;
    result.vectors = spectrum.vectors;
    result.agreement = subspaceAgreement(expected, expected_dim,
                                        result.vectors, 3);
    if (!alignEigenBasisToAxes(result.vectors, &result.aligned_basis,
                               &result.original_indices)) {
      result.finite = false;
      return result;
    }
    for (int axis = 0; axis < 3; ++axis)
      result.aligned_values(axis) = result.values(
          result.original_indices[static_cast<std::size_t>(axis)]);
    result.contribution = result.aligned_basis.cwiseProduct(result.aligned_basis);
    Eigen::Index weakest_axis = 0;
    result.aligned_values.minCoeff(&weakest_axis);
    result.weakest_axis = axisName(static_cast<int>(weakest_axis), rotation);
    result.weakest = result.aligned_basis.col(weakest_axis);
    std::ostringstream mapping;
    for (int axis = 0; axis < 3; ++axis) {
      if (axis) mapping << '|';
      mapping << axisName(axis, rotation) << ':'
              << result.original_indices[static_cast<std::size_t>(axis)];
    }
    result.axis_to_mode = mapping.str();
    return result;
  }
  if (!result.finite) return result;
  result.weakest = result.vectors.col(0);
  result.aligned_basis = result.vectors;
  result.contribution = result.vectors.cwiseProduct(result.vectors);
  Eigen::Index dominant = 0;
  result.weakest.cwiseAbs().maxCoeff(&dominant);
  result.weakest_axis = axisName(static_cast<int>(dominant), rotation);
  std::array<int, 3> axis_match{{-1, -1, -1}};
  std::array<bool, 3> used{{false, false, false}};
  for (int axis = 0; axis < 3; ++axis) {
    double best = -1.0;
    for (int mode = 0; mode < 3; ++mode) {
      const double value = std::abs(result.vectors.col(mode)(axis));
      if (!used[static_cast<std::size_t>(mode)] && value > best) {
        best = value;
        axis_match[static_cast<std::size_t>(axis)] = mode;
      }
    }
    const int mode = axis_match[static_cast<std::size_t>(axis)];
    if (mode >= 0) used[static_cast<std::size_t>(mode)] = true;
  }
  std::ostringstream mapping;
  for (int axis = 0; axis < 3; ++axis) {
    if (axis) mapping << '|';
    mapping << axisName(axis, rotation) << ':' << axis_match[static_cast<std::size_t>(axis)];
  }
  result.axis_to_mode = mapping.str();
  return result;
}

struct Analysis {
  bool finite = false;
  Eigen::Matrix<double, 6, 1> raw_values =
      Eigen::Matrix<double, 6, 1>::Constant(std::numeric_limits<double>::quiet_NaN());
  Eigen::Matrix<double, 6, 6> raw_vectors =
      Eigen::Matrix<double, 6, 6>::Constant(std::numeric_limits<double>::quiet_NaN());
  Matrix3d hrr = Matrix3d::Zero(), htt = Matrix3d::Zero();
  Matrix3d sr = Matrix3d::Constant(std::numeric_limits<double>::quiet_NaN());
  Matrix3d st = Matrix3d::Constant(std::numeric_limits<double>::quiet_NaN());
  std::string solve_rot = "NOT_RUN", solve_trans = "NOT_RUN";
  double cond_rr = std::numeric_limits<double>::quiet_NaN();
  double cond_tt = std::numeric_limits<double>::quiet_NaN();
  std::array<MethodComponent, 6> results;
  Timing timing;
};

Analysis analyzeHessian(HessianOutput* hessian,
                        const std::array<bool, 3>& weak_translation,
                        const std::array<bool, 3>& weak_rotation) {
  Analysis result;
  result.timing = hessian->timing;
  if (!hessian->finite) return result;
  result.hrr = hessian->normalized.block<3, 3>(0, 0);
  const Matrix3d hrt = hessian->normalized.block<3, 3>(0, 3);
  const Matrix3d htr = hessian->normalized.block<3, 3>(3, 0);
  result.htt = hessian->normalized.block<3, 3>(3, 3);
  result.cond_rr = conditionEstimate(result.hrr);
  result.cond_tt = conditionEstimate(result.htt);

  const auto schur_start = Clock::now();
  const SolveResult solve_tt = solveStable(result.htt, htr);
  const SolveResult solve_rr = solveStable(result.hrr, hrt);
  result.solve_rot = solve_tt.method;
  result.solve_trans = solve_rr.method;
  if (solve_tt.success) result.sr = result.hrr - hrt * solve_tt.x;
  else ++result.timing.factor_failures;
  if (solve_rr.success) result.st = result.htt - htr * solve_rr.x;
  else ++result.timing.factor_failures;
  if (solve_tt.method == "QR") ++result.timing.fallback_count;
  if (solve_rr.method == "QR") ++result.timing.fallback_count;
  result.sr = 0.5 * (result.sr + result.sr.transpose());
  result.st = 0.5 * (result.st + result.st.transpose());
  result.timing.schur_ms = std::chrono::duration<double, std::milli>(
      Clock::now() - schur_start).count();

  const auto eig_start = Clock::now();
  const Spectrum3 block_r = diagonalize(result.hrr);
  const Spectrum3 block_t = diagonalize(result.htt);
  const Spectrum3 schur_r = diagonalize(result.sr);
  const Spectrum3 schur_t = diagonalize(result.st);
  const Spectrum3 raw_marker = diagonalize6(hessian->normalized,
      &result.raw_values, &result.raw_vectors);
  result.timing.eigensolve_ms = std::chrono::duration<double, std::milli>(
      Clock::now() - eig_start).count();

  const auto physical_start = Clock::now();
  std::string raw_t_modes, raw_r_modes;
  result.results[0] = characterize("RAW6", "TRANSLATION", Matrix3d::Zero(),
      weak_translation, &result.raw_values, &result.raw_vectors, &raw_t_modes);
  result.results[1] = characterize("RAW6", "ROTATION", Matrix3d::Zero(),
      weak_rotation, &result.raw_values, &result.raw_vectors, &raw_r_modes);
  // RAW6 is a coupled 6D spectrum. Preserve its actual spectrum in raw_values;
  // the block projection vectors are descriptive only, not a decoupled EVD.
  if (block_t.finite) result.results[2] = characterize("BLOCK", "TRANSLATION",
      result.htt, weak_translation);
  if (block_r.finite) result.results[3] = characterize("BLOCK", "ROTATION",
      result.hrr, weak_rotation);
  if (schur_t.finite) result.results[4] = characterize("SCHUR", "TRANSLATION",
      result.st, weak_translation);
  if (schur_r.finite) result.results[5] = characterize("SCHUR", "ROTATION",
      result.sr, weak_rotation);
  result.timing.physical_ms = std::chrono::duration<double, std::milli>(
      Clock::now() - physical_start).count();
  result.timing.total_ms = hessian->timing.total_ms + result.timing.schur_ms +
      result.timing.eigensolve_ms + result.timing.physical_ms;
  result.finite = raw_marker.finite && block_t.finite && block_r.finite &&
      std::all_of(result.results.begin(), result.results.end(),
                  [](const MethodComponent& item) { return item.finite; });
  return result;
}

std::vector<std::string> methodComponentHeader() {
  return {"fixture", "perturbation_id", "component", "method",
      "expected_weak_axes", "expected_subspace_dimension", "weakest_physical_axis",
      "direction_or_subspace_agreement", "lambda1", "lambda2", "lambda3",
      "raw6_lambda1", "raw6_lambda2", "raw6_lambda3", "raw6_all_six_eigenvalues",
      "weakest_eigenvector_xyz_or_rotation_xyz", "eigenvectors_flat_row_major",
      "aligned_basis_flat_row_major", "aligned_eigenvalues_axis_order",
      "axis_contribution_squared_flat_row_major", "axis_to_eigenmode_greedy",
      "finite", "rotation_coordinate_transform_condition",
      "rotation_coordinate_transform_valid", "rotation_coordinate_transform_failure_count",
      "analytic_score", "analytic_hessian_asymmetry_frobenius",
      "registration_converged", "registration_iterations", "registration_ms",
      "hessian_extraction_ms", "canonicalization_ms", "rotation_coordinate_transform_ms",
      "normalization_ms",
      "schur_ms", "eigensolve_ms", "physical_axis_ms", "analyzer_total_ms",
      "schur_rotation_solve", "schur_translation_solve", "cond_hrr", "cond_htt",
      "factorization_fallback_count", "factorization_failure_count"};
}

std::vector<std::string> makeResultRow(const std::string& fixture,
    const std::string& perturbation, const MethodComponent& item,
    const std::array<bool, 3>& weak_t, const std::array<bool, 3>& weak_r,
    const HessianOutput& hessian, const Analysis& analysis,
    bool converged, int iterations, double registration_ms) {
  const bool rotation = item.component == "ROTATION";
  const auto& axes = rotation ? weak_r : weak_t;
  std::vector<std::string> row = {fixture, perturbation, item.component, item.method,
      [&axes, rotation]() {
        std::string value;
        for (int i = 0; i < 3; ++i) if (axes[static_cast<std::size_t>(i)]) {
          if (!value.empty()) value += '|';
          value += axisName(i, rotation);
        }
        return value.empty() ? "none" : value;
      }(),
      std::to_string(item.expected_dim), item.weakest_axis,
      number(item.agreement), number(item.values(0)), number(item.values(1)),
      number(item.values(2)), number(analysis.raw_values(0)),
      number(analysis.raw_values(1)), number(analysis.raw_values(2)),
      flattenVector6(analysis.raw_values), flatten(item.weakest),
      flatten(item.vectors), flatten(item.aligned_basis), flatten(item.aligned_values),
      flatten(item.contribution), item.axis_to_mode,
      item.finite ? "1" : "0", number(hessian.jacobian_condition),
      hessian.coordinate_transform_valid ? "1" : "0",
      hessian.coordinate_transform_valid ? "0" : "1",
      number(hessian.score), number(hessian.asymmetry),
      converged ? "1" : "0", std::to_string(iterations), number(registration_ms),
      number(hessian.timing.hessian_ms), number(hessian.timing.canonical_ms),
      number(hessian.timing.rotation_transform_ms),
      number(hessian.timing.normalization_ms), number(analysis.timing.schur_ms),
      number(analysis.timing.eigensolve_ms), number(analysis.timing.physical_ms),
      number(analysis.timing.total_ms), analysis.solve_rot, analysis.solve_trans,
      number(analysis.cond_rr), number(analysis.cond_tt),
      std::to_string(analysis.timing.fallback_count),
      std::to_string(analysis.timing.factor_failures)};
  return row;
}

void writeSynthetic(const std::string& output_dir,
                    const std::vector<FixtureExpectation>& expectations,
                    const std::vector<Perturbation>& perturbations) {
  const std::string results_path = output_dir + "/synthetic_results.csv";
  const std::string comparison_path = output_dir + "/raw_block_schur_comparison.csv";
  const std::string timings_path = output_dir + "/timing_samples.csv";
  const std::string validation_path =
      output_dir + "/coordinate_transform_validation_synthetic.csv";
  std::ofstream results(results_path), comparison(comparison_path), timings(timings_path),
      validation(validation_path);
  if (!results || !comparison || !timings || !validation)
    throw std::runtime_error("cannot create synthetic output files");
  writeHeader(validation, {"case_id", "rx", "ry", "rz", "jacobian_condition",
      "axis", "analytic_jacobian_xyz", "fd_jacobian_xyz", "error_norm", "pass"});
  writeJacobianValidation(validation, "IDENTITY_NEAR", Vector3d(0.01, -0.02, 0.03));
  writeHeader(results, methodComponentHeader());
  writeHeader(comparison, {"fixture", "perturbation_id", "component", "expected_weak_axes",
      "expected_subspace_dimension", "raw6_agreement", "block_agreement", "schur_agreement",
      "raw6_weakest_axis", "block_weakest_axis", "schur_weakest_axis",
      "raw6_lambda1", "raw6_lambda2", "raw6_lambda3", "block_lambda1", "block_lambda2",
      "block_lambda3", "schur_lambda1", "schur_lambda2", "schur_lambda3",
      "raw6_finite", "block_finite", "schur_finite"});
  writeHeader(timings, {"dataset", "fixture_or_transaction", "perturbation_id", "registration_ms",
      "hessian_extraction_ms", "canonicalization_ms", "rotation_coordinate_transform_ms",
      "normalization_ms", "schur_ms",
      "eigensolve_ms", "physical_axis_ms", "analyzer_total_ms", "fallback_count",
      "factorization_failure_count"});

  const Vector6d truth = truthPoseVector();
  for (const FixtureExpectation& expected : expectations) {
    const Cloud::Ptr target = makeFixture(expected.name);
    const Eigen::Matrix4f true_pose = poseFromVector(truth);
    const Cloud::Ptr source = transformCloud(target, true_pose.inverse());
    AuditedNdt ndt;
    ndt.setInputTarget(target);
    ndt.setResolution(kResolution);
    ndt.setStepSize(0.08);
    ndt.setTransformationEpsilon(0.001);
    ndt.setMaximumIterations(40);
    for (const Perturbation& perturbation : perturbations) {
      const Eigen::Matrix4f initial = poseFromVector(truth + perturbation.delta);
      ndt.setInputSource(source);
      Cloud aligned;
      const auto reg_start = Clock::now();
      ndt.align(aligned, initial);
      const double registration_ms = std::chrono::duration<double, std::milli>(
          Clock::now() - reg_start).count();
      const bool converged = ndt.hasConverged();
      const int iterations = ndt.getFinalNumIteration();
      const Eigen::Matrix4f final_pose = ndt.getFinalTransformation();
      HessianOutput hessian;
      Analysis analysis;
      if (converged) {
        writeJacobianValidation(validation, expected.name + "_" + perturbation.id,
                                poseVector(final_pose).tail<3>());
        hessian = extractAndNormalize(ndt, source, final_pose);
        analysis = analyzeHessian(&hessian, expected.weak_translation,
                                  expected.weak_rotation);
      }
      for (const MethodComponent& item : analysis.results)
        writeRow(results, makeResultRow(expected.name, perturbation.id, item,
            expected.weak_translation, expected.weak_rotation, hessian,
            analysis, converged, iterations, registration_ms));
      for (const std::string component : {"TRANSLATION", "ROTATION"}) {
        const int raw_index = component == "TRANSLATION" ? 0 : 1;
        const int block_index = component == "TRANSLATION" ? 2 : 3;
        const int schur_index = component == "TRANSLATION" ? 4 : 5;
        const MethodComponent& raw = analysis.results[static_cast<std::size_t>(raw_index)];
        const MethodComponent& block = analysis.results[static_cast<std::size_t>(block_index)];
        const MethodComponent& schur = analysis.results[static_cast<std::size_t>(schur_index)];
        const auto& axes = component == "TRANSLATION" ? expected.weak_translation : expected.weak_rotation;
        std::string axis_text;
        for (int i = 0; i < 3; ++i) if (axes[static_cast<std::size_t>(i)]) {
          if (!axis_text.empty()) axis_text += '|';
          axis_text += axisName(i, component == "ROTATION");
        }
        if (axis_text.empty()) axis_text = "none";
        writeRow(comparison, {expected.name, perturbation.id, component, axis_text,
            std::to_string(countAxes(axes)), number(raw.agreement), number(block.agreement),
            number(schur.agreement), raw.weakest_axis, block.weakest_axis,
            schur.weakest_axis, number(analysis.raw_values(0)),
            number(analysis.raw_values(1)), number(analysis.raw_values(2)),
            number(block.values(0)), number(block.values(1)), number(block.values(2)),
            number(schur.values(0)), number(schur.values(1)), number(schur.values(2)),
            raw.finite ? "1" : "0", block.finite ? "1" : "0", schur.finite ? "1" : "0"});
      }
      writeRow(timings, {"SYNTHETIC", expected.name, perturbation.id,
          number(registration_ms), number(hessian.timing.hessian_ms),
          number(hessian.timing.canonical_ms),
          number(hessian.timing.rotation_transform_ms),
          number(hessian.timing.normalization_ms),
          number(analysis.timing.schur_ms), number(analysis.timing.eigensolve_ms),
          number(analysis.timing.physical_ms), number(analysis.timing.total_ms),
          std::to_string(analysis.timing.fallback_count),
          std::to_string(analysis.timing.factor_failures)});
      std::cout << "SYNTHETIC " << expected.name << ' ' << perturbation.id
                << " converged=" << converged << " iteration=" << iterations
                << " registration_ms=" << registration_ms
                << " analyzer_ms=" << analysis.timing.total_ms << std::endl;
    }
  }
}

struct ScanRecord {
  std::uint64_t transaction = 0;
  std::uint64_t stamp_ns = 0;
  std::uint64_t byte_offset = 0;
  std::uint64_t point_count = 0;
  std::uint64_t expected_cloud_hash = 0;
  double time_s = 0.0;
  Eigen::Matrix4f predicted = Eigen::Matrix4f::Identity();
  std::string reason;
};

std::vector<ScanRecord> readScans(const std::string& path) {
  const CsvTable table = readCsv(path);
  std::vector<ScanRecord> scans;
  for (const auto& row : table.rows) {
    ScanRecord scan;
    scan.transaction = parseU64(table.get(row, "transaction_id"));
    scan.stamp_ns = parseU64(table.get(row, "stamp_ns"));
    scan.time_s = parseDouble(table.get(row, "time_s"));
    scan.byte_offset = parseU64(table.get(row, "cloud_byte_offset"));
    scan.point_count = parseU64(table.get(row, "cloud_point_count"));
    scan.expected_cloud_hash = parseU64(table.get(row, "ndt_source_cloud_hash"));
    scan.predicted = poseFromQuaternion(
        parseDouble(table.get(row, "predicted_x")),
        parseDouble(table.get(row, "predicted_y")),
        parseDouble(table.get(row, "predicted_z")),
        parseDouble(table.get(row, "predicted_qx")),
        parseDouble(table.get(row, "predicted_qy")),
        parseDouble(table.get(row, "predicted_qz")),
        parseDouble(table.get(row, "predicted_qw")));
    scans.push_back(scan);
  }
  if (scans.size() != 4127) throw std::runtime_error("frozen scan count mismatch");
  return scans;
}

std::map<std::uint64_t, std::vector<std::string>> readBaselineRows(const std::string& path) {
  const CsvTable table = readCsv(path);
  std::map<std::uint64_t, std::vector<std::string>> rows;
  for (const auto& row : table.rows)
    rows[parseU64(table.get(row, "transaction_id"))] = row;
  if (rows.size() != 4127) throw std::runtime_error("frozen baseline row count mismatch");
  return rows;
}

Cloud::Ptr loadPackedCloud(const std::string& path, const ScanRecord& scan) {
  std::ifstream input(path, std::ios::binary);
  if (!input) throw std::runtime_error("cannot open frozen packed XYZ input");
  input.seekg(static_cast<std::streamoff>(scan.byte_offset));
  if (!input) throw std::runtime_error("cannot seek packed XYZ input");
  Cloud::Ptr cloud(new Cloud);
  cloud->reserve(static_cast<std::size_t>(scan.point_count));
  for (std::uint64_t i = 0; i < scan.point_count; ++i) {
    float xyz[3];
    input.read(reinterpret_cast<char*>(xyz), sizeof(xyz));
    if (input.gcount() != static_cast<std::streamsize>(sizeof(xyz)))
      throw std::runtime_error("truncated packed XYZ input");
    cloud->push_back(Point(xyz[0], xyz[1], xyz[2]));
  }
  finalize(cloud, false);
  return cloud;
}

std::vector<std::size_t> chooseFloorFrames(const std::vector<ScanRecord>& scans,
                                          std::map<std::size_t, std::string>* reasons) {
  std::set<std::size_t> selected;
  constexpr int uniform_count = 120;
  for (int k = 0; k < uniform_count; ++k) {
    const auto index = static_cast<std::size_t>(std::llround(
        static_cast<double>(k) * static_cast<double>(scans.size() - 1) /
        static_cast<double>(uniform_count - 1)));
    selected.insert(index);
    (*reasons)[index] = "UNIFORM_120";
  }
  const std::array<double, 12> window_targets{{
      84.0, 87.0, 90.0, 93.0, 95.0,
      151.0, 153.5, 156.0, 158.0,
      286.0, 289.0, 292.0}};
  for (const double target : window_targets) {
    auto closest = std::min_element(scans.begin(), scans.end(), [target](const ScanRecord& a, const ScanRecord& b) {
      return std::abs(a.time_s - target) < std::abs(b.time_s - target);
    });
    const std::size_t index = static_cast<std::size_t>(std::distance(scans.begin(), closest));
    selected.insert(index);
    const std::string label = target <= 95.0 ? "WINDOW_84_95" :
        (target <= 158.0 ? "WINDOW_151_158" : "WINDOW_286_292");
    if ((*reasons)[index].empty()) (*reasons)[index] = label;
    else if ((*reasons)[index].find(label) == std::string::npos)
      (*reasons)[index] += "|" + label;
  }
  if (selected.size() > 150) throw std::runtime_error("Floor01 frame selection exceeds 150");
  return std::vector<std::size_t>(selected.begin(), selected.end());
}

void writeFloor01(const std::string& map_path, const std::string& packed_path,
                  const std::string& scans_path, const std::string& baseline_path,
                  const std::string& output_dir) {
  const std::vector<ScanRecord> scans = readScans(scans_path);
  const auto baseline = readBaselineRows(baseline_path);
  const CsvTable baseline_table = readCsv(baseline_path);
  const Cloud::Ptr target = loadTarget(map_path);
  if (target->size() != 549606) throw std::runtime_error("frozen map target preprocessing point-count mismatch");
  AuditedNdt ndt;
  ndt.setInputTarget(target);
  ndt.setResolution(kResolution);
  ndt.setStepSize(0.08);
  ndt.setTransformationEpsilon(0.001);
  ndt.setMaximumIterations(40);

  std::map<std::size_t, std::string> reasons;
  const std::vector<std::size_t> selected = chooseFloorFrames(scans, &reasons);
  std::ofstream manifest(output_dir + "/floor01_manifest.csv");
  std::ofstream results(output_dir + "/floor01_uobs.csv");
  std::ofstream comparison(output_dir + "/floor01_comparison.csv");
  std::ofstream timings(output_dir + "/timing_samples_floor01.csv");
  std::ofstream validation(output_dir + "/coordinate_transform_validation_floor01.csv");
  if (!manifest || !results || !comparison || !timings || !validation)
    throw std::runtime_error("cannot create Floor01 outputs");
  writeHeader(validation, {"case_id", "rx", "ry", "rz", "jacobian_condition",
      "axis", "analytic_jacobian_xyz", "fd_jacobian_xyz", "error_norm", "pass"});
  writeJacobianValidation(validation, "IDENTITY_NEAR", Vector3d(0.01, -0.02, 0.03));
  writeHeader(manifest, {"transaction_id", "frame_index_zero_based", "stamp_ns", "time_s",
      "selection_reason", "source_point_count", "source_cloud_hash_expected",
      "source_cloud_hash_actual", "source_hash_match", "start_pose_saved_predictor_xyz_q_xyzw",
      "single_start_converged", "iterations", "fitness", "transformation_probability",
      "registration_ms", "baseline_translation_delta_m", "baseline_rotation_delta_deg",
      "baseline_fitness_delta", "baseline_iteration_match", "final_pose_matrix16",
      "map_sha256_expected", "target_point_count", "rotation_coordinate_transform_condition",
      "rotation_coordinate_transform_valid", "rotation_coordinate_transform_failure_count"});
  writeHeader(results, {"transaction_id", "time_s", "component", "method",
      "weakest_physical_axis", "lambda1", "lambda2", "lambda3",
      "raw6_all_six_eigenvalues", "weakest_eigenvector_xyz_or_rotation_xyz",
      "eigenvectors_flat_row_major", "aligned_basis_flat_row_major",
      "aligned_eigenvalues_axis_order", "axis_contribution_squared_flat_row_major",
      "axis_to_eigenmode_greedy", "finite", "rotation_coordinate_transform_condition",
      "rotation_coordinate_transform_valid", "rotation_coordinate_transform_failure_count",
      "analytic_score",
      "analytic_hessian_asymmetry_frobenius", "ndt_converged", "ndt_iterations",
      "ndt_fitness", "ndt_probability", "registration_ms", "hessian_extraction_ms",
      "canonicalization_ms", "rotation_coordinate_transform_ms", "normalization_ms",
      "schur_ms", "eigensolve_ms",
      "physical_axis_ms", "analyzer_total_ms", "schur_rotation_solve",
      "schur_translation_solve", "cond_hrr", "cond_htt", "factorization_fallback_count",
      "factorization_failure_count", "h_score_raw_flat_row_major",
      "h_information_euler_canonical_flat_row_major",
      "h_rotation_coordinate_jacobian_flat_row_major",
      "h_information_physical_flat_row_major", "h_normalized_dimensionless_flat_row_major",
      "h_rr_flat_row_major", "h_rt_flat_row_major", "h_tr_flat_row_major",
      "h_tt_flat_row_major", "s_rotation_flat_row_major", "s_translation_flat_row_major"});
  writeHeader(comparison, {"transaction_id", "time_s", "component", "raw6_weakest_axis",
      "block_weakest_axis", "schur_weakest_axis", "raw6_lambda1", "raw6_lambda2",
      "raw6_lambda3", "block_lambda1", "block_lambda2", "block_lambda3",
      "schur_lambda1", "schur_lambda2", "schur_lambda3", "raw6_finite",
      "block_finite", "schur_finite"});
  writeHeader(timings, {"dataset", "fixture_or_transaction", "perturbation_id", "registration_ms",
      "hessian_extraction_ms", "canonicalization_ms", "rotation_coordinate_transform_ms",
      "normalization_ms", "schur_ms",
      "eigensolve_ms", "physical_axis_ms", "analyzer_total_ms", "fallback_count",
      "factorization_failure_count"});

  for (const std::size_t index : selected) {
    const ScanRecord& scan = scans[index];
    const auto baseline_row = baseline.find(scan.transaction);
    if (baseline_row == baseline.end()) throw std::runtime_error("missing baseline frame");
    // Use the frozen baseline replay's scan-start predictor, not a later mode's pose.
    const Eigen::Matrix4f start = poseFromQuaternion(
        parseDouble(baseline_table.get(baseline_row->second, "predicted_lidar_x")),
        parseDouble(baseline_table.get(baseline_row->second, "predicted_lidar_y")),
        parseDouble(baseline_table.get(baseline_row->second, "predicted_lidar_z")),
        parseDouble(baseline_table.get(baseline_row->second, "predicted_lidar_qx")),
        parseDouble(baseline_table.get(baseline_row->second, "predicted_lidar_qy")),
        parseDouble(baseline_table.get(baseline_row->second, "predicted_lidar_qz")),
        parseDouble(baseline_table.get(baseline_row->second, "predicted_lidar_qw")));
    const Cloud::Ptr raw = loadPackedCloud(packed_path, scan);
    const Cloud::Ptr source = preprocessSource(raw);
    const std::uint64_t actual_hash = sourceCloudHash(source);
    const bool hash_match = actual_hash == scan.expected_cloud_hash;
    if (!hash_match) throw std::runtime_error("Floor01 preprocessed source hash mismatch at tx=" + integer(scan.transaction));
    ndt.setInputSource(source);
    Cloud aligned;
    const auto reg_start = Clock::now();
    ndt.align(aligned, start);
    const double registration_ms = std::chrono::duration<double, std::milli>(Clock::now() - reg_start).count();
    const bool converged = ndt.hasConverged();
    const int iterations = ndt.getFinalNumIteration();
    const Eigen::Matrix4f final_pose = ndt.getFinalTransformation();
    const double fitness = ndt.getFitnessScore();
    const double probability = ndt.getTransformationProbability();
    if (!converged) throw std::runtime_error("sampled Floor01 single-start NDT did not converge at tx=" + integer(scan.transaction));

    writeJacobianValidation(validation, "FLOOR01_TX_" + integer(scan.transaction),
                            poseVector(final_pose).tail<3>());
    HessianOutput hessian = extractAndNormalize(ndt, source, final_pose);
    const std::array<bool, 3> no_expected{{false, false, false}};
    Analysis analysis = analyzeHessian(&hessian, no_expected, no_expected);

    auto field = [&baseline_table, &baseline_row](const std::string& name) {
      return parseDouble(baseline_table.get(baseline_row->second, name));
    };
    const Eigen::Matrix4f saved_raw = poseFromQuaternion(
        field("raw_lidar_x"), field("raw_lidar_y"), field("raw_lidar_z"),
        field("raw_lidar_qx"), field("raw_lidar_qy"), field("raw_lidar_qz"),
        field("raw_lidar_qw"));
    const double t_delta = translationDifference(final_pose, saved_raw);
    const double r_delta = rotationDifferenceDeg(final_pose, saved_raw);
    const double fit_delta = std::abs(fitness - field("replayed_fitness"));
    Eigen::Quaternionf start_q(start.block<3, 3>(0, 0));
    Eigen::Quaternionf final_q(final_pose.block<3, 3>(0, 0));
    std::ostringstream start_pose_text, final_pose_text;
    start_pose_text << start.block<3,1>(0,3).transpose() << ';'
        << start_q.x() << ';' << start_q.y() << ';' << start_q.z() << ';' << start_q.w();
    final_pose_text << std::setprecision(9);
    for (int r = 0; r < 4; ++r) for (int c = 0; c < 4; ++c) {
      if (r || c) final_pose_text << ';';
      final_pose_text << final_pose(r, c);
    }
    writeRow(manifest, {integer(scan.transaction), std::to_string(index), integer(scan.stamp_ns),
        number(scan.time_s), reasons[index], std::to_string(source->size()),
        integer(scan.expected_cloud_hash), integer(actual_hash), hash_match ? "1" : "0",
        start_pose_text.str(), converged ? "1" : "0", std::to_string(iterations),
        number(fitness), number(probability), number(registration_ms), number(t_delta),
        number(r_delta), number(fit_delta), iterations == static_cast<int>(field("replayed_iterations")) ? "1" : "0",
        final_pose_text.str(), kExpectedMapSha, std::to_string(target->size()),
        number(hessian.jacobian_condition), hessian.coordinate_transform_valid ? "1" : "0",
        hessian.coordinate_transform_valid ? "0" : "1"});

    for (const MethodComponent& item : analysis.results) {
      writeRow(results, {integer(scan.transaction), number(scan.time_s), item.component,
          item.method, item.weakest_axis, number(item.values(0)), number(item.values(1)),
          number(item.values(2)), flattenVector6(analysis.raw_values), flatten(item.weakest),
          flatten(item.vectors), flatten(item.aligned_basis), flatten(item.aligned_values),
          flatten(item.contribution), item.axis_to_mode, item.finite ? "1" : "0",
          number(hessian.jacobian_condition), hessian.coordinate_transform_valid ? "1" : "0",
          hessian.coordinate_transform_valid ? "0" : "1",
          number(hessian.score), number(hessian.asymmetry),
          converged ? "1" : "0", std::to_string(iterations), number(fitness),
          number(probability), number(registration_ms), number(hessian.timing.hessian_ms),
          number(hessian.timing.canonical_ms), number(hessian.timing.rotation_transform_ms),
          number(hessian.timing.normalization_ms),
          number(analysis.timing.schur_ms), number(analysis.timing.eigensolve_ms),
          number(analysis.timing.physical_ms), number(analysis.timing.total_ms),
          analysis.solve_rot, analysis.solve_trans, number(analysis.cond_rr),
          number(analysis.cond_tt), std::to_string(analysis.timing.fallback_count),
          std::to_string(analysis.timing.factor_failures), flatten(hessian.raw_score),
          flatten(hessian.information), flatten(hessian.spatial_jacobian),
          flatten(hessian.physical_information), flatten(hessian.normalized), flatten(analysis.hrr),
          flatten(hessian.normalized.block<3,3>(0,3)),
          flatten(hessian.normalized.block<3,3>(3,0)), flatten(analysis.htt),
          flatten(analysis.sr), flatten(analysis.st)});
    }
    for (const std::string component : {"TRANSLATION", "ROTATION"}) {
      const int raw_index = component == "TRANSLATION" ? 0 : 1;
      const int block_index = component == "TRANSLATION" ? 2 : 3;
      const int schur_index = component == "TRANSLATION" ? 4 : 5;
      const MethodComponent& raw_item = analysis.results[static_cast<std::size_t>(raw_index)];
      const MethodComponent& block_item = analysis.results[static_cast<std::size_t>(block_index)];
      const MethodComponent& schur_item = analysis.results[static_cast<std::size_t>(schur_index)];
      writeRow(comparison, {integer(scan.transaction), number(scan.time_s), component,
          raw_item.weakest_axis, block_item.weakest_axis, schur_item.weakest_axis,
          number(analysis.raw_values(0)), number(analysis.raw_values(1)),
          number(analysis.raw_values(2)), number(block_item.values(0)),
          number(block_item.values(1)), number(block_item.values(2)),
          number(schur_item.values(0)), number(schur_item.values(1)),
          number(schur_item.values(2)), raw_item.finite ? "1" : "0",
          block_item.finite ? "1" : "0", schur_item.finite ? "1" : "0"});
    }
    writeRow(timings, {"FLOOR01", integer(scan.transaction), "NA",
        number(registration_ms), number(hessian.timing.hessian_ms),
        number(hessian.timing.canonical_ms), number(hessian.timing.rotation_transform_ms),
        number(hessian.timing.normalization_ms),
        number(analysis.timing.schur_ms), number(analysis.timing.eigensolve_ms),
        number(analysis.timing.physical_ms), number(analysis.timing.total_ms),
        std::to_string(analysis.timing.fallback_count),
        std::to_string(analysis.timing.factor_failures)});
    std::cout << "FLOOR01 tx=" << scan.transaction << " time_s=" << scan.time_s
              << " source_hash=PASS converged=" << converged
              << " iterations=" << iterations << " baseline_delta_t=" << t_delta
              << " baseline_delta_r_deg=" << r_delta
              << " analyzer_ms=" << analysis.timing.total_ms << std::endl;
  }
  std::cout << "FLOOR01_SELECTED_COUNT=" << selected.size() << std::endl;
}

}  // namespace

int main(int argc, char** argv) {
  try {
    if (argc == 5 && std::string(argv[1]) == "--synthetic") {
      const auto expectations = readExpectations(argv[3]);
      const auto perturbations = readPerturbations(argv[4]);
      if (expectations.size() != 4) throw std::runtime_error("expected four frozen fixtures");
      writeSynthetic(argv[2], expectations, perturbations);
      return 0;
    }
    if (argc == 8 && std::string(argv[1]) == "--floor01") {
      writeFloor01(argv[2], argv[3], argv[4], argv[5], argv[6]);
      // argv[7] is reserved for the separately audited input manifest.
      std::ifstream manifest(argv[7]);
      if (!manifest) throw std::runtime_error("missing frozen Floor01 input manifest");
      return 0;
    }
    std::cerr << "usage:\n"
              << "  p6_i3_uobs_ndt --synthetic OUT_DIR EXPECTED_AXES.csv PERTURBATIONS.csv\n"
              << "  p6_i3_uobs_ndt --floor01 MAP.pcd XYZ.bin scans.csv baseline_replay.csv OUT_DIR input_manifest.txt\n";
    return 2;
  } catch (const std::exception& error) {
    std::cerr << "P6_I3_ERROR: " << error.what() << '\n';
    return 1;
  }
}
