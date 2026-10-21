#pragma once

#include "../../p62_r14b_newer_college/conditional_base_pilot/tools/common.hpp"
#include <Eigen/Eigenvalues>
#include <Eigen/Cholesky>
#include <Eigen/SVD>
#include <cstdint>
#include <cstring>
#include <iomanip>
#include <numeric>
#include <unordered_map>

namespace coupled {
using namespace pilot;
using Signature = std::vector<std::vector<uintptr_t>>;
constexpr double kTranslationScale = 0.30;
constexpr double kRotationScale = 3.0 * shadow::PI / 180.0;
constexpr double kGradientStep = 5e-4;
constexpr double kHessianStep = 8e-3;
constexpr double kDirectionalStep = 1e-3;
constexpr double kLambda = 0.01;

inline Matrix6 scaleMatrix() {
  Matrix6 s = Matrix6::Zero();
  s.diagonal() << kTranslationScale, kTranslationScale, kTranslationScale,
                  kRotationScale, kRotationScale, kRotationScale;
  return s;
}

class ExactPclObjective : public shadow::NativeNdt {
 public:
  struct Associations {
    std::vector<std::vector<TargetGridLeafConstPtr>> leaves;  // PCL-return order is preserved for scoring.
    Signature canonical_ids;                                 // Sorted IDs are only for comparisons.
    size_t association_count = 0;
  };

  // setInputTarget() calls PCL 1.10 NDT::init(), which builds target_cells_.
  // This method is therefore called once per frozen map, never once per frame.
  void configureMap(const Cloud::Ptr& map) {
    require(!configured_map_, "Gaussian Grid already configured; refusing rebuild");
    shadow::NativeNdt::configure(map);
    require(frozenParameters(), "PCL NDT parameters differ from frozen R15 contract");
    // NDT recomputes these constants in computeTransformation(). Mirror the
    // exact PCL 1.10 equations from the configured stored parameters without
    // calling init() again (which would rebuild the Gaussian Grid).
    const double c1 = 10 * (1 - this->outlier_ratio_);
    const double c2 = this->outlier_ratio_ / std::pow(this->resolution_, 3);
    const double d3 = -std::log(c2);
    this->gauss_d1_ = -std::log(c1 + c2) - d3;
    this->gauss_d2_ = -2 * std::log(
        (-std::log(c1 * std::exp(-0.5) + c2) - d3) / this->gauss_d1_);
    // computeTransformation initializes this fixed translation Jacobian block
    // before its first call to computeDerivatives(). Mirror that exact PCL
    // precondition; the derivative routine itself does not initialize it.
    this->point_gradient_.setZero();
    this->point_gradient_.block<3, 3>(0, 0).setIdentity();
    this->point_hessian_.setZero();
    map_ = map;
    configured_map_ = true;
    buildLeafIdentityMap();
    initial_grid_fingerprint_ = gridFingerprint();
    this->checkGrid();
  }

  // Changing the source must not touch PCL's target Gaussian Grid.
  void setTraining(const Cloud::Ptr& training) {
    require(configured_map_ && training && !training->empty(), "map/source not ready");
    const std::string before = gridFingerprint();
    this->setInputSource(training);
    training_ = training;
    const std::string after = gridFingerprint();
    require(before == after && after == initial_grid_fingerprint_,
            "Gaussian Grid changed after setInputSource");
  }

  std::string gridFingerprint() {
    require(configured_map_, "Gaussian Grid fingerprint before map configuration");
    uint64_t h = 1469598103934665603ULL;
    const auto byte = [&h](uint8_t b) { h = (h ^ b) * 1099511628211ULL; };
    const auto u64 = [&byte](uint64_t v) { for (int i=0;i<64;i+=8) byte(uint8_t(v>>i)); };
    const auto real = [&u64](double v) { uint64_t bits=0; std::memcpy(&bits,&v,sizeof(bits)); u64(bits); };
    u64(this->target_cells_.getLeaves().size());
    for (const auto& entry : this->target_cells_.getLeaves()) {
      u64(static_cast<uint64_t>(entry.first));
      const auto& leaf = entry.second;
      u64(static_cast<uint64_t>(leaf.getPointCount()));
      const auto mean = leaf.getMean();
      for (int i=0;i<3;++i) real(static_cast<double>(mean[i]));
      const auto cov=leaf.getCov(), inv=leaf.getInverseCov();
      const auto evecs=leaf.getEvecs();
      for(int r=0;r<3;++r)for(int c=0;c<3;++c)real(cov(r,c));
      for(int r=0;r<3;++r)for(int c=0;c<3;++c)real(inv(r,c));
      const auto evals = leaf.getEvals();
      for (int i=0;i<3;++i) real(static_cast<double>(evals[i]));
      for(int r=0;r<3;++r)for(int c=0;c<3;++c)real(evecs(r,c));
    }
    const auto centroids = this->target_cells_.getCentroids();
    require(static_cast<bool>(centroids), "PCL Gaussian Grid has no centroids");
    u64(centroids->size());
    for (const auto& p : *centroids) { real(p.x); real(p.y); real(p.z); }
    std::ostringstream out; out<<std::hex<<std::setw(16)<<std::setfill('0')<<h; return out.str();
  }

  std::string initialGridFingerprint() const { return initial_grid_fingerprint_; }
  size_t gaussianLeafCount() { return this->target_cells_.getLeaves().size(); }
  Cloud::Ptr mapCloud() const { return map_; }

  double storedResolution() const { return this->resolution_; }
  double gaussianD1() const { return this->gauss_d1_; }
  double gaussianD2() const { return this->gauss_d2_; }
  bool frozenParameters() {
    return this->getResolution()==0.8f && std::abs(this->getStepSize()-0.08)<=1e-15 &&
      std::abs(this->getTransformationEpsilon()-0.001)<=1e-15 &&
      this->getMaximumIterations()==40 && std::abs(this->getOulierRatio()-0.55)<=1e-15;
  }

  const Cloud::Ptr& trainingCloud() const { return training_; }

  Associations associations(const Pose& pose) {
    require(configured_map_ && training_ && rigid(pose), "invalid association input");
    const Cloud trans = transformed(pose);
    Associations result;
    result.leaves.reserve(trans.size()); result.canonical_ids.reserve(trans.size());
    std::vector<TargetGridLeafConstPtr> leaves; std::vector<float> distances;
    for (const auto& q : trans) {
      leaves.clear(); distances.clear();
      this->target_cells_.radiusSearch(q, this->resolution_, leaves, distances);
      std::vector<uintptr_t> ids; ids.reserve(leaves.size());
      for (const auto* leaf : leaves) {
        const auto it = leaf_ids_.find(leaf);
        require(it != leaf_ids_.end(), "association references a leaf outside frozen grid");
        ids.push_back(static_cast<uintptr_t>(it->second));
      }
      std::sort(ids.begin(), ids.end());
      result.association_count += leaves.size();
      result.leaves.push_back(leaves); result.canonical_ids.push_back(std::move(ids));
    }
    return result;
  }

  double scoreFixed(const std::vector<Eigen::Vector3d>& continuous_points,
                    const std::vector<std::vector<TargetGridLeafConstPtr>>& fixed_leaves) const {
    require(training_ && continuous_points.size()==training_->size() &&
            fixed_leaves.size()==training_->size(), "fixed score shape mismatch");
    double total=0.0;
    for (size_t i=0;i<continuous_points.size();++i) {
      const Eigen::Vector3d& y=continuous_points[i];
      for (const auto* leaf:fixed_leaves[i]) {
        const Eigen::Vector3d e=y-leaf->getMean();
        const double e2=e.dot(leaf->getInverseCov()*e);
        const double exponential=std::exp(-this->gauss_d2_*e2/2.0);
        const double guarded=this->gauss_d2_*exponential;
        if (!std::isfinite(guarded) || guarded<0.0 || guarded>1.0) continue;
        const double increment=-this->gauss_d1_*exponential;
        if (!std::isfinite(increment)) return NAN;
        total+=increment;
      }
    }
    return total;
  }

  Cloud transformed(const Pose& pose) const {
    require(rigid(pose), "non-rigid pose for exact PCL transform");
    Cloud out;
    pcl::transformPointCloud(*training_, out, pose.cast<float>());
    return out;
  }

  Signature associationSignature(const Pose& pose) {
    const Cloud trans = transformed(pose);
    Signature signature;
    signature.reserve(trans.size());
    std::vector<TargetGrid::LeafConstPtr> leaves;
    std::vector<float> distances;
    for (const auto& q : trans) {
      leaves.clear(); distances.clear();
      this->target_cells_.radiusSearch(q, this->resolution_, leaves, distances);
      std::vector<uintptr_t> ids; ids.reserve(leaves.size());
      for (const auto& leaf : leaves)
        ids.push_back(reinterpret_cast<uintptr_t>(leaf));
      std::sort(ids.begin(), ids.end());
      signature.push_back(std::move(ids));
    }
    return signature;
  }

  double directScore(const Pose& pose, Signature* signature = nullptr) {
    const Cloud trans = transformed(pose);
    double total = 0.0;
    Signature local;
    if (signature) local.reserve(trans.size());
    std::vector<TargetGrid::LeafConstPtr> leaves;
    std::vector<float> distances;
    for (size_t i = 0; i < trans.size(); ++i) {
      const auto& q = trans[i];
      leaves.clear(); distances.clear();
      this->target_cells_.radiusSearch(q, this->resolution_, leaves, distances);
      if (signature) {
        std::vector<uintptr_t> ids; ids.reserve(leaves.size());
        for (const auto& leaf : leaves)
          ids.push_back(reinterpret_cast<uintptr_t>(leaf));
        std::sort(ids.begin(), ids.end());
        local.push_back(std::move(ids));
      }
      const Eigen::Vector3d y(q.x, q.y, q.z);
      for (const auto& leaf : leaves) {
        const Eigen::Vector3d residual = y - leaf->getMean();
        const double exponential = std::exp(
            -this->gauss_d2_ * residual.dot(leaf->getInverseCov() * residual) / 2.0);
        double score_inc = -this->gauss_d1_ * exponential;
        const double guarded = this->gauss_d2_ * exponential;
        if (guarded > 1.0 || guarded < 0.0 || std::isnan(guarded)) continue;
        total += score_inc;
      }
    }
    if (signature) *signature = std::move(local);
    return total;
  }

  double pclDerivativeScore(const Pose& pose) {
    Cloud trans = transformed(pose);
    const Eigen::Matrix4f tf = pose.cast<float>();
    Eigen::Transform<float, 3, Eigen::Affine, Eigen::ColMajor> affine;
    affine.matrix() = tf;
    const Eigen::Vector3f euler = affine.rotation().eulerAngles(0, 1, 2);
    Eigen::Matrix<double, 6, 1> p;
    p << tf(0, 3), tf(1, 3), tf(2, 3), euler(0), euler(1), euler(2);
    Eigen::Matrix<double, 6, 1> score_gradient;
    Eigen::Matrix<double, 6, 6> hessian;
    const double score = this->computeDerivatives(
        score_gradient, hessian, trans, p, false);
    return score;
  }

  double objective(const Pose& pose) {
    return -directScore(pose) / static_cast<double>(training_->size());
  }

  Matrix6 directGeometry(const Pose& pose, size_t& supported) {
    require(rigid(pose) && training_ && !training_->empty(), "invalid G input");
    const Eigen::Matrix3d r = pose.block<3, 3>(0, 0);
    // Use the same float PointXYZ transform as PCL for target-grid queries
    // and residual weights; retain the analytic differential in double.
    const Cloud transformed_training = transformed(pose);
    Matrix6 g = Matrix6::Zero(); supported = 0;
    std::vector<TargetGrid::LeafConstPtr> leaves;
    std::vector<float> distances;
    for (size_t index = 0; index < training_->size(); ++index) {
      const auto& point = training_->points[index];
      const Eigen::Vector3d x(point.x, point.y, point.z);
      const auto& transformed_point = transformed_training.points[index];
      const Eigen::Vector3d y(transformed_point.x, transformed_point.y, transformed_point.z);
      const pcl::PointXYZ query = transformed_point;
      leaves.clear(); distances.clear();
      this->target_cells_.radiusSearch(query, this->resolution_, leaves, distances);
      if (!leaves.empty()) ++supported;
      Eigen::Matrix<double, 3, 6> j;
      j.leftCols<3>() = r;
      j.rightCols<3>() = -r * cause::skew(x);
      j *= scaleMatrix();
      for (const auto& leaf : leaves) {
        const Eigen::Vector3d e = y - leaf->getMean();
        const auto& q = leaf->getInverseCov();
        const double exponential = std::exp(-this->gauss_d2_ * e.dot(q * e) / 2.0);
        const double guard = this->gauss_d2_ * exponential;
        if (!std::isfinite(guard) || guard < 0.0 || guard > 1.0) continue;
        const double weight = -this->gauss_d1_ * this->gauss_d2_ * exponential;
        if (weight >= 0.0 && std::isfinite(weight))
          g.noalias() += weight * j.transpose() * q * j;
      }
    }
    g /= static_cast<double>(training_->size());
    return 0.5 * (g + g.transpose()).eval();
  }

  Matrix6 convertedR6Geometry(const Pose& pose, size_t& supported) {
    const Matrix6 g_legacy = this->geometry(training_, pose, supported);
    Matrix6 d_inv = Matrix6::Identity();
    d_inv.diagonal().head<3>().setConstant(1.0 / 0.8);
    Matrix6 a = Matrix6::Zero();
    a.block<3, 3>(0, 0) = pose.block<3, 3>(0, 0);
    a.block<3, 3>(3, 3) = pose.block<3, 3>(0, 0);
    a *= scaleMatrix();
    return a.transpose() * d_inv.transpose() * g_legacy * d_inv * a;
  }

  // Rebuild the legacy-coordinate expression from precisely the same float
  // query points, association leaves, residuals, and Gaussian weights used by
  // directGeometry(). This isolates algebraic coordinate-conversion error
  // from the historical R6 geometry() arithmetic differences.
  Matrix6 convertedSameArithmeticGeometry(const Pose& pose, size_t& supported) {
    require(rigid(pose) && training_ && !training_->empty(), "invalid same-path G input");
    const Eigen::Matrix3d r=pose.block<3,3>(0,0);
    const Cloud transformed_training=transformed(pose);
    Matrix6 legacy=Matrix6::Zero(); supported=0;
    std::vector<TargetGridLeafConstPtr> leaves; std::vector<float> distances;
    for(size_t i=0;i<training_->size();++i){
      const auto& p=training_->points[i];
      const Eigen::Vector3d x(p.x,p.y,p.z), rx=r*x;
      const auto& qpt=transformed_training.points[i];
      const Eigen::Vector3d y(qpt.x,qpt.y,qpt.z);
      leaves.clear();distances.clear();this->target_cells_.radiusSearch(qpt,this->resolution_,leaves,distances);
      if(!leaves.empty())++supported;
      Eigen::Matrix<double,3,6> j;
      j.leftCols<3>()=.8*Eigen::Matrix3d::Identity();
      j.rightCols<3>()=-cause::skew(rx);
      for(const auto* leaf:leaves){
        const Eigen::Vector3d e=y-leaf->getMean();const auto& q=leaf->getInverseCov();
        const double exponential=std::exp(-this->gauss_d2_*e.dot(q*e)/2.0);
        const double guard=this->gauss_d2_*exponential;
        if(!std::isfinite(guard)||guard<0.0||guard>1.0)continue;
        const double weight=-this->gauss_d1_*this->gauss_d2_*exponential;
        if(weight>=0.0&&std::isfinite(weight))legacy.noalias()+=weight*j.transpose()*q*j;
      }
    }
    legacy/=static_cast<double>(training_->size());
    Matrix6 d_inv=Matrix6::Identity();d_inv.diagonal().head<3>().setConstant(1.0/.8);
    Matrix6 a=Matrix6::Zero();a.block<3,3>(0,0)=r;a.block<3,3>(3,3)=r;a*=scaleMatrix();
    return a.transpose()*d_inv.transpose()*legacy*d_inv*a;
  }

 private:
  void buildLeafIdentityMap() {
    leaf_ids_.clear();
    for (const auto& entry : this->target_cells_.getLeaves())
      leaf_ids_.emplace(&entry.second, static_cast<uint64_t>(entry.first));
    require(leaf_ids_.size()==this->target_cells_.getLeaves().size(), "nonunique Gaussian leaf identity");
  }
  Cloud::Ptr map_, training_;
  bool configured_map_=false;
  std::unordered_map<TargetGridLeafConstPtr,uint64_t> leaf_ids_;
  std::string initial_grid_fingerprint_;
};

struct DerivativeResult {
  bool ok = false;
  std::string status = "NOT_RUN";
  Matrix6 h_raw = Matrix6::Constant(NAN);
  Vector6 g_z = Vector6::Constant(NAN);
  double score_parity = NAN;
  double h_antisymmetry = NAN;
  double gradient_check_max_rel = NAN;
  double hessian_check_max_rel = NAN;
  double hessian_step_spread_max = NAN;
  int association_queries = 0;
  std::string association_status = "UNTESTED";
};

inline Pose poseAt(const Pose& t0, const Vector6& z) {
  return t0 * cause::expSE3(scaleMatrix() * z);
}

inline bool sameSignature(const Signature& a, const Signature& b) {
  return a == b;
}

inline double relativeError(double a, double b) {
  return std::abs(a - b) / std::max({std::abs(a), std::abs(b), 1e-8});
}

class LocalModel {
 public:
  LocalModel(ExactPclObjective& ndt, Pose t0)
      : ndt_(ndt), t0_(std::move(t0)), sig0_(ndt_.associationSignature(t0_)) {}

  const Signature& nominalSignature() const { return sig0_; }

  bool associationStable(const Vector6& z, int& count) const {
    ++count;
    return sameSignature(ndt_.associationSignature(poseAt(t0_, z)), sig0_);
  }

  Vector6 gradient(const Vector6& z, int& association_queries,
                   std::string& failure) const {
    Vector6 g;
    for (int i = 0; i < 6; ++i) {
      Vector6 zp = z, zm = z;
      zp[i] += kGradientStep; zm[i] -= kGradientStep;
      if (!associationStable(zp, association_queries) ||
          !associationStable(zm, association_queries)) {
        failure = "LOCAL_ASSOCIATION_SET_CHANGED"; return Vector6::Constant(NAN);
      }
      const double fp = ndt_.objective(poseAt(t0_, zp));
      const double fm = ndt_.objective(poseAt(t0_, zm));
      if (!std::isfinite(fp) || !std::isfinite(fm)) {
        failure = "NONFINITE_OBJECTIVE_STENCIL"; return Vector6::Constant(NAN);
      }
      g[i] = (fp - fm) / (2.0 * kGradientStep);
    }
    failure.clear();
    return g;
  }

  DerivativeResult validateAndDifferentiate() const {
    DerivativeResult out;
    const double deriv_score = ndt_.pclDerivativeScore(t0_);
    const double direct = ndt_.directScore(t0_);
    out.score_parity = std::abs(deriv_score - direct);
    if (!std::isfinite(out.score_parity) || out.score_parity > 1e-9) {
      out.status = "PCL_SCORE_PARITY_FAILED"; return out;
    }

    const Vector6 zero = Vector6::Zero();
    std::string failure;
    out.g_z = gradient(zero, out.association_queries, failure);
    if (!failure.empty() || !out.g_z.allFinite()) {
      out.status = failure.empty() ? "NONFINITE_GRADIENT" : failure; return out;
    }

    // First-order check: six axes plus four predeclared mixed directions.
    std::vector<Vector6> directions;
    for (int i = 0; i < 6; ++i) { Vector6 v = Vector6::Zero(); v[i] = 1.; directions.push_back(v); }
    for (const auto& ij : std::array<std::pair<int, int>, 4>{{{0,3},{1,4},{2,5},{0,1}}}) {
      Vector6 v = Vector6::Zero(); v[ij.first] = v[ij.second] = 1. / std::sqrt(2.); directions.push_back(v);
    }
    double grad_rel = 0.;
    constexpr double grad_h = kDirectionalStep;
    const double f0 = ndt_.objective(t0_);
    for (const auto& v : directions) {
      const Vector6 zp = grad_h * v, zm = -grad_h * v;
      if (!associationStable(zp, out.association_queries) ||
          !associationStable(zm, out.association_queries)) {
        out.status = "LOCAL_ASSOCIATION_SET_CHANGED"; return out;
      }
      const double fp = ndt_.objective(poseAt(t0_, zp));
      const double fm = ndt_.objective(poseAt(t0_, zm));
      const double fd = (fp - fm) / (2. * grad_h);
      const double predicted = out.g_z.dot(v);
      if (!std::isfinite(fd) || !std::isfinite(predicted)) {
        out.status = "NONFINITE_DIRECTIONAL_DERIVATIVE"; return out;
      }
      if (std::abs(fd) > 1e-8) grad_rel = std::max(grad_rel, relativeError(fd, predicted));
      else if (std::abs(predicted) > 1e-8) grad_rel = std::max(grad_rel, relativeError(fd, predicted));
    }
    out.gradient_check_max_rel = grad_rel;
    if (grad_rel > 2e-2) { out.status = "DIRECTIONAL_DERIVATIVE_FAILED"; return out; }

    // Central difference of the fixed-step exact-score gradient in right Lie
    // coordinates. Every nested objective stencil preserves leaf association.
    for (int i = 0; i < 6; ++i) {
      Vector6 zp = Vector6::Zero(), zm = Vector6::Zero();
      zp[i] = kHessianStep; zm[i] = -kHessianStep;
      const Vector6 gp = gradient(zp, out.association_queries, failure);
      if (!failure.empty()) { out.status = failure; return out; }
      const Vector6 gm = gradient(zm, out.association_queries, failure);
      if (!failure.empty()) { out.status = failure; return out; }
      out.h_raw.col(i) = (gp - gm) / (2. * kHessianStep);
    }
    if (!out.h_raw.allFinite()) { out.status = "NONFINITE_HESSIAN"; return out; }
    out.h_antisymmetry = (out.h_raw - out.h_raw.transpose()).norm() /
                         std::max(1.0, out.h_raw.norm());
    if (out.h_antisymmetry > 5e-2) { out.status = "HESSIAN_ANTISYMMETRY_FAILED"; return out; }

    // Independent scalar second-difference checks on the raw (unsymmetrized)
    // gradient Hessian, including step-size sensitivity.
    const std::array<double, 3> steps{{4e-3, 8e-3, 16e-3}};
    double max_rel = 0., spread_max = 1.;
    for (const auto& v : directions) {
      std::array<double, 3> scalar{};
      for (size_t si = 0; si < steps.size(); ++si) {
        const double h = steps[si];
        const Vector6 zp = h * v, zm = -h * v;
        if (!associationStable(zp, out.association_queries) ||
            !associationStable(zm, out.association_queries)) {
          out.status = "LOCAL_ASSOCIATION_SET_CHANGED"; return out;
        }
        const double fp = ndt_.objective(poseAt(t0_, zp));
        const double fm = ndt_.objective(poseAt(t0_, zm));
        scalar[si] = (fp - 2. * f0 + fm) / (h * h);
        if (!std::isfinite(scalar[si])) { out.status = "NONFINITE_SCALAR_HESSIAN_CHECK"; return out; }
        if (si == 1) {
          const double predicted = v.dot(out.h_raw * v);
          max_rel = std::max(max_rel, relativeError(scalar[si], predicted));
        }
      }
      for (size_t si = 1; si < steps.size(); ++si) {
        const double lo = std::min(std::abs(scalar[si-1]), std::abs(scalar[si]));
        const double hi = std::max(std::abs(scalar[si-1]), std::abs(scalar[si]));
        const double ratio = lo <= 1e-8 ? (hi <= 1e-8 ? 1. : std::numeric_limits<double>::infinity()) : hi / lo;
        spread_max = std::max(spread_max, ratio);
      }
    }
    out.hessian_check_max_rel = max_rel;
    out.hessian_step_spread_max = spread_max;
    if (max_rel > 1e-1 || spread_max > 2.) {
      out.status = "SCALAR_HESSIAN_CHECK_FAILED"; return out;
    }
    out.status = "PASS"; out.ok = true; return out;
  }

 private:
  ExactPclObjective& ndt_;
  Pose t0_;
  Signature sig0_;
};

struct Refinement {
  bool weak_available = false;
  bool coupled_available = false;
  bool exploratory_min_spectral_probe = false;
  int formal_weak_count = 0;
  double minimum_eigenvalue_ratio = NAN;
  std::string status = "NOT_EVALUABLE";
  std::string weak_status = "NOT_EVALUABLE";
  std::string coupled_status = "NOT_EVALUABLE";
  double alpha = NAN;
  Vector6 dw = Vector6::Constant(NAN);
  Vector6 ds = Vector6::Constant(NAN);
  Eigen::MatrixXd hss, hsw, hww;
  Eigen::VectorXd gs, gw, coupling_increment;
  std::string weak_ids, strong_ids, unmodeled_weak_ids;
  Vector6 z_weak = Vector6::Constant(NAN);
  Vector6 z_coupled = Vector6::Constant(NAN);
  Pose weak_pose = Pose::Constant(NAN);
  Pose coupled_pose = Pose::Constant(NAN);
  double local_delta_weak = NAN, local_delta_coupled = NAN;
  double exact_delta_weak = NAN, exact_delta_coupled = NAN;
  double raw_hss_condition = NAN, regularized_hss_condition = NAN;
  double raw_hss_antisymmetry = NAN;
  double coupling_norm = NAN, strong_response_norm = NAN, trust_scale = NAN;
  bool weak_only_formal_eligible = false, coupled_formal_eligible = false;
};

inline double conditionOfPositive(const Eigen::MatrixXd& a, bool& positive) {
  positive = false;
  if (a.rows() == 0 || a.rows() != a.cols() || !a.allFinite()) return NAN;
  Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> solver(a);
  if (solver.info() != Eigen::Success || !solver.eigenvalues().allFinite()) return NAN;
  const auto values = solver.eigenvalues();
  if (values.minCoeff() <= 0.) return NAN;
  positive = true;
  return values.maxCoeff() / values.minCoeff();
}

inline Refinement refine(ExactPclObjective& ndt,
                         const Pose& t0, const Pose& tp,
                         const Eigen::Matrix<double, 6, 1>& eigenvalues,
                         const Matrix6& u, const Matrix6& h_z,
                         const Vector6& g_z, double alpha,
                         bool exploratory_min_spectral_probe=false) {
  Refinement out; out.alpha = alpha;
  std::vector<int> weak;
  const double high = eigenvalues[5];
  if (!std::isfinite(high) || high <= 1e-12) {
    out.status = out.weak_status = out.coupled_status = "NO_INFORMATION"; return out;
  }
  out.minimum_eigenvalue_ratio=eigenvalues[0]/high;
  out.formal_weak_count=int((eigenvalues.array()/high<=.01).count());
  out.exploratory_min_spectral_probe=exploratory_min_spectral_probe;
  if(exploratory_min_spectral_probe) weak.push_back(0);
  else for (int i = 0; i < 6 && int(weak.size()) < 2; ++i)
    if (eigenvalues[i] / high <= .01) weak.push_back(i);
  if (weak.empty()) {
    out.status = out.weak_status = out.coupled_status = "NO_WEAK_DIRECTION"; return out;
  }
  std::vector<int> strong, unmodeled;
  for (int i = 0; i < 6; ++i) {
    if(std::find(weak.begin(),weak.end(),i)!=weak.end()) {
      continue;
    } else if (eigenvalues[i] / high <= .01) {
      unmodeled.push_back(i);
    } else {
      strong.push_back(i);
    }
  }
  for (int id : weak) { if (!out.weak_ids.empty()) out.weak_ids += "|"; out.weak_ids += std::to_string(id); }
  for (int id : strong) { if (!out.strong_ids.empty()) out.strong_ids += "|"; out.strong_ids += std::to_string(id); }
  for (int id : unmodeled) { if (!out.unmodeled_weak_ids.empty()) out.unmodeled_weak_ids += "|"; out.unmodeled_weak_ids += std::to_string(id); }
  Eigen::MatrixXd uw(6, weak.size());
  for (size_t i = 0; i < weak.size(); ++i) uw.col(i) = u.col(weak[i]);
  const Vector6 xi_pred = cause::logSE3(t0.inverse() * tp);
  const Vector6 z_pred = scaleMatrix().inverse() * xi_pred;
  Eigen::VectorXd dw = alpha * uw.transpose() * z_pred;
  if (!dw.allFinite() || dw.norm() <= 1e-10) {
    out.status = out.weak_status = out.coupled_status = "NO_VALID_PREDICTION_PROJECTION"; return out;
  }
  if (dw.norm() > 1.) dw /= dw.norm();

  // Freeze the paired weak-only candidate before attempting the coupled solve.
  // A failed/ill-conditioned Hss must not erase a valid WEAK_ONLY ablation.
  const int ns = strong.size(), nw = weak.size();
  Eigen::VectorXd qweak = Eigen::VectorXd::Zero(6);
  out.dw = Vector6::Zero();
  for (int i = 0; i < nw; ++i) { qweak[weak[i]] = dw[i]; out.dw[weak[i]] = dw[i]; }
  out.z_weak = u * qweak;
  const Matrix6 hsym = .5 * (h_z + h_z.transpose()).eval();
  out.weak_pose = poseAt(t0, out.z_weak);
  out.local_delta_weak = g_z.dot(out.z_weak) + .5 * out.z_weak.dot(hsym * out.z_weak);
  const double f0 = ndt.objective(t0);
  out.exact_delta_weak = ndt.objective(out.weak_pose) - f0;
  if (!out.z_weak.allFinite() || out.z_weak.norm() > 1. + 1e-10 || !rigid(out.weak_pose) ||
      !std::isfinite(out.local_delta_weak) || !std::isfinite(out.exact_delta_weak)) {
    out.status = out.weak_status = out.coupled_status = "INVALID_WEAK_ONLY_CANDIDATE"; return out;
  }
  out.weak_available = true;
  out.weak_status = exploratory_min_spectral_probe ? "SPECTRAL_TAIL_WEAK_ONLY_PROBE_READY" : "WEAK_ONLY_READY";

  Eigen::VectorXd gs(ns), gw(nw), ds(ns);
  Eigen::MatrixXd hss(ns, ns), hsw(ns, nw), hww(nw, nw);
  const auto failCoupled = [&](const std::string& status) {
    out.status = out.coupled_status = status;
  };
  if (ns == 0) {
    failCoupled("NO_STRONG_SUBSPACE"); return out;
  }
  const Matrix6 h_model = .5 * (h_z + h_z.transpose()).eval();
  Eigen::MatrixXd raw_hss(ns, ns);
  for (int i = 0; i < ns; ++i) {
    const int qi = strong[i]; gs[i] = u.col(qi).dot(g_z);
    for (int j = 0; j < ns; ++j) {
      raw_hss(i,j) = u.col(qi).dot(h_z * u.col(strong[j]));
      hss(i,j) = u.col(qi).dot(h_model * u.col(strong[j]));
    }
    for (int j = 0; j < nw; ++j) hsw(i,j) = u.col(qi).dot(h_model * uw.col(j));
  }
  for (int i = 0; i < nw; ++i) {
    gw[i] = uw.col(i).dot(g_z);
    for (int j = 0; j < nw; ++j) hww(i,j) = uw.col(i).dot(h_model * uw.col(j));
  }
  out.raw_hss_antisymmetry = (raw_hss - raw_hss.transpose()).norm() /
                              std::max(1.0, raw_hss.norm());
  if (!std::isfinite(out.raw_hss_antisymmetry) || out.raw_hss_antisymmetry > .05) {
    failCoupled("RAW_HSS_ASYMMETRY_TOO_LARGE"); return out;
  }
  out.hss = hss; out.hsw = hsw; out.hww = hww; out.gs = gs; out.gw = gw;
  out.coupling_norm = hsw.norm();
  if (!gs.allFinite() || !gw.allFinite() || !hss.allFinite() || !hsw.allFinite() || !hww.allFinite()) {
    failCoupled("NONFINITE_COUPLING_BLOCK"); return out;
  }
  bool raw_positive = false;
  out.raw_hss_condition = conditionOfPositive(hss, raw_positive);
  if (!raw_positive || !std::isfinite(out.raw_hss_condition) || out.raw_hss_condition > 1e8) {
    failCoupled("RAW_HSS_NOT_POSITIVE_OR_ILL_CONDITIONED"); return out;
  }
  Eigen::MatrixXd hreg = hss + kLambda * Eigen::MatrixXd::Identity(ns, ns);
  bool reg_positive = false;
  out.regularized_hss_condition = conditionOfPositive(hreg, reg_positive);
  if (!reg_positive || !std::isfinite(out.regularized_hss_condition) || out.regularized_hss_condition > 1e8) {
    failCoupled("REGULARIZED_HSS_NOT_POSITIVE_OR_ILL_CONDITIONED"); return out;
  }
  Eigen::LDLT<Eigen::MatrixXd> ldlt(hreg);
  if (ldlt.info() != Eigen::Success || !ldlt.isPositive()) { failCoupled("LDLT_FAILED"); return out; }
  const Eigen::VectorXd ds_uncoupled = ldlt.solve(-gs);
  ds = ldlt.solve(-(gs + hsw * dw));
  if (ldlt.info() != Eigen::Success || !ds.allFinite()) { failCoupled("NONFINITE_STRONG_RESPONSE"); return out; }
  const double ds_norm = ds.norm();
  const double remaining = std::sqrt(std::max(0., 1. - dw.squaredNorm()));
  out.trust_scale = ds_norm > remaining && ds_norm > 0. ? remaining / ds_norm : 1.;
  ds *= out.trust_scale;
  Eigen::VectorXd qcoupled = qweak;
  for (int i = 0; i < ns; ++i) qcoupled[strong[i]] = ds[i];
  out.z_coupled = u * qcoupled;
  if (!out.z_coupled.allFinite() || out.z_coupled.norm() > 1. + 1e-10) {
    failCoupled("COUPLED_TRUST_REGION_VIOLATION"); return out;
  }
  out.ds = Vector6::Zero();
  for (int i = 0; i < ns; ++i) out.ds[strong[i]] = ds[i];
  out.coupled_pose = poseAt(t0, out.z_coupled);
  out.local_delta_coupled = g_z.dot(out.z_coupled) + .5 * out.z_coupled.dot(h_model * out.z_coupled);
  out.exact_delta_coupled = ndt.objective(out.coupled_pose) - f0;
  if (!rigid(out.coupled_pose) || !std::isfinite(out.local_delta_coupled) || !std::isfinite(out.exact_delta_coupled)) {
    failCoupled("INVALID_COUPLED_CANDIDATE"); return out;
  }
  out.weak_only_formal_eligible = false; out.coupled_formal_eligible = false;
  out.strong_response_norm = ds.norm(); out.coupling_increment = ds - ds_uncoupled;
  out.weak_available = true; out.coupled_available = true;
  out.status = out.coupled_status = exploratory_min_spectral_probe ?
      "SPECTRAL_TAIL_COUPLED_PROBE_COMPUTED" : "COUPLED_RESPONSE_COMPUTED";
  return out;
}

}  // namespace coupled
