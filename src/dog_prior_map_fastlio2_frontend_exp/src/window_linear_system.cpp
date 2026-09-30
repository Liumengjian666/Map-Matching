#include "dog_prior_map_fastlio2_frontend_exp/fixed_lag_window.hpp"
#include <Eigen/Cholesky>
#include <Eigen/SparseCholesky>
#include <cmath>
#include <limits>
#include <map>
#include <set>

namespace dog_prior_map_fastlio2_frontend_exp::fixed_lag {
void WindowLinearSystem::add(std::size_t row, std::size_t col, const Matrix15d& block) {
  const auto key = std::make_pair(std::min(row,col), std::max(row,col));
  auto inserted = upper_blocks.emplace(key, Matrix15d::Zero());
  inserted.first->second += row <= col ? block : Matrix15d(block.transpose());
}
Eigen::MatrixXd WindowLinearSystem::dense() const {
  Eigen::MatrixXd h = Eigen::MatrixXd::Zero(gradient.size(), gradient.size());
  for (const auto& entry : upper_blocks) {
    const Eigen::Index i = entry.first.first*15, j = entry.first.second*15;
    h.block<15,15>(i,j) = entry.second;
    if (i != j) h.block<15,15>(j,i) = entry.second.transpose();
  }
  return (0.5 * (h+h.transpose())).eval();
}
Eigen::SparseMatrix<double> WindowLinearSystem::sparse() const {
  std::vector<Eigen::Triplet<double>> triplets;
  triplets.reserve(upper_blocks.size()*450);
  for (const auto& entry : upper_blocks) {
    const int i = entry.first.first*15, j = entry.first.second*15;
    const Matrix15d block = i == j ? Matrix15d(0.5*(entry.second+entry.second.transpose())) : entry.second;
    for (int r=0; r<15; ++r) for (int c=0; c<15; ++c) {
      if (block(r,c) == 0) continue;
      triplets.emplace_back(i+r,j+c,block(r,c));
      if (i != j) triplets.emplace_back(j+c,i+r,block(r,c));
    }
  }
  Eigen::SparseMatrix<double> h(gradient.size(),gradient.size());
  h.setFromTriplets(triplets.begin(),triplets.end()); h.makeCompressed(); return h;
}
std::size_t WindowLinearSystem::payloadBytes() const {
  return upper_blocks.size()*sizeof(Matrix15d) + gradient.size()*sizeof(double);
}
bool solveWindowLinearSystem(const WindowLinearSystem& system, double damping,
    WindowSolverBackend backend, Eigen::VectorXd* step, std::string* status) {
  if (!step || !system.gradient.allFinite() || !std::isfinite(damping) || damping < 0)
    return false;
  if (backend == WindowSolverBackend::BLOCK_SPARSE) {
    Eigen::SparseMatrix<double> h = system.sparse();
    for (Eigen::Index i=0; i<h.rows(); ++i) {
      const double diagonal = h.coeff(i,i);
      h.coeffRef(i,i) += damping * std::max(std::abs(diagonal),1.0);
    }
    h.makeCompressed();
    Eigen::SimplicialLDLT<Eigen::SparseMatrix<double>> factor;
    factor.compute(h);
    if (factor.info() == Eigen::Success && factor.vectorD().allFinite() &&
        (factor.vectorD().array() > 0).all()) {
      *step = factor.solve(-system.gradient);
      if (factor.info() == Eigen::Success && step->allFinite()) {
        if (status) *status = "BLOCK_SPARSE_SIMPLICIAL_LDLT";
        return true;
      }
    }
    if (status) *status = "SPARSE_SOLVER_FALLBACK_DENSE";
  } else if (status) *status = "DENSE_REFERENCE_LDLT";
  Eigen::MatrixXd h = system.dense();
  h.diagonal().array() += damping*h.diagonal().cwiseAbs().array().max(1.0);
  Eigen::LDLT<Eigen::MatrixXd> factor(h);
  if (factor.info() != Eigen::Success) return false;
  *step = factor.solve(-system.gradient);
  return factor.info() == Eigen::Success && step->allFinite();
}

bool solveLatestMarginalColumnsSparse(const WindowLinearSystem& system,
    Eigen::MatrixXd* columns, double* backward_error, std::string* status) {
  const auto unavailable = [&](const char* detail) {
    if (status) *status = detail;
    return false;
  };
  if (!columns || !backward_error || system.gradient.size() < 15)
    return unavailable("INVALID_MARGINAL_SYSTEM");
  const Eigen::SparseMatrix<double> h = system.sparse();
  const Eigen::VectorXd diagonal = h.diagonal();
  if (!diagonal.allFinite() || (diagonal.array() <= 0).any())
    return unavailable("NONPOSITIVE_HESSIAN_DIAGONAL");
  const Eigen::VectorXd scale = diagonal.array().sqrt().inverse();
  Eigen::SparseMatrix<double> balanced = h;
  for (int k = 0; k < balanced.outerSize(); ++k)
    for (Eigen::SparseMatrix<double>::InnerIterator it(balanced, k); it; ++it) {
      if (!std::isfinite(it.value())) return unavailable("NONFINITE_HESSIAN");
      it.valueRef() *= scale(it.row()) * scale(it.col());
    }
  Eigen::SimplicialLLT<Eigen::SparseMatrix<double>, Eigen::Lower,
      Eigen::NaturalOrdering<int>> factor;
  factor.compute(balanced);
  if (factor.info() != Eigen::Success) return unavailable("HESSIAN_NOT_SPD");
  // Preserve the existing balanced-pivot reliability gate, including ordering.
  const Eigen::SparseMatrix<double> lower = factor.matrixL();
  if (lower.diagonal().array().square().minCoeff() <= 1e-12)
    return unavailable("NUMERICALLY_SINGULAR_HESSIAN");
  Eigen::MatrixXd selector = Eigen::MatrixXd::Zero(h.rows(), 15);
  selector.bottomRows(15).setIdentity();
  *columns = scale.asDiagonal() * factor.solve(scale.asDiagonal() * selector);
  *backward_error = (h * *columns - selector).norm() /
      (h.norm() * columns->norm() + selector.norm());
  if (factor.info() != Eigen::Success || !columns->allFinite() ||
      !std::isfinite(*backward_error) || *backward_error > 1e-10)
    return unavailable("SOLVE_RESIDUAL");
  if (status) *status = "SPARSE_UNDAMPED_SIMPLICIAL_LLT";
  return true;
}

bool FixedLagWindow::blockLinearizedSystem(WindowLinearSystem* system,
    std::string* reason, ObjectiveBreakdown* breakdown) const {
  LidarIterationSnapshot snapshot;
  if (!buildLidarIterationSnapshot(&snapshot, reason)) return false;
  return blockLinearizedSystemWithLidarSnapshot(snapshot, system, reason,
                                                 breakdown);
}

double FixedLagWindow::objectiveWithLidarSnapshot(
    const LidarIterationSnapshot& snapshot, std::string* reason,
    ObjectiveBreakdown* breakdown) const {
  WindowLinearSystem system;
  if (!blockLinearizedSystemWithLidarSnapshot(snapshot, &system, reason,
                                               breakdown))
    return std::numeric_limits<double>::infinity();
  return system.cost;
}

bool FixedLagWindow::objectiveAtStatesWithLidarSnapshotForDebug(
    const WindowStateVector& candidate_states,
    const LidarIterationSnapshot& snapshot, ObjectiveBreakdown* breakdown,
    std::string* reason) const {
  if (reason) reason->clear();
  if (!breakdown || candidate_states.empty() ||
      candidate_states.size() != states_.size()) {
    if (reason) *reason = "debug_snapshot_objective_state_shape_mismatch";
    return false;
  }
  for (std::size_t i = 0; i < candidate_states.size(); ++i) {
    const auto& state = candidate_states[i];
    if (state.stamp_ns != states_[i].stamp_ns || !state.rotation.allFinite() ||
        !state.position.allFinite() || !state.velocity.allFinite() ||
        !state.gyro_bias.allFinite() || !state.accel_bias.allFinite() ||
        (state.rotation.transpose() * state.rotation -
         Eigen::Matrix3d::Identity()).norm() >= 1e-7 ||
        std::abs(state.rotation.determinant() - 1.0) >= 1e-7) {
      if (reason) *reason = "debug_snapshot_objective_candidate_invalid";
      return false;
    }
  }
  FixedLagWindow trial = *this;
  trial.states_ = candidate_states;
  WindowLinearSystem system;
  return trial.blockLinearizedSystemWithLidarSnapshot(
      snapshot, &system, reason, breakdown);
}

bool FixedLagWindow::blockLinearizedSystemWithLidarSnapshot(
    const LidarIterationSnapshot& lidar_snapshot, WindowLinearSystem* system,
    std::string* reason, ObjectiveBreakdown* breakdown) const {
  auto fail = [&](const char* text) { if (reason) *reason = text; return false; };
  if (reason) reason->clear();
  if (!system || states_.empty()) return fail("null_or_empty_block_system");
  std::map<std::pair<std::uint64_t, std::uint64_t>,
           const FrozenLidarProjection*> projection_by_identity;
  std::set<std::uint64_t> projection_observation_ids;
  for (const auto& projection : lidar_snapshot.projections) {
    const auto identity = std::make_pair(projection.observation_id,
                                         projection.stamp_ns);
    if (!projection.valid ||
        !projection_observation_ids.insert(projection.observation_id).second ||
        !projection_by_identity.emplace(identity, &projection).second)
      return fail("invalid_or_duplicate_lidar_snapshot_identity");
  }
  if (projection_by_identity.size() != lidar_factors_.size())
    return fail("lidar_snapshot_factor_count_mismatch");
  if (breakdown) *breakdown = ObjectiveBreakdown();
  *system = WindowLinearSystem();
  const Eigen::Index dimension = states_.size()*15;
  system->gradient = Eigen::VectorXd::Zero(dimension);
  if (prior_.valid) {
    if (prior_.reference_states.size() != states_.size() ||
        prior_.information.rows() != dimension || prior_.information.cols() != dimension ||
        prior_.gradient.size() != dimension) return fail("window_prior_dimension_mismatch");
    Eigen::VectorXd d(dimension);
    std::vector<Matrix15d, Eigen::aligned_allocator<Matrix15d>> charts(states_.size(), Matrix15d::Identity());
    const double h = std::max(1e-8, options_.finite_difference_step);
    for (std::size_t i=0; i<states_.size(); ++i) {
      d.segment<15>(i*15) = localDifference(states_[i],prior_.reference_states[i]);
      for (int axis=0; axis<3; ++axis) {
        Vector15d delta = Vector15d::Zero(); delta(axis)=h;
        WindowState plus=states_[i], minus=states_[i];
        if (!applyLocalIncrement(&plus,delta,reason) || !applyLocalIncrement(&minus,-delta,reason)) return false;
        charts[i].col(axis) = (localDifference(plus,prior_.reference_states[i])-
                              localDifference(minus,prior_.reference_states[i]))/(2*h);
      }
    }
    const Eigen::VectorXd force = prior_.information*d+prior_.gradient;
    const double prior_cost = d.dot(prior_.information*d)+2*prior_.gradient.dot(d);
    system->cost = prior_cost;
    if (breakdown) breakdown->prior_cost = prior_cost;
    for (std::size_t i=0; i<states_.size(); ++i) {
      system->gradient.segment<15>(i*15) += charts[i].transpose()*force.segment<15>(i*15);
      for (std::size_t j=i; j<states_.size(); ++j) {
        const Matrix15d block = prior_.information.block<15,15>(i*15,j*15);
        if (!block.isZero(0)) system->add(i,j,charts[i].transpose()*block*charts[j]);
      }
    }
  }
  auto accumulate = [&](std::size_t i, const Eigen::MatrixXd& a,
      std::size_t j, const Eigen::MatrixXd* b, const Eigen::VectorXd& r,
      const Eigen::MatrixXd& covariance, double* component_cost) {
    if (!covariance.allFinite() ||
        (covariance-covariance.transpose()).cwiseAbs().maxCoeff() > 1e-8) return false;
    Eigen::LLT<Eigen::MatrixXd> check(covariance);
    if (check.info() != Eigen::Success) return false;
    const Eigen::MatrixXd info = covariance.ldlt().solve(Eigen::MatrixXd::Identity(covariance.rows(),covariance.cols()));
    system->add(i,i,a.transpose()*info*a);
    system->gradient.segment<15>(i*15) += a.transpose()*info*r;
    if (b) {
      system->add(j,j,b->transpose()*info*(*b));
      system->add(i,j,a.transpose()*info*(*b));
      system->gradient.segment<15>(j*15) += b->transpose()*info*r;
    }
    const double factor_cost = r.dot(info*r);
    system->cost += factor_cost;
    if (breakdown && component_cost) *component_cost += factor_cost;
    return true;
  };
  for (const auto& record : imu_factors_) {
    std::size_t i,j; Matrix15d a,b; Vector15d r;
    if (!findStateIndex(record.from_stamp_ns,&i) || !findStateIndex(record.to_stamp_ns,&j))
      return fail("imu_factor_state_removed_without_marginalization");
    if (!linearizeImuFactor(states_[i],states_[j],record.measurement,imu_noise_,&a,&b,&r,reason)) return false;
    const Eigen::MatrixXd dynamic_b=b;
    if (!accumulate(i,a,j,&dynamic_b,r,record.measurement.covariance,
                    breakdown ? &breakdown->imu_cost : nullptr))
      return fail("imu_factor_covariance_not_spd");
  }
  for (const auto& record : lidar_factors_) {
    std::size_t i; Eigen::VectorXd r; Eigen::MatrixXd a,cov;
    if (!findStateIndex(record.measurement.stamp_ns,&i)) return fail("lidar_factor_state_removed_without_marginalization");
    const auto identity = std::make_pair(record.measurement.observation_id,
                                         record.measurement.stamp_ns);
    const auto projection = projection_by_identity.find(identity);
    if (projection == projection_by_identity.end())
      return fail("lidar_snapshot_factor_identity_missing");
    if (!linearizeLidarFactorWithFrozenProjection(
            states_[i], record.measurement, *projection->second,
            &r, &a, &cov, reason)) return false;
    if (!accumulate(i,a,0,nullptr,r,cov,
                    breakdown ? &breakdown->lidar_cost : nullptr))
      return fail("lidar_factor_covariance_not_spd");
    if (breakdown) {
      // This is the exact frozen projection used for this system's H/g and
      // cost; candidate acceptance receives this same snapshot.
      const Eigen::MatrixXd information = cov.ldlt().solve(
          Eigen::MatrixXd::Identity(cov.rows(), cov.cols()));
      const double factor_cost = r.dot(information * r);
      if (record.measurement.stamp_ns >= breakdown->latest_lidar_stamp_ns) {
        breakdown->latest_lidar_stamp_ns = record.measurement.stamp_ns;
        breakdown->latest_lidar_observation_id = record.measurement.observation_id;
        breakdown->latest_lidar_factor_cost = factor_cost;
      }
      if (factor_cost > breakdown->max_single_lidar_factor_cost) {
        breakdown->max_single_lidar_factor_cost = factor_cost;
        breakdown->max_lidar_stamp_ns = record.measurement.stamp_ns;
        breakdown->max_lidar_observation_id = record.measurement.observation_id;
      }
    }
  }
  for (const auto& record : visual_factors_) {
    std::size_t i,j; Eigen::VectorXd r; Eigen::MatrixXd a,b,cov;
    if (!findStateIndex(record.measurement.reference_stamp_ns,&i) ||
        !findStateIndex(record.measurement.current_stamp_ns,&j)) return fail("visual_factor_state_removed_without_marginalization");
    if (!linearizeSelectedVisualFactor(states_[i],states_[j],record.measurement,&r,&a,&b,&cov,reason)) return false;
    if (!accumulate(i,a,j,&b,r,cov,
                    breakdown ? &breakdown->visual_cost : nullptr))
      return fail("visual_factor_covariance_not_spd");
  }
  for (auto& block : system->upper_blocks) {
    if (!block.second.allFinite()) return fail("nonfinite_window_linear_system");
    if (block.first.first == block.first.second)
      block.second = (0.5*(block.second+block.second.transpose())).eval();
  }
  if (breakdown) breakdown->total_cost = system->cost;
  return system->gradient.allFinite() && std::isfinite(system->cost);
}
}  // namespace dog_prior_map_fastlio2_frontend_exp::fixed_lag
