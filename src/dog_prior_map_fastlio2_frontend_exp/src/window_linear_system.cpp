#include "dog_prior_map_fastlio2_frontend_exp/fixed_lag_window.hpp"
#include <Eigen/Cholesky>
#include <Eigen/SparseCholesky>
#include <cmath>

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

bool FixedLagWindow::blockLinearizedSystem(WindowLinearSystem* system, std::string* reason) const {
  auto fail = [&](const char* text) { if (reason) *reason = text; return false; };
  if (reason) reason->clear();
  if (!system || states_.empty()) return fail("null_or_empty_block_system");
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
    system->cost = d.dot(prior_.information*d)+2*prior_.gradient.dot(d);
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
      const Eigen::MatrixXd& covariance) {
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
    system->cost += r.dot(info*r);
    return true;
  };
  for (const auto& record : imu_factors_) {
    std::size_t i,j; Matrix15d a,b; Vector15d r;
    if (!findStateIndex(record.from_stamp_ns,&i) || !findStateIndex(record.to_stamp_ns,&j))
      return fail("imu_factor_state_removed_without_marginalization");
    if (!linearizeImuFactor(states_[i],states_[j],record.measurement,imu_noise_,&a,&b,&r,reason)) return false;
    const Eigen::MatrixXd dynamic_b=b;
    if (!accumulate(i,a,j,&dynamic_b,r,record.measurement.covariance)) return fail("imu_factor_covariance_not_spd");
  }
  for (const auto& record : lidar_factors_) {
    std::size_t i; Eigen::VectorXd r; Eigen::MatrixXd a,cov;
    if (!findStateIndex(record.measurement.stamp_ns,&i)) return fail("lidar_factor_state_removed_without_marginalization");
    if (!linearizeLidarFactor(states_[i],record.measurement,&r,&a,&cov,reason)) return false;
    if (!accumulate(i,a,0,nullptr,r,cov)) return fail("lidar_factor_covariance_not_spd");
  }
  for (const auto& record : visual_factors_) {
    std::size_t i,j; Eigen::VectorXd r; Eigen::MatrixXd a,b,cov;
    if (!findStateIndex(record.measurement.reference_stamp_ns,&i) ||
        !findStateIndex(record.measurement.current_stamp_ns,&j)) return fail("visual_factor_state_removed_without_marginalization");
    if (!linearizeSelectedVisualFactor(states_[i],states_[j],record.measurement,&r,&a,&b,&cov,reason)) return false;
    if (!accumulate(i,a,j,&b,r,cov)) return fail("visual_factor_covariance_not_spd");
  }
  for (auto& block : system->upper_blocks) {
    if (!block.second.allFinite()) return fail("nonfinite_window_linear_system");
    if (block.first.first == block.first.second)
      block.second = (0.5*(block.second+block.second.transpose())).eval();
  }
  return system->gradient.allFinite() && std::isfinite(system->cost);
}
}  // namespace dog_prior_map_fastlio2_frontend_exp::fixed_lag
