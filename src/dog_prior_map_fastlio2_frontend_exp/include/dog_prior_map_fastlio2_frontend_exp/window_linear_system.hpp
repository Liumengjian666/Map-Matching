#pragma once
#include "dog_prior_map_fastlio2_frontend_exp/window_factors.hpp"
#include <Eigen/SparseCore>
#include <map>

namespace dog_prior_map_fastlio2_frontend_exp::fixed_lag {
enum class WindowSolverBackend { BLOCK_SPARSE, DENSE_REFERENCE };
struct WindowLinearSystem {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  // Only upper state blocks are stored. A Schur prior may make these dense.
  std::map<std::pair<std::size_t, std::size_t>, Matrix15d> upper_blocks;
  Eigen::VectorXd gradient;
  double cost = 0;
  void add(std::size_t row, std::size_t col, const Matrix15d& block);
  Eigen::MatrixXd dense() const;
  Eigen::SparseMatrix<double> sparse() const;
  std::size_t payloadBytes() const;
};
bool solveWindowLinearSystem(const WindowLinearSystem& system, double damping,
    WindowSolverBackend backend, Eigen::VectorXd* step, std::string* status);
}  // namespace dog_prior_map_fastlio2_frontend_exp::fixed_lag
