#pragma once

#include "dog_prior_map_fastlio2_frontend_exp/window_factors.hpp"

namespace dog_prior_map_fastlio2_frontend_exp::fixed_lag {

struct SquareRootRows {
  Eigen::MatrixXd a;
  Eigen::VectorXd b;
};

struct SquareRootQrDiagnostics {
  Eigen::Index stack_rows = 0, columns = 0;
  Eigen::Index marginalized_rank = 0;
  double marginalized_threshold = 0, r_diagonal_min = 0, r_diagonal_max = 0;
  Eigen::Index rows_before_compression = 0, rows_after_compression = 0;
  Eigen::Index active_columns = 0, compression_rank = 0;
  double compression_threshold = 0, discarded_row_jacobian_norm = 0;
  double discarded_constant_squared = 0;
  std::string status = "NOT_RUN";
};

bool factorInitialSquareRootPrior(const Matrix15d& information,
    const Vector15d& gradient, SquareRootRows* rows, std::string* reason);
bool whitenSquareRootRows(const Eigen::MatrixXd& jacobian,
    const Eigen::VectorXd& residual, const Eigen::MatrixXd& covariance,
    SquareRootRows* rows, std::string* reason);
bool eliminateSquareRootOldest(const SquareRootRows& stacked,
    Eigen::Index marginalized_dimension, SquareRootRows* retained,
    SquareRootQrDiagnostics* diagnostics, std::string* reason);

}  // namespace dog_prior_map_fastlio2_frontend_exp::fixed_lag
