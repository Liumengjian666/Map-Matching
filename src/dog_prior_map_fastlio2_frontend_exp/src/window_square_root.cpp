#include "dog_prior_map_fastlio2_frontend_exp/window_square_root.hpp"

#include <Eigen/Cholesky>
#include <Eigen/QR>
#include <algorithm>
#include <limits>

namespace dog_prior_map_fastlio2_frontend_exp::fixed_lag {
namespace {
bool fail(std::string* reason, const char* message) {
  if (reason) *reason = message;
  return false;
}
bool finiteRows(const SquareRootRows& rows) {
  return rows.a.rows() == rows.b.size() && rows.a.allFinite() && rows.b.allFinite();
}
bool symmetricAtRoundoffScale(const Eigen::MatrixXd& matrix) {
  if (matrix.rows() == 0 || matrix.rows() != matrix.cols() || !matrix.allFinite()) return false;
  const double scale = matrix.cwiseAbs().maxCoeff();
  const double threshold = std::numeric_limits<double>::epsilon() *
      static_cast<double>(matrix.rows()) * scale;
  return (matrix - matrix.transpose()).cwiseAbs().maxCoeff() <= threshold;
}
}  // namespace

bool factorInitialSquareRootPrior(const Matrix15d& information,
    const Vector15d& gradient, SquareRootRows* rows, std::string* reason) {
  if (reason) reason->clear();
  if (!rows || !symmetricAtRoundoffScale(information) || !gradient.allFinite() ||
      (information - information.transpose()).cwiseAbs().maxCoeff() > 1e-10)
    return fail(reason, "square_root_initial_prior_factorization_failed");
  Eigen::LLT<Matrix15d> llt(information);
  if (llt.info() != Eigen::Success)
    return fail(reason, "square_root_initial_prior_factorization_failed");
  SquareRootRows candidate;
  candidate.a = llt.matrixL().transpose();
  candidate.b = llt.matrixL().solve(gradient);
  if (!finiteRows(candidate))
    return fail(reason, "square_root_initial_prior_factorization_failed");
  *rows = std::move(candidate);
  return true;
}

bool whitenSquareRootRows(const Eigen::MatrixXd& jacobian,
    const Eigen::VectorXd& residual, const Eigen::MatrixXd& covariance,
    SquareRootRows* rows, std::string* reason) {
  if (!rows || jacobian.rows() == 0 || jacobian.rows() != residual.size() ||
      covariance.rows() != residual.size() || covariance.cols() != residual.size() ||
      !jacobian.allFinite() || !residual.allFinite() || !symmetricAtRoundoffScale(covariance) ||
      (covariance - covariance.transpose()).cwiseAbs().maxCoeff() > 1e-8)
    return fail(reason, "square_root_factor_invalid_rows_or_covariance");
  Eigen::LLT<Eigen::MatrixXd> llt(covariance);
  if (llt.info() != Eigen::Success)
    return fail(reason, "square_root_factor_covariance_not_spd");
  SquareRootRows candidate;
  candidate.a = llt.matrixL().solve(jacobian);
  candidate.b = llt.matrixL().solve(residual);
  if (!finiteRows(candidate)) return fail(reason, "square_root_whitening_nonfinite");
  *rows = std::move(candidate);
  return true;
}

bool eliminateSquareRootOldest(const SquareRootRows& stacked,
    Eigen::Index marginalized_dimension, SquareRootRows* retained,
    SquareRootQrDiagnostics* diagnostics, std::string* reason) {
  if (reason) reason->clear();
  if (!retained || !diagnostics) return fail(reason, "square_root_null_output");
  *diagnostics = SquareRootQrDiagnostics();
  diagnostics->stack_rows = stacked.a.rows();
  diagnostics->columns = stacked.a.cols();
  if (!finiteRows(stacked) || marginalized_dimension <= 0 ||
      stacked.a.cols() <= marginalized_dimension)
    return fail(reason, "square_root_invalid_stack");
  if (stacked.a.rows() < marginalized_dimension)
    return fail(reason, "square_root_marginalized_block_rank_deficient");
  const Eigen::MatrixXd marginalized = stacked.a.leftCols(marginalized_dimension);
  Eigen::ColPivHouseholderQR<Eigen::MatrixXd> qr(marginalized);
  const double relative_threshold = std::numeric_limits<double>::epsilon() *
      static_cast<double>(std::max(marginalized.rows(), marginalized.cols()));
  qr.setThreshold(relative_threshold);
  diagnostics->marginalized_rank = qr.rank();
  diagnostics->marginalized_threshold = relative_threshold * qr.maxPivot();
  const Eigen::VectorXd diagonal = qr.matrixR().topLeftCorner(
      marginalized_dimension, marginalized_dimension).diagonal().cwiseAbs();
  diagnostics->r_diagonal_min = diagonal.minCoeff();
  diagnostics->r_diagonal_max = diagonal.maxCoeff();
  if (qr.rank() != marginalized_dimension) {
    diagnostics->status = "RANK_DEFICIENT";
    return fail(reason, "square_root_marginalized_block_rank_deficient");
  }
  // A_m P=Q R. Permuting eliminated coordinates does not permute A_r.
  Eigen::MatrixXd augmented(stacked.a.rows(), stacked.a.cols() - marginalized_dimension + 1);
  augmented.leftCols(augmented.cols() - 1) = stacked.a.rightCols(stacked.a.cols() - marginalized_dimension);
  augmented.rightCols(1) = stacked.b;
  const Eigen::MatrixXd transformed = qr.householderQ().adjoint() * augmented;
  const Eigen::Index remaining_rows = stacked.a.rows() - marginalized_dimension;
  const Eigen::MatrixXd a_ret = transformed.bottomRows(remaining_rows).leftCols(augmented.cols() - 1);
  const Eigen::VectorXd b_ret = transformed.bottomRows(remaining_rows).rightCols(1);
  diagnostics->rows_before_compression = remaining_rows;
  diagnostics->active_columns = 0;
  for (Eigen::Index column = 0; column < a_ret.cols(); ++column)
    if (!a_ret.col(column).isZero(0)) ++diagnostics->active_columns;
  SquareRootRows candidate;
  if (remaining_rows == 0) {
    candidate.a = Eigen::MatrixXd::Zero(0, a_ret.cols());
    candidate.b.resize(0);
  } else {
    // Row-space compression stays in square-root coordinates; no A^T A.
    Eigen::ColPivHouseholderQR<Eigen::MatrixXd> compression(a_ret);
    const double relative = std::numeric_limits<double>::epsilon() *
        static_cast<double>(std::max(a_ret.rows(), a_ret.cols()));
    compression.setThreshold(relative);
    const Eigen::Index rank = compression.rank();
    diagnostics->compression_rank = rank;
    diagnostics->compression_threshold = relative * compression.maxPivot();
    Eigen::MatrixXd r = Eigen::MatrixXd::Zero(a_ret.rows(), a_ret.cols());
    r = compression.matrixR().template triangularView<Eigen::Upper>();
    const Eigen::VectorXd transformed_b = compression.householderQ().adjoint() * b_ret;
    candidate.a = r.topRows(rank) * compression.colsPermutation().transpose();
    candidate.b = transformed_b.head(rank);
    diagnostics->discarded_row_jacobian_norm = r.bottomRows(r.rows() - rank).norm();
    diagnostics->discarded_constant_squared = transformed_b.tail(transformed_b.size() - rank).squaredNorm();
  }
  if (!finiteRows(candidate)) return fail(reason, "square_root_qr_nonfinite");
  diagnostics->rows_after_compression = candidate.a.rows();
  diagnostics->status = "SUCCESS_SQUARE_ROOT_QR";
  *retained = std::move(candidate);
  return true;
}
}  // namespace dog_prior_map_fastlio2_frontend_exp::fixed_lag
