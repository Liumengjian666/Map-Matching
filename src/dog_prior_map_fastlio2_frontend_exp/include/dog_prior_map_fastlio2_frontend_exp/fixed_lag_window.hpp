#pragma once

#include "dog_prior_map_fastlio2_frontend_exp/window_factors.hpp"
#include "dog_prior_map_fastlio2_frontend_exp/window_linear_system.hpp"

#include <Eigen/Core>

#include <cstdint>
#include <set>
#include <string>
#include <vector>

namespace dog_prior_map_fastlio2_frontend_exp::fixed_lag {

struct FixedLagOptions {
  double maximum_duration_s = 2.0;
  std::size_t maximum_nodes = 48;
  std::size_t maximum_active_observation_ids = 4096;
  int maximum_optimizer_iterations = 8;
  double initial_damping = 1e-6;
  double maximum_step_norm = 2.0;
  double finite_difference_step = 1e-6;
  double gradient_convergence_tolerance = 1e-9;
  WindowSolverBackend solver_backend = WindowSolverBackend::BLOCK_SPARSE;
  bool debug_rank_diagnostic = false;
  // Opt-in observability only. This must not participate in solver decisions.
  bool capture_optimizer_trace = false;
  // Opt-in marginalization forensics. It may collect matrix diagnostics but
  // must never participate in production decisions or change factor order.
  bool capture_marginalization_diagnostics = false;
};

enum class OptimizerStatus {
  NOT_RUN,
  ACCEPTED_UPDATE,
  CONVERGED_WITHOUT_STEP,
  FAILED_ALL_CANDIDATES,
  INVALID_LINEAR_SYSTEM,
};

const char* toString(OptimizerStatus status);

struct WindowSummary {
  std::size_t window_node_count = 0;
  double window_time_span_s = 0.0;
  std::size_t imu_factor_count = 0;
  std::size_t lidar_factor_count = 0;
  std::size_t visual_factor_count = 0;
  std::size_t active_observation_id_count = 0;
  std::size_t duplicate_measurement_count = 0;
  std::size_t lidar_factor_skipped_count = 0;
  int last_lidar_reliable_rank = 0;
  std::string lidar_factor_skipped_reason;
  Eigen::Index hessian_dimension = 0;
  Eigen::Index hessian_numerical_rank = 0;
  double retained_prior_cross_information_norm = 0.0;
  double marginalization_solve_jitter = 0.0;
  double marginalization_jitter_information_delta_norm = 0.0;
  double marginalization_jitter_gradient_delta_norm = 0.0;
  int optimizer_iterations = 0;
  double optimizer_initial_cost = 0.0;
  double optimizer_final_cost = 0.0;
  bool optimizer_success = false;
  bool marginalization_performed = false;
  bool marginalization_psd = false;
  bool prediction_feedback_ready = false;
  std::uint64_t window_revision = 0;
  std::uint64_t optimized_revision = 0;
  std::uint64_t retired_observation_id_watermark = 0;
  std::uint64_t latest_state_timestamp = 0;
  std::string optimizer_status = "NOT_RUN";
  std::string solver_status = "NOT_RUN";
  std::size_t sparse_solver_fallback_count = 0;
  double linearization_ms = 0, solve_ms = 0, marginal_covariance_ms = 0;
  double rank_diagnostic_ms = 0;
  std::size_t marginal_covariance_requests = 0;
  std::size_t dense_marginal_reference_requests = 0;
  std::string marginalization_status = "NOT_REQUESTED";
  std::string prediction_feedback_status = "NOT_READY";
  std::string verified_relocalization_status =
      "NOT_VERIFIED_GLOBAL_RELOCALIZATION";
};

struct WindowMarginalCovariance {
  bool valid = false;
  Matrix15d covariance15 = Matrix15d::Zero();
  Matrix6d map_pose_covariance6 = Matrix6d::Zero();
  double normalized_backward_error = 0;
  std::string status = "WINDOW_MARGINAL_COVARIANCE_UNAVAILABLE";
};

using WindowStateVector =
    std::vector<WindowState, Eigen::aligned_allocator<WindowState>>;

struct ObjectiveBreakdown {
  double prior_cost = 0.0;
  double imu_cost = 0.0;
  double lidar_cost = 0.0;
  double visual_cost = 0.0;
  double total_cost = 0.0;
  double latest_lidar_factor_cost = 0.0;
  std::uint64_t latest_lidar_stamp_ns = 0;
  std::uint64_t latest_lidar_observation_id = 0;
  double max_single_lidar_factor_cost = 0.0;
  std::uint64_t max_lidar_stamp_ns = 0;
  std::uint64_t max_lidar_observation_id = 0;
};

struct OptimizerIterationTrace {
  int iteration = 0;
  std::uint64_t lidar_snapshot_generation = 0;
  double max_projector_change_from_previous_outer = 0.0;
  std::size_t candidate_basis_relinearization_calls = 0;
  double damping_before = 0.0;
  double damping_after = 0.0;
  // These costs are the frozen-projection surrogate used for H/g and
  // candidate acceptance during this exact outer iteration.
  double surrogate_current_cost = 0.0;
  double surrogate_candidate_cost = 0.0;
  // Compatibility aliases retained for existing diagnostic consumers.
  double current_cost = 0.0;
  double gradient_inf_norm = 0.0;
  std::string solver_status = "NOT_RUN";
  double raw_step_norm = 0.0;
  double applied_step_norm = 0.0;
  bool step_clipped = false;
  double g_dot_step = 0.0;
  double step_H_step = 0.0;
  double predicted_reduction = 0.0;
  double candidate_cost = 0.0;
  double actual_reduction = 0.0;
  double rho = 0.0;
  bool rho_valid = false;
  bool accepted = false;
  ObjectiveBreakdown current_breakdown;
  ObjectiveBreakdown candidate_breakdown;
  // Retained only for opt-in failure diagnosis; empty when trace capture is off.
  Eigen::VectorXd raw_step;
  Eigen::VectorXd applied_step;
};

struct LidarIterationSnapshot {
  std::uint64_t generation = 0;
  FrozenLidarProjectionVector projections;
};

struct DirectionalDerivativeTrace {
  std::string direction_name;
  Eigen::VectorXd direction;
  double epsilon = 0.0;
  double diagnostic_relinearized_basis_fd = 0.0;
  double diagnostic_relinearized_basis_model = 0.0;
  double diagnostic_relinearized_basis_relative_error = 0.0;
  double frozen_basis_fd = 0.0;
  double frozen_basis_model = 0.0;
  double frozen_basis_relative_error = 0.0;
  bool valid = false;
  std::string status = "UNINITIALIZED";
};

struct DampingSweepTrace {
  double damping = 0.0;
  bool solved = false;
  std::string solver_status = "NOT_RUN";
  double raw_step_norm = 0.0;
  double applied_step_norm = 0.0;
  bool step_clipped = false;
  double g_dot_step = 0.0;
  double step_H_step = 0.0;
  double predicted_reduction = 0.0;
  double diagnostic_relinearized_basis_candidate_cost = 0.0;
  double diagnostic_relinearized_basis_actual_reduction = 0.0;
  double frozen_basis_candidate_cost = 0.0;
  double frozen_basis_actual_reduction = 0.0;
  double diagnostic_relinearized_basis_rho = 0.0;
  bool rho_valid = false;
  ObjectiveBreakdown diagnostic_relinearized_basis_breakdown;
  ObjectiveBreakdown frozen_basis_breakdown;
};

struct MarginalizationMatrixStats {
  bool available = false;
  bool finite = false;
  Eigen::Index dimension = 0;
  double symmetry_frobenius_norm = 0.0;
  double symmetry_max_abs = 0.0;
  double lambda_min = 0.0;
  double lambda_max = 0.0;
  double min_abs_eigenvalue = 0.0;
  double spectral_scale = 0.0;
  double frobenius_norm = 0.0;
  double relative_negative_ratio = 0.0;
  Eigen::Index negative_eigenvalue_count = 0;
  Eigen::Index numerical_rank = 0;
  std::string status = "NOT_AVAILABLE";
};

// Diagnostics only: spectrum uses the symmetric part, while the original
// symmetry defect is reported separately. Numerical rank uses eps*n*||A||2.
MarginalizationMatrixStats marginalizationMatrixStatsForDiagnostics(
    const Eigen::MatrixXd& matrix);
std::uint64_t marginalizationPriorHashForDiagnostics(
    const Eigen::MatrixXd& information, const Eigen::VectorXd& gradient);

struct MarginalizationLdltStats {
  bool available = false;
  double min_abs_d = 0.0;
  double max_abs_d = 0.0;
  double pivot_ratio = 0.0;
  double near_zero_threshold = 0.0;
  std::size_t positive_d_count = 0;
  std::size_t negative_d_count = 0;
  std::size_t near_zero_d_count = 0;
};
MarginalizationLdltStats marginalizationLdltStatsForDiagnostics(
    const Eigen::VectorXd& diagonal);

struct MarginalizationTraceRecord {
  std::uint64_t marginalization_enforcement_index = 0;
  std::size_t attempt_index_within_enforcement = 0;
  std::uint64_t latest_state_stamp_ns = 0;
  std::uint64_t oldest_state_stamp_ns = 0;
  std::size_t nodes_before_attempt = 0;
  double span_before_attempt_s = 0.0;
  std::uint64_t removed_state_stamp_ns = 0;
  bool oldest_state_removed = false;
  std::size_t nodes_after_attempt = 0;
  double span_after_attempt_s = 0.0;
  bool trigger_duration_limit = false;
  bool trigger_node_limit = false;
  std::size_t incident_imu_factor_count = 0;
  std::size_t incident_lidar_factor_count = 0;
  std::size_t incident_visual_factor_count = 0;
  bool incoming_prior_valid = false;
  std::uint64_t incoming_prior_hash_fnv1a64 = 0;
  MarginalizationMatrixStats incoming_prior;
  MarginalizationMatrixStats charted_prior;
  MarginalizationMatrixStats touching_factors;
  MarginalizationMatrixStats imu_contribution;
  MarginalizationMatrixStats lidar_contribution;
  MarginalizationMatrixStats visual_contribution;
  MarginalizationMatrixStats consumed_system;
  MarginalizationMatrixStats hmm;
  MarginalizationMatrixStats raw_schur;
  MarginalizationMatrixStats symmetrized_schur;
  MarginalizationMatrixStats new_prior;
  double ldlt_initial_min_abs_d = 0.0;
  double ldlt_initial_max_abs_d = 0.0;
  double ldlt_initial_pivot_ratio = 0.0;
  std::size_t ldlt_initial_positive_d_count = 0;
  std::size_t ldlt_initial_negative_d_count = 0;
  std::size_t ldlt_initial_near_zero_d_count = 0;
  double ldlt_solve_min_abs_d = 0.0;
  double ldlt_solve_max_abs_d = 0.0;
  double ldlt_solve_pivot_ratio = 0.0;
  std::size_t ldlt_solve_positive_d_count = 0;
  std::size_t ldlt_solve_negative_d_count = 0;
  std::size_t ldlt_solve_near_zero_d_count = 0;
  double solve_jitter = 0.0;
  double hmm_condition_proxy = 0.0;
  bool solve_finite = false;
  double solve_h_backward_error = 0.0;
  double solve_b_backward_error = 0.0;
  double raw_schur_asymmetry_frobenius_norm = 0.0;
  bool new_gradient_evaluated = false;
  bool new_gradient_finite = false;
  double new_gradient_norm = 0.0;
  double new_gradient_max_abs = 0.0;
  std::string first_bad_stage = "NONE";
  std::string marginalization_result = "NOT_RUN";
};

struct MarginalizationFactorCounts {
  std::size_t imu = 0;
  std::size_t lidar = 0;
  std::size_t visual = 0;
};

struct MarginalizationFailureCapsule {
  bool valid = false;
  MarginalizationTraceRecord trace;
  std::vector<std::uint64_t> state_stamps_before_enforcement;
  std::vector<std::uint64_t> state_stamps_at_failure;
  std::uint64_t prior_hash_before_enforcement_fnv1a64 = 0;
  std::uint64_t prior_hash_at_failure_fnv1a64 = 0;
  MarginalizationFactorCounts factors_before_enforcement;
  MarginalizationFactorCounts factors_at_failure;
  std::vector<std::uint64_t> state_stamps_at_attempt;
  Eigen::MatrixXd incoming_prior_information;
  Eigen::VectorXd incoming_prior_gradient;
  Eigen::MatrixXd charted_prior_information;
  Eigen::VectorXd charted_prior_gradient;
  Eigen::MatrixXd consumed_hessian;
  Eigen::VectorXd consumed_gradient;
  Eigen::MatrixXd imu_hessian;
  Eigen::MatrixXd lidar_hessian;
  Eigen::MatrixXd visual_hessian;
  Eigen::MatrixXd correction_h;
  Eigen::VectorXd correction_b;
};

class FixedLagWindow {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  explicit FixedLagWindow(
      const FixedLagOptions& options = {},
      const ImuNoiseParameters& imu_noise = {});

  bool addState(const WindowState& state, std::string* reason = nullptr);
  bool initializeWithPriorAtomic(const WindowState& state,
                                 const Matrix15d& information,
                                 const Vector15d& gradient,
                                 std::string* reason = nullptr);
  bool addStateWithImuFactorAtomic(
      const WindowState& state, std::uint64_t observation_id,
      std::uint64_t from_stamp_ns,
      const ImuPreintegratedMeasurement& measurement,
      std::string* reason = nullptr);
  bool addImuFactor(std::uint64_t observation_id, std::uint64_t from_stamp_ns,
                    std::uint64_t to_stamp_ns,
                    const ImuPreintegratedMeasurement& measurement,
                    std::string* reason = nullptr);
  bool addLidarFactor(const LidarWindowMeasurement& measurement,
                      std::string* reason = nullptr);
  bool addVisualFactor(const VisualRelativeMeasurement& measurement,
                       std::string* reason = nullptr);

  bool setInitialPrior(std::uint64_t stamp_ns,
                       const Matrix15d& information,
                       const Vector15d& gradient,
                       std::string* reason = nullptr);

  bool optimize(std::string* reason = nullptr);
  bool marginalizeIfNeeded(std::string* reason = nullptr);

  const std::vector<WindowState, Eigen::aligned_allocator<WindowState>>& states()
      const;
  const WindowState* stateAt(std::uint64_t stamp_ns) const;
  const WindowState* latestState() const;
  WindowSummary summary() const;
  const Eigen::MatrixXd& priorInformation() const;
  const Eigen::VectorXd& priorGradient() const;
  bool linearizedSystem(Eigen::MatrixXd* hessian,
                        Eigen::VectorXd* gradient, double* cost,
                        std::string* reason = nullptr) const;
  bool blockLinearizedSystem(WindowLinearSystem* system,
                             std::string* reason = nullptr,
                             ObjectiveBreakdown* breakdown = nullptr) const;
  bool buildLidarIterationSnapshot(
      LidarIterationSnapshot* snapshot, std::string* reason = nullptr,
      std::uint64_t generation = 0) const;
  bool blockLinearizedSystemWithLidarSnapshot(
      const LidarIterationSnapshot& snapshot, WindowLinearSystem* system,
      std::string* reason = nullptr,
      ObjectiveBreakdown* breakdown = nullptr) const;
  double objectiveWithLidarSnapshot(
      const LidarIterationSnapshot& snapshot, std::string* reason = nullptr,
      ObjectiveBreakdown* breakdown = nullptr) const;
  bool objectiveAtStatesWithLidarSnapshotForDebug(
      const WindowStateVector& candidate_states,
      const LidarIterationSnapshot& snapshot, ObjectiveBreakdown* breakdown,
      std::string* reason = nullptr) const;
  bool objectiveBreakdownForDebug(ObjectiveBreakdown* output,
                                  std::string* reason = nullptr) const;
  bool objectiveBreakdownAtStatesForDebug(
      const WindowStateVector& candidate_states, bool freeze_lidar_bases_at_x0,
      ObjectiveBreakdown* output, std::string* reason = nullptr) const;
  bool diagnoseOptimizerFailureForDebug(
      const std::vector<double>& epsilons,
      const std::vector<double>& damping_values,
      ObjectiveBreakdown* start_breakdown,
      std::vector<DirectionalDerivativeTrace>* derivatives,
      std::vector<DampingSweepTrace>* damping_sweep,
      double* maximum_state_difference,
      std::string* reason = nullptr) const;
  const std::vector<OptimizerIterationTrace>& optimizerTraceForDebug() const;
  const std::vector<MarginalizationTraceRecord>&
  marginalizationTraceForDiagnostics() const;
  const MarginalizationFailureCapsule&
  marginalizationFailureCapsuleForDiagnostics() const;
  bool latestMarginalCovariance(WindowMarginalCovariance* output,
                               std::string* reason = nullptr) const;
  bool latestMarginalCovarianceDenseReferenceForTest(
      WindowMarginalCovariance* output, std::string* reason = nullptr) const;
  bool predictionFeedbackSeed(WindowState* output,
                              std::string* reason = nullptr) const;

 private:
  struct ImuFactorRecord {
    std::uint64_t observation_id = 0;
    std::uint64_t from_stamp_ns = 0;
    std::uint64_t to_stamp_ns = 0;
    ImuPreintegratedMeasurement measurement;
  };
  struct LidarFactorRecord { LidarWindowMeasurement measurement; };
  struct VisualFactorRecord { VisualRelativeMeasurement measurement; };
  struct PriorInformation {
    Eigen::MatrixXd information;
    Eigen::VectorXd gradient;
    std::vector<WindowState, Eigen::aligned_allocator<WindowState>> reference_states;
    bool valid = false;
  };

  struct MarginalizationAssemblyDiagnostics {
    Eigen::MatrixXd prior_hessian;
    Eigen::VectorXd prior_gradient;
    Eigen::MatrixXd imu_hessian;
    Eigen::VectorXd imu_gradient;
    Eigen::MatrixXd lidar_hessian;
    Eigen::VectorXd lidar_gradient;
    Eigen::MatrixXd visual_hessian;
    Eigen::VectorXd visual_gradient;
  };

  enum class LinearizationScope {
    ALL_FACTORS,
    FACTORS_TOUCHING_OLDEST,
  };

  bool findStateIndex(std::uint64_t stamp_ns, std::size_t* index) const;
  bool linearizeSelected(LinearizationScope scope, Eigen::MatrixXd* hessian,
                         Eigen::VectorXd* gradient, double* cost,
                         std::string* reason,
                         MarginalizationAssemblyDiagnostics* diagnostics =
                             nullptr) const;
  bool linearize(Eigen::MatrixXd* hessian, Eigen::VectorXd* gradient,
                 double* cost, std::string* reason) const;
  bool linearizeMarginalizationSubgraph(Eigen::MatrixXd* hessian,
                                        Eigen::VectorXd* gradient,
                                        double* cost,
                                        std::string* reason,
      MarginalizationAssemblyDiagnostics* diagnostics = nullptr) const;
  double objective(std::string* reason,
                   ObjectiveBreakdown* breakdown = nullptr) const;
  bool evaluateObjectiveForDebug(
      const WindowStateVector& candidate_states,
      bool freeze_lidar_bases_at_x0, ObjectiveBreakdown* output,
      std::string* reason) const;
  bool applyGlobalIncrement(const Eigen::VectorXd& increment,
                            std::string* reason);
  bool marginalizeOldest(std::string* reason);
  bool observationIdAvailable(std::uint64_t observation_id,
                              std::string* reason);
  void retireObservationId(std::uint64_t observation_id);
  void markWindowMutated();

  FixedLagOptions options_;
  ImuNoiseParameters imu_noise_;
  WindowStateVector states_;
  std::vector<ImuFactorRecord> imu_factors_;
  std::vector<LidarFactorRecord> lidar_factors_;
  std::vector<VisualFactorRecord> visual_factors_;
  // Observation IDs are a single globally monotonic stream across factor
  // families. Active IDs are bounded explicitly; the scalar retired watermark
  // rejects replay without retaining an unbounded historical set.
  std::set<std::uint64_t> active_observation_ids_;
  std::uint64_t retired_observation_id_watermark_ = 0;
  std::uint64_t window_revision_ = 0;
  std::uint64_t optimized_revision_ = 0;
  PriorInformation prior_;
  mutable WindowSummary summary_;
  std::vector<OptimizerIterationTrace> optimizer_trace_;
  std::vector<MarginalizationTraceRecord> marginalization_trace_;
  MarginalizationFailureCapsule marginalization_failure_capsule_;
  std::uint64_t marginalization_enforcement_index_ = 0;
  std::size_t marginalization_attempt_index_ = 0;
  bool marginalization_trigger_duration_limit_ = false;
  bool marginalization_trigger_node_limit_ = false;
  std::vector<std::uint64_t> enforcement_state_stamps_before_;
  std::uint64_t enforcement_prior_hash_before_ = 0;
  MarginalizationFactorCounts enforcement_factor_counts_before_;
};

// Applies one stacked 15D local increment per state as a transaction.  On any
// failure the entire aligned state vector is restored byte-for-byte at the
// semantic field level; no prefix of states remains updated.
bool applyGlobalIncrementAtomically(
    std::vector<WindowState, Eigen::aligned_allocator<WindowState>>* states,
    const Eigen::VectorXd& increment, std::string* reason = nullptr);

}  // namespace dog_prior_map_fastlio2_frontend_exp::fixed_lag
