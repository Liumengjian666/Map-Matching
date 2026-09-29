#pragma once

#include "dog_prior_map_fastlio2_frontend_exp/window_factors.hpp"

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
  std::string marginalization_status = "NOT_REQUESTED";
  std::string prediction_feedback_status = "NOT_READY";
  std::string verified_relocalization_status =
      "NOT_VERIFIED_GLOBAL_RELOCALIZATION";
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

  enum class LinearizationScope {
    ALL_FACTORS,
    FACTORS_TOUCHING_OLDEST,
  };

  bool findStateIndex(std::uint64_t stamp_ns, std::size_t* index) const;
  bool linearizeSelected(LinearizationScope scope, Eigen::MatrixXd* hessian,
                         Eigen::VectorXd* gradient, double* cost,
                         std::string* reason) const;
  bool linearize(Eigen::MatrixXd* hessian, Eigen::VectorXd* gradient,
                 double* cost, std::string* reason) const;
  bool linearizeMarginalizationSubgraph(Eigen::MatrixXd* hessian,
                                        Eigen::VectorXd* gradient,
                                        double* cost,
                                        std::string* reason) const;
  double objective(std::string* reason) const;
  bool applyGlobalIncrement(const Eigen::VectorXd& increment,
                            std::string* reason);
  bool marginalizeOldest(std::string* reason);
  bool observationIdAvailable(std::uint64_t observation_id,
                              std::string* reason);
  void retireObservationId(std::uint64_t observation_id);
  void markWindowMutated();

  FixedLagOptions options_;
  ImuNoiseParameters imu_noise_;
  std::vector<WindowState, Eigen::aligned_allocator<WindowState>> states_;
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
  WindowSummary summary_;
};

// Applies one stacked 15D local increment per state as a transaction.  On any
// failure the entire aligned state vector is restored byte-for-byte at the
// semantic field level; no prefix of states remains updated.
bool applyGlobalIncrementAtomically(
    std::vector<WindowState, Eigen::aligned_allocator<WindowState>>* states,
    const Eigen::VectorXd& increment, std::string* reason = nullptr);

}  // namespace dog_prior_map_fastlio2_frontend_exp::fixed_lag
