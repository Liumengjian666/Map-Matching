#pragma once

#include "dog_prior_map_fastlio2_frontend_exp/dual_reliability.hpp"
#include "dog_prior_map_fastlio2_frontend_exp/fixed_lag_experiment.hpp"
#include "dog_prior_map_fastlio2_frontend_exp/measurement_provenance.hpp"

#include <Eigen/Core>

#include <cstdint>
#include <map>
#include <set>
#include <string>
#include <tuple>
#include <vector>

namespace dog_prior_map_fastlio2_frontend_exp::fixed_lag {

// These are immutable, already validated sensor products.  The adapter never
// calls NDT or re-runs the visual frontend; it only converts the products into
// factors in the existing 15D fixed-lag window.
struct FrozenLidarEvent {
  std::uint64_t transaction_id = 0;
  std::uint64_t stamp_ns = 0;
  Pose3d map_T_lidar;
  Matrix6d residual_covariance = Matrix6d::Identity();
  reliability::LocalRisk local_risk;
  bool ndt_converged = false;
  bool map_support_valid = false;
  bool measurement_commit_allowed = true;
};

// Explicit relocalization input. Old-window selected NIS is diagnostic only:
// this request is not an addLidarFactor operation.
struct RecoveryReinitializationRequest {
  FrozenLidarEvent registration;
  Pose3d initial_map_T_lidar;
  int iterations = 0;
  int maximum_iterations = 80;
  double objective = 0.0;
  double fitness = 0.0;
  bool measurement_covariance_available = false;
  WindowMarginalCovariance premeasurement_covariance;
};

bool validateRecoveryRegistration(const RecoveryReinitializationRequest& request,
                                  std::string* reason = nullptr);

struct FrozenVisualEvent {
  std::uint64_t ref_ns = 0;
  std::uint64_t cur_ns = 0;
  std::uint64_t depth_ns = 0;
  Eigen::Vector3d translation_ref_imu = Eigen::Vector3d::Zero();
  Eigen::Matrix3d measurement_covariance = Eigen::Matrix3d::Identity();
  reliability::VisualQualityObservation quality;
  bool source_valid = false;
  VisualMeasurementProvenance provenance = VisualMeasurementProvenance::UNKNOWN;
};

struct FixedLagAdapterCalibration {
  // T_imu_lidar maps LiDAR coordinates into the IMU body coordinates.
  Pose3d T_imu_lidar;
  reliability::DualReliabilityConfig reliability_config;
  // Historical adapters default to compatibility. V3 explicitly sets false.
  bool allow_compatibility_visual_inputs = true;
};

enum class AdapterEventDisposition {
  ACCEPTED,
  DUPLICATE_SOURCE,
  SKIPPED_INVALID_SOURCE,
  REJECTED_CAUSALITY,
  REJECTED_WINDOW,
  REJECTED_FACTOR,
};

struct AdapterEventStatus {
  AdapterEventDisposition disposition = AdapterEventDisposition::REJECTED_FACTOR;
  std::uint64_t observation_id = 0;
  std::string reason = "NOT_PROCESSED";
};

struct AdapterLifecycleDiagnostics {
  std::size_t peak_imu_buffer_size = 0;
  std::size_t peak_active_source_records = 0;
  std::size_t expired_source_records_removed = 0;
  bool last_imu_factor_covariance_regularized = false;
  double last_imu_regularization = 0.0;
  double last_imu_physical_min_eigenvalue = 0.0;
  double last_imu_factor_min_eigenvalue = 0.0;
  double last_imu_information_change_norm = 0.0;
};

const char* toString(AdapterEventDisposition disposition);

// Causal adapter around the already-tested FixedLagExperimentalController.
// Internal observation IDs are allocated only after an event has passed all
// source, timestamp and measurement checks.  Raw transaction IDs and visual
// endpoint timestamps are source keys, never window observation IDs.
class FixedLagEventAdapter {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  FixedLagEventAdapter(
      const FixedLagOptions& options = {},
      const ImuNoiseParameters& imu_noise = {},
      const FixedLagAdapterCalibration& calibration = {});

  bool initialize(const WindowState& initial_state,
                  const Matrix15d& initial_information,
                  const Vector15d& initial_gradient,
                  std::string* reason = nullptr);

  bool appendImu(const ImuSample& sample, std::string* reason = nullptr);
  bool prepareStateAt(std::uint64_t stamp_ns, WindowState* predicted,
                      std::string* reason = nullptr);
  bool activeStateAt(std::uint64_t stamp_ns, WindowState* state,
                     std::string* reason = nullptr) const;
  bool latestMarginalCovariance(WindowMarginalCovariance* output,
                                std::string* reason = nullptr) const;
  bool previewLidarMeasurement(const FrozenLidarEvent& event,
                               LidarWindowMeasurement* output,
                               std::string* reason = nullptr) const;
  bool processLidarEvent(const FrozenLidarEvent& event,
                         std::string* reason = nullptr);
  bool resetFromValidatedRecovery(const RecoveryReinitializationRequest& request,
                                  std::string* reason = nullptr);
  bool processVisualReferenceStamp(std::uint64_t ref_ns,
                                   std::string* reason = nullptr);
  bool processVisualEvent(const FrozenVisualEvent& event,
                          std::string* reason = nullptr);
  bool optimizeCurrentWindow(std::string* reason = nullptr);
  bool latestOptimizedState(WindowState* output,
                            std::string* reason = nullptr) const;

  WindowSummary summary() const;
  const AdapterEventStatus& lastEventStatus() const;
  std::uint64_t nextObservationId() const;
  std::size_t sourceRecordCount() const;
  std::size_t imuSampleCount() const;
  AdapterLifecycleDiagnostics lifecycleDiagnostics() const;
  const VisualRelativeMeasurement& lastVisualAdmission() const;
  const FixedLagWindow* debugWindowForDiagnostics() const;

 private:
  using SourceKey = std::tuple<unsigned char, std::uint64_t, std::uint64_t>;

  bool fail(AdapterEventDisposition disposition, const std::string& reason,
            std::string* output_reason);
  bool accept(std::uint64_t observation_id, std::string* output_reason);
  bool sourceSeen(const SourceKey& key) const;
  void recordSource(const SourceKey& key, std::uint64_t expiry_stamp_ns);
  void pruneExpiredHistory();
  bool ensureStateAt(std::uint64_t stamp_ns, std::string* reason);
  bool buildPredictedState(std::uint64_t stamp_ns, WindowState* output,
                           ImuPreintegratedMeasurement* preintegrated,
                           std::string* reason) const;
  bool convertLidarMeasurement(const FrozenLidarEvent& event,
                               const WindowState& predicted_state,
                               LidarWindowMeasurement* output,
                               std::string* reason) const;
  bool rebuildLidarBasis(const FrozenLidarEvent& event,
                         const WindowState& state, Matrix6d* basis, int* rank,
                         std::string* reason) const;
  Pose3d lidarToImu(const Pose3d& map_T_lidar) const;
  bool configureVisualDirection(const FrozenVisualEvent& event,
                                VisualRelativeMeasurement* measurement,
                                std::string* reason) const;
  std::uint64_t allocateObservationId();
  void setStatus(AdapterEventDisposition disposition, std::uint64_t id,
                 const std::string& reason, std::string* output_reason);

  FixedLagExperimentalController controller_;
  FixedLagAdapterCalibration calibration_;
  ImuNoiseParameters imu_noise_;
  std::vector<ImuSample, Eigen::aligned_allocator<ImuSample>> imu_samples_;
  std::set<SourceKey> source_records_;
  std::map<SourceKey, std::uint64_t> source_expiry_stamps_;
  struct LidarRiskRecord {
    std::uint64_t transaction_id = 0;
    std::uint64_t stamp_ns = 0;
    Pose3d map_T_lidar;
    reliability::LocalRisk risk;
    bool map_support_valid = false;
    bool ndt_converged = false;
    bool factor_committed = false;
  };
  std::vector<LidarRiskRecord> lidar_risk_history_;
  std::uint64_t lidar_transaction_watermark_ = 0;
  std::uint64_t last_lidar_stamp_ns_ = 0;
  std::uint64_t next_observation_id_ = 1;
  std::uint64_t last_imu_stamp_ns_ = 0;
  bool initialized_ = false;
  AdapterEventStatus last_status_;
  VisualRelativeMeasurement last_visual_admission_;
  mutable AdapterLifecycleDiagnostics lifecycle_diagnostics_;
};

}  // namespace dog_prior_map_fastlio2_frontend_exp::fixed_lag
