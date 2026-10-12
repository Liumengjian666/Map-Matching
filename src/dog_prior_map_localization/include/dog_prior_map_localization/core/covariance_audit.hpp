#pragma once

#include <algorithm>
#include <cmath>
#include <deque>
#include <fstream>
#include <iomanip>
#include <map>
#include <stdexcept>
#include <string>

#include <Eigen/Eigenvalues>
#include "dog_prior_map_localization/core/estimator_types.hpp"
#include "dog_prior_map_localization/core/positive_covariance_factor.hpp"

namespace dog_prior_map_localization
{
// Opt-in P26 diagnostic: aggregate all checks, retain only a bounded first-fault
// neighborhood and one before/after matrix pair. Never changes covariance.
class CovarianceAudit
{
public:
  struct Metrics { double minimum, asymmetry, trace; bool valid; double bound=NAN; bool certified=false; };
  struct Entry { std::string event; double stamp; Metrics before, after; };
  struct Aggregate {
    size_t count = 0, invalid_before = 0, invalid_after = 0;
    double minimum_before = INFINITY, minimum_after = INFINITY, max_asymmetry = 0.0;
    double minimum_bound_before=INFINITY, minimum_bound_after=INFINITY;
    size_t uncertified_before=0, uncertified_after=0;
  };
  ~CovarianceAudit() { finish(); }
  bool enabled() const { return stream_.is_open(); }
  void useFactorCertificate(bool enabled) { factor_certificate_=enabled; }
  void captureAt(double stamp) { reference_capture_stamp_=stamp; }
  void open(const std::string& path)
  {
    if (path.empty()) return;
    std::ifstream existing(path);
    if (existing.good()) throw std::runtime_error("covariance audit output already exists");
    path_ = path; stream_.open(path);
    if (!stream_) throw std::runtime_error("covariance audit output unavailable");
    stream_ << "event,stamp,before_min_eigenvalue,after_min_eigenvalue,before_asymmetry,after_asymmetry,before_trace,after_trace,before_valid,after_valid,before_positive_bound,after_positive_bound,before_certified,after_certified\n";
  }
  static Metrics inspect(const Matrix15d& covariance)
  {
    Metrics m{NAN, INFINITY, covariance.trace(), false};
    if (!covariance.allFinite()) return m;
    m.asymmetry = (covariance-covariance.transpose()).cwiseAbs().maxCoeff();
    const Matrix15d symmetric = 0.5*(covariance+covariance.transpose());
    Eigen::SelfAdjointEigenSolver<Matrix15d> eigen(symmetric, Eigen::EigenvaluesOnly);
    if (eigen.info()!=Eigen::Success || !eigen.eigenvalues().allFinite()) return m;
    m.minimum = eigen.eigenvalues().minCoeff();
    // Symmetry is reported independently; the first-fault capture must locate
    // PSD loss rather than a scale-dependent absolute symmetry threshold.
    m.valid = m.minimum >= -1e-10;
    const auto certificate=factorPositiveCovariance(covariance);
    m.certified=certificate.valid; m.bound=certificate.minimum_eigenvalue_lower_bound;
    return m;
  }
  void check(const std::string& event, double stamp,
             const Matrix15d& before, const Matrix15d& after)
  {
    if (!enabled()) return;
    Entry row{event, stamp, inspect(before), inspect(after)};
    if (factor_certificate_) { row.before.valid=row.before.certified; row.after.valid=row.after.certified; }
    auto& a = aggregates_[event]; ++a.count;
    a.invalid_before += !row.before.valid; a.invalid_after += !row.after.valid;
    a.minimum_before = std::min(a.minimum_before,row.before.minimum);
    a.minimum_after = std::min(a.minimum_after,row.after.minimum);
    a.max_asymmetry = std::max(a.max_asymmetry,std::max(row.before.asymmetry,row.after.asymmetry));
    a.minimum_bound_before=std::min(a.minimum_bound_before,row.before.bound);
    a.minimum_bound_after=std::min(a.minimum_bound_after,row.after.bound);
    a.uncertified_before+=!row.before.certified; a.uncertified_after+=!row.after.certified;
    if (!reference_captured_ && reference_capture_stamp_>0.0 && event=="SKF_UPDATE" &&
        stamp>=reference_capture_stamp_-1e-6) {
      reference_captured_=true;
      saveMatrices(path_+".reference_matrix.csv",before,after);
      std::ofstream receipt(path_+".reference_event.csv");
      receipt<<"event,stamp,before_bound,after_bound,before_certified,after_certified\n"<<std::setprecision(17)
             <<event<<','<<stamp<<','<<row.before.bound<<','<<row.after.bound<<','
             <<row.before.certified<<','<<row.after.certified<<'\n';
    }
    recent_.push_back(row); if (recent_.size()>12) recent_.pop_front();
    if (!fault_seen_ && (!row.before.valid || !row.after.valid))
    {
      fault_seen_ = true; following_ = 12;
      for (const auto& prior : recent_) write(prior);
      saveMatrices(path_+".first_matrix.csv",before,after);
    }
    else if (fault_seen_ && following_>0) { write(row); --following_; }
  }
  void finish()
  {
    if (!enabled()) return;
    std::ofstream summary(path_+".summary.csv");
    summary << "event,count,invalid_before,invalid_after,min_before,min_after,max_asymmetry,min_positive_bound_before,min_positive_bound_after,uncertified_before,uncertified_after\n" << std::setprecision(17);
    for (const auto& item : aggregates_) {
      const auto& a=item.second;
      summary << item.first<<','<<a.count<<','<<a.invalid_before<<','<<a.invalid_after<<','
              <<a.minimum_before<<','<<a.minimum_after<<','<<a.max_asymmetry<<','
              <<a.minimum_bound_before<<','<<a.minimum_bound_after<<','<<a.uncertified_before<<','<<a.uncertified_after<<'\n';
    }
    stream_.close();
  }
private:
  static void saveMatrices(const std::string& path,const Matrix15d& before,const Matrix15d& after) {
    std::ofstream matrix(path); matrix<<"stage,row,column,value\n"<<std::setprecision(17);
    for(int i=0;i<15;++i)for(int j=0;j<15;++j)
      matrix<<"before,"<<i<<','<<j<<','<<before(i,j)<<'\n'
            <<"after,"<<i<<','<<j<<','<<after(i,j)<<'\n';
  }
  void write(const Entry& r) {
    stream_<<std::setprecision(17)<<r.event<<','<<r.stamp<<','<<r.before.minimum<<','<<r.after.minimum<<','
           <<r.before.asymmetry<<','<<r.after.asymmetry<<','<<r.before.trace<<','<<r.after.trace<<','
           <<r.before.valid<<','<<r.after.valid<<','<<r.before.bound<<','<<r.after.bound<<','
           <<r.before.certified<<','<<r.after.certified<<'\n'; stream_.flush();
  }
  std::string path_; std::ofstream stream_; std::map<std::string,Aggregate> aggregates_;
  std::deque<Entry> recent_; bool fault_seen_=false; int following_=0;
  bool factor_certificate_=false;
  double reference_capture_stamp_=0.0; bool reference_captured_=false;
};
} // namespace dog_prior_map_localization
