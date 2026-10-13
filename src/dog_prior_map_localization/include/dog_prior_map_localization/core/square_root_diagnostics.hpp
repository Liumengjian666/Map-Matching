#pragma once
#include "dog_prior_map_localization/core/square_root_covariance.hpp"
#include <algorithm>
#include <chrono>
#include <fstream>
#include <iomanip>
#include <limits>
#include <map>
#include <numeric>
#include <stdexcept>
#include <string>
#include <vector>

namespace dog_prior_map_localization {
using CovarianceClock = std::chrono::steady_clock;
inline double covarianceMilliseconds(CovarianceClock::time_point start) {
  return std::chrono::duration<double,std::milli>(CovarianceClock::now()-start).count();
}
// O(n^2) factor checks, aggregate timers and five sparse checkpoints only.
// Never invokes a per-step eigensolver or writes a per-step dense matrix.
class SquareRootDiagnostics {
 public:
  void open(const std::string& prefix) {
    if (prefix.empty()) return;
    prefix_=prefix;
    scale_.open(prefix+"_scale.csv"); factors_.open(prefix+"_factors.csv");
    events_.open(prefix+"_updates.csv");
    if (!scale_ || !factors_ || !events_) throw std::runtime_error("SQRT_DIAGNOSTIC_OPEN_FAILED");
    scale_ << "checkpoint,stamp,trace,max_variance,min_diag,max_abs_factor\n";
    factors_ << "checkpoint,stamp,row,column,value\n";
    events_ << "event,stamp,trace_before,trace_after\n";
    scale_<<std::setprecision(17); factors_<<std::setprecision(17); events_<<std::setprecision(17);
  }
  void record(const std::string& event, double stamp, const Matrix15d& S,
              double ms=0.0, double before_trace=-1.0) {
    auto& row=rows_[event]; ++row.count;
    if (!validCovarianceRoot(S)) { ++row.invalid; throw std::runtime_error("INVALID_SQRT_WRITE:"+event); }
    row.min_diag=std::min(row.min_diag,S.diagonal().minCoeff());
    if (ms>=0.0) row.ms.push_back(ms);
    if (before_trace>=0.0 && events_) events_<<event<<','<<stamp<<','<<before_trace<<','<<S.squaredNorm()<<'\n';
    if (event=="IMU_PROPAGATE") {
      last_=S; last_stamp_=stamp;
      static const double offsets[]={10.0,18.355424,18.839069};
      while(next_<3 && stamp>=1517157219.1889789+offsets[next_]) {
        checkpoint("OFFSET_"+std::to_string(offsets[next_]),stamp,S); ++next_;
      }
    }
  }
  void checkpoint(const std::string& name, double stamp, const Matrix15d& S) {
    if (!scale_) return;
    scale_<<name<<','<<stamp<<','<<S.squaredNorm()<<','<<S.rowwise().squaredNorm().maxCoeff()
          <<','<<S.diagonal().minCoeff()<<','<<S.cwiseAbs().maxCoeff()<<'\n';
    for(int i=0;i<15;++i) for(int j=0;j<15;++j) factors_<<name<<','<<stamp<<','<<i<<','<<j<<','<<S(i,j)<<'\n';
  }
  void finish() {
    if(prefix_.empty()) return;
    if(last_stamp_>0.0) checkpoint("FINAL_PREFIX_STATE",last_stamp_,last_);
    std::ofstream out(prefix_+"_summary.csv"); out<<std::setprecision(17);
    out<<"event,count,invalid,min_diag,mean_ms,p95_ms,max_ms\n";
    for(auto& item:rows_) {
      auto& r=item.second; std::sort(r.ms.begin(),r.ms.end());
      double mean=0,p95=0,max=0;
      if(!r.ms.empty()) {
        mean=std::accumulate(r.ms.begin(),r.ms.end(),0.0)/r.ms.size();
        const double position=0.95*(r.ms.size()-1); const auto lo=static_cast<std::size_t>(position);
        p95=r.ms[lo]+(position-lo)*(r.ms[std::min(lo+1,r.ms.size()-1)]-r.ms[lo]); max=r.ms.back();
      }
      out<<item.first<<','<<r.count<<','<<r.invalid<<','<<r.min_diag<<','<<mean<<','<<p95<<','<<max<<'\n';
    }
    prefix_.clear();
  }
 private:
  struct Row { std::size_t count=0,invalid=0; double min_diag=std::numeric_limits<double>::infinity(); std::vector<double> ms; };
  std::string prefix_; std::ofstream scale_,factors_,events_; std::map<std::string,Row> rows_;
  std::size_t next_=0; Matrix15d last_=Matrix15d::Identity(); double last_stamp_=0.0;
};
}  // namespace dog_prior_map_localization
