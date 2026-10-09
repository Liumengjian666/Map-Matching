#pragma once
#include "shadow_logging.hpp"
#include "dog_prior_map_fastlio2_frontend_exp/coupled_ndt_anchor.hpp"
namespace p10log {
struct AnchorLogger {
  std::ofstream rows;
  explicit AnchorLogger(const std::string& dir): rows(output(dir+"/anchor.csv")) {
    rows << "transaction_id,valid,frozen,evaluated,status,anchor_stamp_ns,propagated_stamp_ns,age_s,anchor_origin,anchor_prediction,weak_dimension,weak_basis,contributions,nominal_weak_cost,alternative_weak_cost,nominal_sum,alternative_sum,mean_advantage,weak_fraction,strong_fraction,after_valid,after_status,after_origin_stamp_ns,diagnostic_valid\n";
  }
  void write(uint64_t tx,const paper::CoupledAnchorReceipt& r,const paper::CoupledAnchorState& after) {
    rows << tx << ',' << r.valid << ',' << r.frozen << ',' << r.evaluated << ',' << r.status << ','
      << r.anchor_stamp_ns << ',' << r.propagated_stamp_ns << ',' << r.age_s << ','
      << values(r.origin) << ',' << values(r.prediction) << ',' << r.weak_dimension << ','
      << values(r.weak_basis) << ',' << r.contributions << ',' << r.nominal_cost << ',' << r.alternative_cost
      << ',' << r.nominal_sum << ',' << r.alternative_sum << ',' << r.mean_advantage << ','
      << r.weak_fraction << ',' << r.strong_fraction << ',' << after.valid << ',' << after.status << ','
      << after.origin_stamp_ns << ',' << r.diagnostic_valid << '\n';
  }
};
}
