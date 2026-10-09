#include "dog_prior_map_fastlio2_frontend_exp/coupled_ndt_anchor.hpp"
#include "dog_prior_map_fastlio2_frontend_exp/registration_geometry.hpp"
#include <Eigen/Geometry>
#include <algorithm>
#include <cmath>

namespace dog_prior_map_fastlio2_frontend_exp {
namespace {
bool rigid(const Eigen::Matrix4d& T) {
  const Eigen::Matrix3d R=T.block<3,3>(0,0);
  return T.allFinite() && (T.row(3)-Eigen::RowVector4d(0,0,0,1)).norm()<1e-6 &&
      (R.transpose()*R-Eigen::Matrix3d::Identity()).norm()<1e-4 && std::abs(R.determinant()-1)<1e-4;
}
void invalidate(CoupledAnchorState* a, const std::string& status) {
  a->valid=false; a->frozen=false; a->status=status;
}
void snapshot(const CoupledAnchorState& a, uint64_t stamp, CoupledAnchorReceipt* r) {
  r->origin=a.origin; r->prediction=a.prediction; r->weak_basis=a.weak_basis;
  r->anchor_stamp_ns=a.origin_stamp_ns; r->propagated_stamp_ns=a.propagated_stamp_ns;
  r->valid=a.valid; r->frozen=a.frozen; r->weak_dimension=a.weak_dimension;
  r->contributions=a.contributions; r->nominal_sum=a.nominal_sum; r->alternative_sum=a.alternative_sum;
  r->age_s=stamp>=a.origin_stamp_ns && a.origin_stamp_ns ? (stamp-a.origin_stamp_ns)/1e9 : 0;
  r->mean_advantage=a.contributions ? (a.nominal_sum-a.alternative_sum)/a.contributions : 0;
  r->status=a.status;
}
bool accumulate(const Eigen::Matrix4f& nominal, const Eigen::Matrix4f& alternative,
                CoupledAnchorState* a, CoupledAnchorReceipt* receipt) {
  CoupledVector6 n,b,d;
  if (!a->valid || a->weak_dimension<1 || a->weak_dimension>2 || !a->weak_basis.allFinite() ||
      !coupledAnchorChart(nominal.cast<double>(),a->prediction,&n) ||
      !coupledAnchorChart(alternative.cast<double>(),a->prediction,&b) ||
      !coupledAnchorChart(alternative.cast<double>(),nominal.cast<double>(),&d)) return false;
  const auto W=a->weak_basis.leftCols(a->weak_dimension);
  if ((W.transpose()*W-Eigen::MatrixXd::Identity(a->weak_dimension,a->weak_dimension)).norm()>1e-8)
    return false;
  receipt->nominal_cost=(W.transpose()*n).squaredNorm();
  receipt->alternative_cost=(W.transpose()*b).squaredNorm();
  a->nominal_sum+=receipt->nominal_cost; a->alternative_sum+=receipt->alternative_cost;
  ++a->contributions; receipt->evaluated=true;
  receipt->weak_fraction=d.squaredNorm()>1e-20 ? std::min(1.0,(W.transpose()*d).squaredNorm()/d.squaredNorm()) : 0;
  receipt->strong_fraction=d.squaredNorm()>1e-20 ? 1-receipt->weak_fraction : 0;
  return std::isfinite(a->nominal_sum) && std::isfinite(a->alternative_sum);
}
}  // namespace

bool coupledAnchorChart(const Eigen::Matrix4d& candidate,
    const Eigen::Matrix4d& reference, CoupledVector6* eta) {
  if (!eta || !rigid(candidate) || !rigid(reference)) return false;
  eta->head<3>()=(candidate.block<3,1>(0,3)-reference.block<3,1>(0,3))/.8;
  const Eigen::Matrix3d Rc=Eigen::Quaterniond(candidate.block<3,3>(0,0)).normalized().toRotationMatrix();
  const Eigen::Matrix3d Ra=Eigen::Quaterniond(reference.block<3,3>(0,0)).normalized().toRotationMatrix();
  eta->tail<3>()=so3Log((Rc*Ra.transpose()).eval());
  return eta->allFinite();
}

void advanceCoupledAnchor(CoupledAnchorState* a, uint64_t stamp,
    const Eigen::Matrix4d& imu) {
  if (!a || !a->valid) return;
  a->invalidated_stamp_ns=stamp;  // Prevent renewal on the frame causing invalidation.
  if (!a->origin_stamp_ns || stamp<=a->propagated_stamp_ns || stamp<a->origin_stamp_ns ||
      (stamp-a->propagated_stamp_ns)/1e9>.25 || !rigid(imu) || !rigid(a->prediction)) {
    invalidate(a,"INVALID_PROPAGATION"); return;
  }
  if ((stamp-a->origin_stamp_ns)/1e9>kCoupledAnchorLifetimeS) {
    invalidate(a,"EXPIRED"); return;
  }
  a->prediction=a->prediction*imu; a->propagated_stamp_ns=stamp;
  if (!rigid(a->prediction)) invalidate(a,"INVALID_PROPAGATED_POSE");
  else { a->status="PROPAGATED"; a->invalidated_stamp_ns=0; }
}

void settleCoupledAnchor(CoupledAnchorState* a, uint64_t stamp,
    const Eigen::Matrix4d& corrected, bool stable, bool feedback) {
  if (!a) return;
  if (feedback) { invalidate(a,"FEEDBACK_CONSUMED"); a->invalidated_stamp_ns=stamp; return; }
  if (!stable || a->valid || a->frozen || !stamp || stamp==a->invalidated_stamp_ns || !rigid(corrected)) return;
  *a=CoupledAnchorState{}; a->origin=a->prediction=corrected;
  a->origin_stamp_ns=a->propagated_stamp_ns=stamp; a->valid=true; a->status="ESTABLISHED";
}

CoupledEventResult runAnchoredCoupledNdtShadow(const Eigen::Matrix4f& nominal,
    const Eigen::Matrix4f& prediction, uint64_t stamp, std::size_t count,
    bool nominal_effective, const CoupledNdtBackend& backend, const CoupledEventConfig& config,
    PendingCandidate* pending, const Eigen::Matrix4d& imu,
    CoupledAnchorState* anchor, CoupledAnchorReceipt* receipt) {
  CoupledEventResult out;
  if (!anchor || !receipt || !pending) { out.event="INVALID_ANCHOR_INTERFACE"; return out; }
  *receipt=CoupledAnchorReceipt{};
  advanceCoupledAnchor(anchor,stamp,imu);
  snapshot(*anchor,stamp,receipt);
  const bool pending_before=pending->active;
  const double innovation_t=(nominal.block<3,1>(0,3)-prediction.block<3,1>(0,3)).cast<double>().norm();
  const double innovation_r=Eigen::Quaterniond(nominal.block<3,3>(0,0).cast<double>()).normalized().angularDistance(
      Eigen::Quaterniond(prediction.block<3,3>(0,0).cast<double>()).normalized())*180/std::acos(-1.0);
  const bool triggered=innovation_t>config.trigger_translation_m || innovation_r>config.trigger_rotation_deg;
  if (nominal_effective && (!anchor->valid || (pending_before &&
      (!anchor->frozen || anchor->event_stamp_ns!=pending->origin_stamp_ns))) && (triggered || pending_before)) {
    out.shadow.nominal_pose=out.shadow.recommended_pose=nominal;
    out.shadow.prediction_pose=prediction; out.recommendation_available=true;
    out.pending_before=pending_before; out.origin_stamp_ns=pending->origin_stamp_ns;
    out.innovation_translation_m=innovation_t; out.innovation_rotation_deg=innovation_r;
    out.innovation_trigger=triggered; out.mode="ANCHOR_SKIP";
    out.imu_interval=imu;
    out.event="ANCHOR_UNAVAILABLE_RETAINED_NOMINAL"; out.shadow.status=out.event;
    out.admission_status=receipt->status=anchor->valid ? "ANCHOR_EVENT_IDENTITY_FAILED" : anchor->status;
    *pending=PendingCandidate{}; anchor->frozen=false;
    return out;
  }
  // R3 provides quality/continuity. Do not allow R4 diagnostic validity to veto
  // an anchor-valid episode. Save its minimal history before R3 clears state.
  const PendingCandidate history=*pending;
  CoupledEventConfig diagnostic=config; diagnostic.branch_admission=false;
  diagnostic.branch_rotation_guard=false;
  out=runEventCoupledNdtShadow(nominal,prediction,stamp,count,nominal_effective,backend,diagnostic,pending,&imu);
  out.admitted=false; out.admitted_pose=nominal;
  out.origin_stamp_ns=history.origin_stamp_ns;
  out.previous_nominal=history.previous_nominal; out.previous_alternative=history.pose;
  out.imu_interval=imu;
  out.energy_difference=history.energy_difference; out.motion_difference=history.motion_difference;
  out.branch_difference=out.energy_difference+config.search.motion_weight*out.motion_difference;
  if (!nominal_effective) { invalidate(anchor,"NOMINAL_INEFFECTIVE"); anchor->invalidated_stamp_ns=stamp; }
  if (out.event=="PENDING_CREATED") {
    double score=std::numeric_limits<double>::quiet_NaN();
    ++out.shadow.terminal_score_calls;
    try { score=backend.score(nominal); } catch (...) {}
    auto found=std::find_if(out.shadow.candidates.begin(),out.shadow.candidates.end(),
        [&](const CoupledCandidate& c) {return c.id==out.pending_candidate_id;});
    if (!std::isfinite(score) || score<=0 || found==out.shadow.candidates.end() ||
        -found->refinement.score_sum/count > -score/count +
          config.search.near_quality_fraction*std::max(1.0,std::abs(score/count))) {
      *pending=PendingCandidate{};anchor->frozen=false;
      out.pending_after=false;out.event="PENDING_CREATION_ADMISSION_FAILED";
      out.admission_status="INVALID_CREATION_SCORE";return out;
    }
    // Version 1: use the new position evidence when choosing which existing
    // terminal to track, not just as a veto after the other terminal is lost.
    const int k=out.shadow.weak_dimension;
    const auto W=out.shadow.eigenvectors.leftCols(k);
    double best_cost=std::numeric_limits<double>::infinity(),best_energy=best_cost;
    for (auto candidate=out.shadow.candidates.begin();candidate!=out.shadow.candidates.end();++candidate) {
      const auto& terminal=candidate->refinement;
      if (!candidate->selected_for_refinement || !terminal.successful || !terminal.converged ||
          !rigid(terminal.pose.cast<double>()) || !std::isfinite(terminal.score_sum) || terminal.score_sum<=0 || k<1 || k>2) continue;
      const double t=(terminal.pose.block<3,1>(0,3).cast<double>()-nominal.block<3,1>(0,3).cast<double>()).norm();
      const double r=Eigen::Quaterniond(terminal.pose.block<3,3>(0,0).cast<double>()).normalized().angularDistance(
          Eigen::Quaterniond(nominal.block<3,3>(0,0).cast<double>()).normalized())*180/std::acos(-1.0);
      const double energy=-terminal.score_sum/count;
      if (!(t>config.search.separation_m || r>config.search.separation_deg) ||
          t>config.search.weak_translation_bound_m || r>config.search.weak_rotation_bound_deg ||
          energy>out.shadow.nominal_energy+config.search.near_quality_fraction*std::max(1.0,std::abs(out.shadow.nominal_energy)) ||
          energy>-score/count+config.search.near_quality_fraction*std::max(1.0,std::abs(score/count))) continue;
      CoupledVector6 eta;
      if (!coupledAnchorChart(terminal.pose.cast<double>(),anchor->prediction,&eta)) continue;
      const double cost=(W.transpose()*eta).squaredNorm();
      if (cost<best_cost || (cost==best_cost && (energy<best_energy || (energy==best_energy && candidate->id<found->id)))) {
        best_cost=cost;best_energy=energy;found=candidate;
      }
    }
    pending->pose=found->refinement.pose;out.pending_candidate_id=found->id;
    out.origin_stamp_ns=stamp;out.candidate_pose=pending->pose;
    pending->origin_stamp_ns=stamp;pending->previous_nominal=nominal;
    pending->created_nominal=nominal;pending->created_alternative=pending->pose;
    out.nominal_branch_energy=-score/count;
    out.alternative_branch_energy=-found->refinement.score_sum/count;
    out.energy_difference=(out.alternative_branch_energy-out.nominal_branch_energy)/std::max(1.0,std::abs(out.nominal_branch_energy));
    out.motion_difference=0;
    anchor->weak_dimension=out.shadow.weak_dimension;
    anchor->weak_basis.setZero();
    if (anchor->weak_dimension>=1 && anchor->weak_dimension<=2)
      anchor->weak_basis.leftCols(anchor->weak_dimension)=out.shadow.eigenvectors.leftCols(anchor->weak_dimension);
    anchor->frozen=true; anchor->event_stamp_ns=stamp;
    anchor->nominal_sum=anchor->alternative_sum=0; anchor->contributions=0;
  }
  const bool contribution=out.event=="PENDING_CREATED" || out.event=="PENDING_FIRST_SUPPORT" || out.temporally_supported;
  if (contribution) {
    const double alternative_rotation=Eigen::Quaterniond(out.candidate_pose.block<3,3>(0,0).cast<double>()).normalized().angularDistance(
        Eigen::Quaterniond(prediction.block<3,3>(0,0).cast<double>()).normalized())*180/std::acos(-1.0);
    out.rotation_consistent=(!pending_before || history.rotation_consistent) &&
        alternative_rotation<=out.innovation_rotation_deg+config.local_rotation_tolerance_deg;
    if (pending_before) {
      out.nominal_branch_energy=out.shadow.nominal_energy;
      out.alternative_branch_energy=-out.temporal_terminal.score_sum/count;
      const bool motion_valid=coupledBranchMotionCost(history.previous_nominal,nominal,imu,
          config.search,&out.nominal_motion_cost) && coupledBranchMotionCost(history.pose,out.candidate_pose,imu,
          config.search,&out.alternative_motion_cost);
      if (motion_valid && std::isfinite(out.energy_difference) && std::isfinite(out.motion_difference)) {
        out.energy_difference+=(out.alternative_branch_energy-out.nominal_branch_energy)/std::max(1.0,std::abs(out.nominal_branch_energy));
        out.motion_difference+=out.alternative_motion_cost-out.nominal_motion_cost;
      }
      // Explicitly unavailable diagnostics do not enter anchor admission.
      receipt->diagnostic_valid=history.admission_valid && motion_valid &&
          std::isfinite(out.energy_difference) && std::isfinite(out.motion_difference);
    } else receipt->diagnostic_valid=std::isfinite(out.energy_difference);
    out.branch_difference=out.energy_difference+config.search.motion_weight*out.motion_difference;
    receipt->diagnostic_valid=receipt->diagnostic_valid && std::isfinite(out.branch_difference);
    if (pending->active) {
      pending->energy_difference=out.energy_difference;pending->motion_difference=out.motion_difference;
      pending->previous_nominal=nominal;
      pending->rotation_consistent=out.rotation_consistent;
      pending->admission_valid=receipt->diagnostic_valid;  // Sticky diagnostic validity only.
    }
    if (!(std::isfinite(out.nominal_branch_energy) && out.nominal_branch_energy<0)) {
      *pending=PendingCandidate{};anchor->frozen=false;out.pending_after=false;
      out.admission_status="INVALID_NOMINAL_MATCH_QUALITY";return out;
    }
    if (!accumulate(nominal,out.candidate_pose,anchor,receipt)) {
      *pending=PendingCandidate{}; invalidate(anchor,"INVALID_ANCHOR_PROJECTION");
      anchor->invalidated_stamp_ns=stamp;
      out.admission_valid=false; out.admission_status="INVALID_ANCHOR_PROJECTION";
    } else {
      out.admission_valid=anchor->contributions==out.confirmation_count+1;
      out.admission_status="ANCHOR_AWAITING_CONFIRMATION";
      if (out.temporally_supported && out.admission_valid) {
        const double gain=(anchor->nominal_sum-anchor->alternative_sum)/3;
        out.admitted=anchor->contributions==3 && gain>kCoupledAnchorMeanMargin &&
            anchor->alternative_sum<=(1-kCoupledAnchorRelativeGain)*anchor->nominal_sum;
        out.admitted_pose=out.admitted ? out.candidate_pose : nominal;
        out.admission_status=out.admitted ? "ANCHOR_ADMITTED" : "ANCHOR_ADVANTAGE_INSUFFICIENT";
      }
    }
  }
  snapshot(*anchor,stamp,receipt);
  receipt->status=contribution ? out.admission_status : anchor->status;
  if (!pending->active) anchor->frozen=false;
  out.pending_after=pending->active;
  return out;
}
}  // namespace dog_prior_map_fastlio2_frontend_exp
