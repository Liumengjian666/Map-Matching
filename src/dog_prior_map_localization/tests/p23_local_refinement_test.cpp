#include "../scripts/p23_budgeted_coupled/local_refinement.hpp"
#include <iostream>
int main() {
  namespace f=dog_prior_map_fastlio2_frontend_exp;
  if (!f::coupledNdtSelfTest()) return 1; // Existing synthetic R2 math guards.
  f::CoupledNdtBackend backend;
  int jets=0,values=0;
  backend.jet=[&](const Eigen::Matrix4f&,const f::CoupledVector6&){
    ++jets;f::CoupledNativeJet jet;jet.valid=true;jet.score_sum=0;
    jet.score_hessian=-f::CoupledMatrix6::Identity();return jet;
  };
  backend.score=[&](const Eigen::Matrix4f&){++values;return -1.;};
  auto result=p23::propose(Eigen::Matrix4f::Identity(),1,backend);
  if (!result.valid || result.selected_u!=0 || result.status!="ZERO_RETAINED" ||
      jets!=1 || values!=2 || result.coupled!=Eigen::Matrix4f::Identity()) return 2;
  jets=values=0;
  backend.score=[&](const Eigen::Matrix4f&){++values;return 1.;};
  result=p23::propose(Eigen::Matrix4f::Identity(),1,backend);
  if (!result.valid || result.selected_u!=result.alpha || jets!=3 || values!=4 || result.fallback) return 3;
  if ((result.weak.block<3,1>(0,3)).norm()>.20 ||
      std::abs(Eigen::AngleAxisf(result.weak.block<3,3>(0,0)).angle())>2*M_PI/180) return 4;
  backend.jet=[](const Eigen::Matrix4f&,const f::CoupledVector6&){return f::CoupledNativeJet();};
  result=p23::propose(Eigen::Matrix4f::Identity(),1,backend);
  if (result.valid || result.weak!=Eigen::Matrix4f::Identity()) return 5;
  std::cout<<"P23_BOUNDED_POLICY_AND_EXISTING_R2_MATH_PASS (mock backend; zero real align)\n";
}
