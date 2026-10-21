#include "coupled.hpp"
#include <iostream>

using namespace coupled;

static Cloud::Ptr syntheticCloud() {
  Cloud::Ptr cloud(new Cloud);
  for (int ix=-2; ix<=2; ++ix) for (int iy=-2; iy<=2; ++iy) for (int iz=-2; iz<=2; ++iz) {
    for (int sx : {-1,1}) for (int sy : {-1,1}) for (int sz : {-1,1}) {
      pcl::PointXYZ p;
      p.x = float(ix*2.4 + .4 + sx*.12);
      p.y = float(iy*2.4 + .4 + sy*.12);
      p.z = float(iz*2.4 + .4 + sz*.12);
      cloud->push_back(p);
    }
  }
  shadow::finish(cloud); return cloud;
}

static void check(bool ok, const char* message) {
  if (!ok) throw std::runtime_error(message);
}

int main() {
  try {
    const auto map = syntheticCloud();
    Vector6 anchor;anchor<<.05,-.04,.02,.4,-.3,.5;
    const Pose generic=cause::expSE3(anchor);
    Vector6 offset;offset<<.10,-.05,.03,.02,.01,-.02;
    const Pose displaced=cause::expSE3(offset);
    Cloud::Ptr source(new Cloud);
    pcl::transformPointCloud(*map,*source,(generic.inverse()*displaced).cast<float>());
    shadow::finish(source);
    ExactPclObjective ndt; ndt.configureExact(map, source);
    Pose t0=Pose::Identity();
    const double direct=ndt.directScore(t0);
    const double pcl=ndt.pclDerivativeScore(t0);
    check(std::isfinite(direct)&&std::abs(direct-pcl)<=1e-9,"PCL/native Gaussian score parity");
    size_t direct_supported=0, converted_supported=0;
    const Matrix6 g=ndt.directGeometry(t0,direct_supported);
    const Matrix6 gr=ndt.convertedR6Geometry(t0,converted_supported);
    check(direct_supported==converted_supported,"R6/direct support count");
    check((g-gr).norm()/std::max(1.,g.norm())<=1e-7,"R6 geometry coordinate conversion (PCL float transform tolerance)");
    Eigen::SelfAdjointEigenSolver<Matrix6> eig(g);
    check(eig.info()==Eigen::Success&&eig.eigenvalues().allFinite(),"geometry eigensolver");
    check((eig.eigenvectors().transpose()*eig.eigenvectors()-Matrix6::Identity()).norm()<=1e-10,"geometry basis orthogonality");
    check(eig.eigenvalues().minCoeff()>=-1e-10*std::max(1.,eig.eigenvalues().maxCoeff()),"PSD geometry");

    Vector6 z; z << .3,-.2,.1,.1,-.25,.15;
    const Pose perturbed=poseAt(t0,z);
    check(rigid(perturbed),"right SE3 pose validity");
    check((cause::logSE3(t0.inverse()*perturbed)-scaleMatrix()*z).norm()<1e-8,"full SE3 Exp/Log right perturbation");
    check((cause::expSE3(cause::logSE3(t0.inverse()*perturbed))-(t0.inverse()*perturbed)).norm()<1e-8,"SE3 Exp(Log(T))");
    const auto d=cause::logSE3(t0.inverse()*perturbed);
    check(d.head<3>().norm()<=.30*z.head<3>().norm()+1e-10,"translation physical scaling");
    check(d.tail<3>().norm()<=3.*shadow::PI/180.*z.tail<3>().norm()+1e-10,"rotation physical scaling");

    size_t support_generic=0,support_r6_generic=0;
    const Matrix6 gg=ndt.directGeometry(generic,support_generic);
    const Matrix6 ggr=ndt.convertedR6Geometry(generic,support_r6_generic);
    check(support_generic==support_r6_generic&&(gg-ggr).norm()/std::max(1.,gg.norm())<=1e-7,"generic-pose R6 geometry conversion (PCL float transform tolerance)");
    LocalModel local(ndt,generic);
    const auto deriv=local.validateAndDifferentiate();
    check(deriv.ok,"exact-score right-tangent gradient/Hessian checks on smooth synthetic branch");

    // A point just inside a Gaussian radius boundary must make a derivative
    // stencil fail if its prescribed +x perturbation changes leaf membership.
    Cloud::Ptr boundary_source(new Cloud);
    pcl::PointXYZ boundary_point; boundary_point.x=-3.6001f; boundary_point.y=-4.4f; boundary_point.z=-4.4f;
    boundary_source->push_back(boundary_point); shadow::finish(boundary_source);
    ExactPclObjective boundary_ndt; boundary_ndt.configureExact(map,boundary_source);
    LocalModel boundary_local(boundary_ndt,Pose::Identity());
    Vector6 boundary_step=Vector6::Zero(); boundary_step[0]=kGradientStep;
    int boundary_queries=0; std::string boundary_failure;
    check(boundary_local.nominalSignature().size()==1&&boundary_local.nominalSignature()[0].size()==1&&
          !boundary_local.associationStable(boundary_step,boundary_queries),
          "synthetic Gaussian radius-boundary association change");
    const Vector6 boundary_gradient=boundary_local.gradient(Vector6::Zero(),boundary_queries,boundary_failure);
    check(boundary_failure=="LOCAL_ASSOCIATION_SET_CHANGED"&&!boundary_gradient.allFinite(),
          "association-changing derivative stencil rejected");

    // The Schur cross contribution must vanish exactly when Hsw is zero.
    Eigen::Matrix2d hss; hss << 2.,.1,.1,1.5;
    Eigen::Vector2d gs(1.,-.5), dw(.2,-.1);
    Eigen::Matrix<double,2,2> hsw=Eigen::Matrix<double,2,2>::Zero();
    Eigen::LDLT<Eigen::Matrix2d> ldlt(hss + kLambda*Eigen::Matrix2d::Identity());
    const Eigen::Vector2d ds0=ldlt.solve(-gs);
    const Eigen::Vector2d ds1=ldlt.solve(-(gs+hsw*dw));
    check((ds1-ds0).norm()<1e-12,"zero Hsw has zero coupled increment");
    hsw << .2,-.1,.05,.3;
    const Eigen::Vector2d ds2=ldlt.solve(-(gs+hsw*dw));
    check((hss*ds2+gs+hsw*dw+kLambda*ds2).norm()<1e-12,"coupled block optimality equation");
    check((ds2-ds0).norm()>1e-6,"nonzero Hsw produces coupled response");

    // End-to-end refinement selects at most two weak directions. A third
    // qualifying weak direction is reported and held fixed, never solved as strong.
    Eigen::Matrix<double,6,1> multi_weak;multi_weak<<0.,.005,.01,1.,1.,1.;
    Vector6 prediction_z=Vector6::Zero();prediction_z[0]=.2;prediction_z[1]=-.1;prediction_z[2]=.8;
    const Pose prediction_pose=generic*cause::expSE3(scaleMatrix()*prediction_z);
    Matrix6 coupled_h=Matrix6::Identity();coupled_h(3,0)=coupled_h(0,3)=.4;
    const Refinement coupled_ref=refine(ndt,generic,prediction_pose,multi_weak,Matrix6::Identity(),coupled_h,Vector6::Zero(),1.);
    check(coupled_ref.weak_available&&coupled_ref.coupled_available&&
          coupled_ref.weak_ids=="0|1"&&coupled_ref.strong_ids=="3|4|5"&&coupled_ref.unmodeled_weak_ids=="2",
          "selected weak, strong, and unmodeled weak subspaces");
    check(std::abs(coupled_ref.z_weak[2])<1e-12&&std::abs(coupled_ref.z_coupled[2])<1e-12&&
          std::abs(coupled_ref.z_weak[3])<1e-12&&std::abs(coupled_ref.z_coupled[3])>1e-4,
          "unmodeled weak stays fixed while Hsw moves a strong coordinate");
    Eigen::Vector3d strong_ds;strong_ds<<coupled_ref.ds[3],coupled_ref.ds[4],coupled_ref.ds[5];
    Eigen::Vector2d selected_dw;selected_dw<<coupled_ref.dw[0],coupled_ref.dw[1];
    check(coupled_ref.raw_hss_antisymmetry<=.05&&coupled_ref.raw_hss_condition<=1e8&&
          std::abs(coupled_ref.strong_response_norm-coupled_ref.ds.norm())<1e-12&&
          (coupled_ref.hss*strong_ds+coupled_ref.gs+coupled_ref.hsw*selected_dw+kLambda*strong_ds).norm()<1e-10,
          "end-to-end raw-Hss gates, trust response, and coupled block stationarity");

    Matrix6 uncoupled_h=Matrix6::Identity();
    const Refinement uncoupled_ref=refine(ndt,generic,prediction_pose,multi_weak,Matrix6::Identity(),uncoupled_h,Vector6::Zero(),1.);
    check(uncoupled_ref.weak_available&&uncoupled_ref.coupled_available&&
          uncoupled_ref.coupling_norm<1e-12&&uncoupled_ref.ds.norm()<1e-12,
          "end-to-end Hsw=0 has no response when gs=0");

    // A rejected coupled strong block must not erase the independently
    // defined WEAK_ONLY ablation.
    Eigen::Matrix<double,6,1> test_eigs;test_eigs<<0.,1.,1.,1.,1.,1.;
    Matrix6 test_h=Matrix6::Identity();test_h(1,1)=-1.;
    Vector6 test_g=Vector6::Zero();test_g[0]=.2;
    Vector6 predicted_z=Vector6::Zero();predicted_z[0]=.2;
    const Pose test_tp=generic*cause::expSE3(scaleMatrix()*predicted_z);
    const Refinement blocked=refine(ndt,generic,test_tp,test_eigs,Matrix6::Identity(),test_h,test_g,1.);
    check(blocked.weak_available&&!blocked.coupled_available&&
          blocked.weak_status=="WEAK_ONLY_READY"&&
          blocked.coupled_status=="RAW_HSS_NOT_POSITIVE_OR_ILL_CONDITIONED"&&
          blocked.z_weak.allFinite()&&std::abs(blocked.z_weak[0]-.2)<1e-10,
          "coupled solve failure preserves paired WEAK_ONLY candidate");

    std::cout << "MATHEMATICAL_TESTS_PASS score_parity=" << std::abs(direct-pcl)
              << " geometry_rel=" << (g-gr).norm()/std::max(1.,g.norm())
              << " derivative_rel=" << deriv.gradient_check_max_rel
              << " hessian_antisym=" << deriv.h_antisymmetry
              << " scalar_hessian_rel=" << deriv.hessian_check_max_rel
              << " step_spread=" << deriv.hessian_step_spread_max << '\n';
  } catch (const std::exception& e) {
    std::cerr << "MATHEMATICAL_TESTS_FAIL " << e.what() << '\n'; return 1;
  }
  return 0;
}
