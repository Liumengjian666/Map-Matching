#include "p6_a2d_synthetic_window.hpp"
#include <Eigen/Cholesky>
#include <iostream>
using namespace a2d_test;
int main() {
  double max_h=0,max_g=0,max_cost=0,max_step=0,max_state=0;
  auto verify = [&](FixedLagWindow& window) {
    std::string reason; Eigen::MatrixXd h; Eigen::VectorXd g; double cost;
    WindowLinearSystem block;
    check(window.linearizedSystem(&h,&g,&cost,&reason),reason);
    check(window.blockLinearizedSystem(&block,&reason),reason);
    const double he=(h-block.dense()).norm()/std::max(1.0,h.norm());
    const double ge=(g-block.gradient).norm()/std::max(1.0,g.norm());
    const double ce=std::abs(cost-block.cost);
    max_h=std::max(max_h,he); max_g=std::max(max_g,ge); max_cost=std::max(max_cost,ce);
    check(he<1e-10 && ge<1e-10 && ce<1e-12,"block dense H/g/cost mismatch");
    for(double damping:{1e-6,.01,1.0}) {
      // Solver parity uses exactly the same assembled H/g. Independent
      // assembly parity is checked above; after convergence g is ~1e-13,
      // so relative differences between separately accumulated roundoff are
      // not a solver error (nor a step the optimizer would actually take).
      Eigen::MatrixXd damped=block.dense();
      damped.diagonal().array()+=damping*damped.diagonal().cwiseAbs().array().max(1.0);
      const Eigen::VectorXd reference=damped.ldlt().solve(-block.gradient);
      Eigen::VectorXd sparse; std::string status;
      check(solveWindowLinearSystem(block,damping,WindowSolverBackend::BLOCK_SPARSE,&sparse,&status),status);
      check(status=="BLOCK_SPARSE_SIMPLICIAL_LDLT","unexpected fallback");
      const double error=reference.norm()==0 ? sparse.norm() :
          (reference-sparse).norm()/reference.norm();
      if (error>=1e-9) std::cerr<<"step mismatch D="<<h.rows()<<" lambda="<<damping
          <<" reference_norm="<<reference.norm()<<" abs="<<(reference-sparse).norm()
          <<" relative="<<error<<" H="<<he<<" g="<<ge<<'\n';
      max_step=std::max(max_step,error); check(error<1e-9,"step equivalence");
    }
    // Removing the former H += Zero(D,D) must be bitwise innocuous.
    const auto copy=h; h+=Eigen::MatrixXd::Zero(h.rows(),h.cols());
    check((h-copy).norm()==0,"zero-add regression");
  };
  for(int n:{3,8,16}) {
    auto sparse=make(n,WindowSolverBackend::BLOCK_SPARSE);
    auto dense=make(n,WindowSolverBackend::DENSE_REFERENCE);
    verify(sparse);
    std::string reason;
    check(sparse.optimize(&reason),reason); check(dense.optimize(&reason),reason);
    check(sparse.summary().rank_diagnostic_ms==0 && sparse.summary().hessian_numerical_rank==-1,"default rank eigensolve enabled");
    for(int i=0;i<n;++i) max_state=std::max(max_state,localDifference(sparse.states()[i],dense.states()[i]).norm());
    check(max_state<1e-9,"optimized state equivalence");
    verify(sparse); // nonzero SO3 displacement from the prior's fixed chart
  }
  FixedLagOptions options; options.maximum_nodes=4; options.maximum_duration_s=100;
  ImuNoiseParameters noise; noise.gravity.setZero();
  FixedLagWindow window(options,noise);
  for(int i=0;i<12;++i) {
    append(&window,i); verify(window);
    std::string reason; check(window.optimize(&reason),reason);
    check(window.marginalizeIfNeeded(&reason),reason); verify(window);
    check(window.summary().imu_factor_count<=3 && window.summary().visual_factor_count<=3,"active count after Schur");
  }
  WindowLinearSystem indefinite;
  indefinite.gradient=Eigen::VectorXd::Ones(15);
  indefinite.add(0,0,-2*Matrix15d::Identity());
  Eigen::VectorXd fallback_step; std::string fallback_status;
  check(solveWindowLinearSystem(indefinite,.1,WindowSolverBackend::BLOCK_SPARSE,
      &fallback_step,&fallback_status) && fallback_status=="SPARSE_SOLVER_FALLBACK_DENSE",
      "dense fallback must not be silent");
  std::cout<<"PASS relative_H="<<max_h<<" relative_g="<<max_g<<" cost="<<max_cost
      <<" relative_step="<<max_step<<" final_state="<<max_state<<'\n';
}
