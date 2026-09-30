#include "p6_a2d_synthetic_window.hpp"
#include <chrono>
#include <iostream>
using namespace a2d_test;
template<class F> double timed(F f) {
  const auto start=std::chrono::steady_clock::now(); f();
  return std::chrono::duration<double,std::milli>(std::chrono::steady_clock::now()-start).count();
}
int main() {
  std::cout<<"nodes,dense_assembly_ms,block_assembly_ms,dense_solve_ms,sparse_solve_ms,marginal_covariance_ms,dense_H_payload_bytes,block_Hg_payload_bytes,sparse_H_payload_bytes,accounted_sparse_peak_upper_bytes,dense_iterations,sparse_iterations,max_state_difference\n";
  for(int n:{8,16,24,32,48}) {
    auto sparse=make(n,WindowSolverBackend::BLOCK_SPARSE), dense=make(n,WindowSolverBackend::DENSE_REFERENCE);
    std::string reason; Eigen::MatrixXd h; Eigen::VectorXd g,step; double cost;
    WindowLinearSystem system;
    const double da=timed([&]{check(dense.linearizedSystem(&h,&g,&cost,&reason),reason);});
    const double ba=timed([&]{check(sparse.blockLinearizedSystem(&system,&reason),reason);});
    const double ds=timed([&]{check(solveWindowLinearSystem(system,1e-6,WindowSolverBackend::DENSE_REFERENCE,&step,&reason),reason);});
    const double ss=timed([&]{check(solveWindowLinearSystem(system,1e-6,WindowSolverBackend::BLOCK_SPARSE,&step,&reason),reason);});
    WindowMarginalCovariance covariance;
    const double mc=timed([&]{check(sparse.latestMarginalCovariance(&covariance,&reason),reason);});
    check(sparse.optimize(&reason),reason); check(dense.optimize(&reason),reason);
    double state_error=0;
    for(int i=0;i<n;++i) state_error=std::max(state_error,localDifference(sparse.states()[i],dense.states()[i]).norm());
    check(state_error<1e-9,"scaling final state mismatch");
    const auto matrix=system.sparse();
    const std::size_t sparse_bytes=matrix.nonZeros()*(sizeof(double)+sizeof(int))+(matrix.outerSize()+1)*sizeof(int);
    // Explicit conservative payload bound, not process RSS: matrix + dense
    // worst-case LDLT payload + conversion triplets + vectors + stored blocks.
    const std::size_t accounted=system.payloadBytes()+sparse_bytes+matrix.rows()*matrix.rows()*12+
        system.upper_blocks.size()*450*sizeof(Eigen::Triplet<double>)+matrix.rows()*32;
    std::cout<<n<<','<<da<<','<<ba<<','<<ds<<','<<ss<<','<<mc<<','<<h.size()*sizeof(double)<<','
        <<system.payloadBytes()<<','<<sparse_bytes<<','<<accounted<<','
        <<dense.summary().optimizer_iterations<<','<<sparse.summary().optimizer_iterations<<','<<state_error<<'\n';
  }
}
