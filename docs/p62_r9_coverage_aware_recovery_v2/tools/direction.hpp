#pragma once
#include "coverage.hpp"
#include "../../p62_r7_motion_selected_shadow/tools/recovery.hpp"
namespace coverage {
// Same normalized PSD G, eigenanalysis and guards as frozen R7 recover(); no align.
inline Recovery direction(NativeNdt& ndt,const Split& s,const Pose& center){
  Recovery r;const auto start=Clock::now();const Matrix6 g=ndt.geometry(s.training,center,r.supported);
  Eigen::SelfAdjointEigenSolver<Matrix6> eig;if(g.allFinite())eig.compute(g);
  if(!g.allFinite()||eig.info()!=Eigen::Success){r.direction="INVALID_G";r.invalid=true;}
  else{r.eigenvalues=eig.eigenvalues();double low=r.eigenvalues[0],high=r.eigenvalues[5];
    if(high<=1e-12)r.direction="NO_INFORMATION";
    else if(low< -1e-8*high){r.direction="INVALID_G_SPECTRUM";r.invalid=true;}
    else{r.weak_count=(r.eigenvalues.array()/high<=.01).count();if(low/high>.01)r.direction="NO_WEAK_DIRECTION";
      else{r.direction="WEAK_DIRECTION";r.weak_vector=eig.eigenvectors().col(0);}}}
  r.g_ms=milliseconds(start);return r;
}
} // namespace coverage
