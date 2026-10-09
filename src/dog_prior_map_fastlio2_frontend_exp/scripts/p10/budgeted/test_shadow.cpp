#include "dog_prior_map_fastlio2_frontend_exp/coupled_ndt_shadow.hpp"
#include <iostream>
int main() {
  const bool ok=dog_prior_map_fastlio2_frontend_exp::coupledNdtSelfTest();
  std::cout << "coupled_ndt_self_test=" << (ok?"PASS":"FAIL") << '\n';
  return ok?0:1;
}
