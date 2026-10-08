// Standalone generated-helper test; no NDT, replay state, visual or GT.
#include <algorithm>
#include <array>
#include <cstdint>
#include <fstream>
#include <iostream>
#include <vector>
#include "source_export_selection.hpp"
int main(int argc,char**argv) {
  if(argc!=2)return 2;
  std::ifstream stream(argv[1]);
  std::vector<uint64_t> expected;
  uint64_t value;
  while(stream>>value)expected.push_back(value);
  if(!stream.eof()||expected.size()!=192||!std::is_sorted(expected.begin(),expected.end())||
     std::adjacent_find(expected.begin(),expected.end())!=expected.end())return 3;
  for(uint64_t tx=0;tx<=5000;++tx) {
    if(isSourceExportTarget(tx)!=std::binary_search(expected.begin(),expected.end(),tx))return 4;
  }
  std::cout<<"R4_EXPORTED_TARGET_SET_EXACT_PARITY=192/192 PASS\n";
  return 0;
}
