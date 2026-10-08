// Read-only source provenance diagnosis. No NDT align/objective/labels/GT.
#define P9_NDT_ENERGY_CONTRACT_LIBRARY
#include "p9_ndt_energy_contract.cpp"
int main(int argc,char**argv) {
  try {
    if(argc!=2)throw std::runtime_error("usage: p9_r4_source_audit SOURCE_MANIFEST.csv");
    const auto input=readCsv(argv[1]);
    std::cout<<"transaction_id,expected_points,actual_points,expected_hash,actual_hash,parity\n";
    for(const auto& row:input.rows) {
      const auto cloud=preprocessSource(loadPackedSource(input.get(row,"raw_cloud_file")));
      const auto expected=parseU64(input.get(row,"prepared_source_hash"));
      const auto points=parseU64(input.get(row,"prepared_source_point_count"));
      const auto hash=sourceHash(*cloud);
      std::cout<<input.get(row,"transaction_id")<<','<<points<<','<<cloud->size()<<','<<expected<<','<<hash<<','
        <<(points==cloud->size()&&expected==hash?"PASS":"FAIL")<<'\n';
    }
    return 0;
  } catch(const std::exception&e){std::cerr<<e.what()<<'\n';return 1;}
}
