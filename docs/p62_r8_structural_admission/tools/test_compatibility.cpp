#define main frozen_r7_driver_not_executed
#include "../../p62_r7_motion_selected_shadow/tools/shadow.cpp"
#undef main
#include "preprocess.hpp"
int main(int argc,char** argv){try{
  require(argc==3,"compatibility R6_SCANS_CSV R7_SCANS_CSV");int total=0;
  for(int dataset=1;dataset<=2;++dataset)for(const auto& row:readCsv(argv[dataset])){
    auto original=loadSource(row);std::ifstream input(row.at("raw_xyz_offset_file"),std::ios::binary);
    auto adapted=preprocessRaw(input,std::stoul(row.at("raw_points")));
    require(cloudHash(original)==cloudHash(adapted),"R7 numeric/order parity");
    require(cloudHash(adapted)==row.at("expected_processed_fnv")&&adapted->size()==std::stoul(row.at("expected_processed_points")),"frozen Source FNV/size");
    auto a=splitSource(original),b=splitSource(adapted);require(cloudHash(a.training)==cloudHash(b.training),"training identity");
    for(int i=0;i<8;++i)require(cloudHash(a.blocks[i])==cloudHash(b.blocks[i]),"fixed validation identity");++total;
  }
  require(total==200,"known fixture count");
  std::cout<<"PREPROCESS_COMPATIBILITY_PASS frozen_real_fixtures=200 NDT_aligns=0\n";
}catch(const std::exception& e){std::cerr<<e.what()<<std::endl;return 1;}}
