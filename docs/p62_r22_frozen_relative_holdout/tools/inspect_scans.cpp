#include "../../p62_r14b_newer_college/conditional_base_pilot/tools/common.hpp"
#include <iostream>
using namespace pilot;
int main(int argc,char**argv){try{
 require(argc==3,"inspect_scans input.csv exclusive_output.csv");{std::ifstream f(argv[2]);require(!f.good(),"no overwrite");}
 auto rs=readCsv(argv[1]);require(rs.size()==61,"61 scans");
 Csv out(argv[2],"local_index,raw_ordinal,stamp_ns,path,points,header_bytes,point_step,pcd_bytes,pcd_sha256,expected_source_points,expected_source_fnv");
 for(const auto&r:rs){auto s=pilot::loadSource(r);for(const auto*k:{"local_index","raw_ordinal","stamp_ns","path","points","header_bytes","point_step","pcd_bytes","pcd_sha256"})out.stream<<r.at(k)<<',';
 out.stream<<s.points->size()<<','<<cloudHash(s.points);out.end();}
 std::cout<<"SOURCE_IDENTITY_PASS 61; NDT align=0\n";
}catch(const std::exception&e){std::cerr<<e.what()<<'\n';return 1;}}
