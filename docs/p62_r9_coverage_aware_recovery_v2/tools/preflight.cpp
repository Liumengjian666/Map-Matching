#include "io.hpp"
#include <iostream>
using namespace coverage;
int main(int argc,char** argv){try{
  require(argc==3,"usage: preflight SCANS_CSV NEW_OUTPUT_DIR");require(::mkdir(argv[2],0775)==0,"refuse existing preflight");
  auto rows=readCsv(argv[1]);require(rows.size()==100,"fixed100 only");std::string root=argv[2];
  Csv out(root+"/COVERAGE_COMPARISON.csv",coverageHeader());
  Csv spatial(root+"/BLOCK_SPATIAL_DISTRIBUTION.csv","scan_index,block_id,sector_id,active,points,angle_start_deg,angle_end_deg,cx_m,cy_m,cz_m,minx_m,miny_m,minz_m,maxx_m,maxy_m,maxz_m");
  for(int i=0;i<100;++i){const auto& row=rows[i];require(std::stoi(row.at("raw_index"))==10641+i,"fixed original window");prediction(row);auto c=loadSource(row);
    require(c->size()==std::stoul(row.at("expected_processed_points"))&&cloudHash(c)==row.at("expected_processed_fnv"),"frozen Source parity");auto s=splitSource(c);Mask m(s);coverageRow(out,row,c,s,m);
    for(int b=0;b<8;++b){Eigen::Vector3d sum=Eigen::Vector3d::Zero(),lo=Eigen::Vector3d::Constant(INFINITY),hi=-lo;
      for(const auto& p:*s.blocks[b]){Eigen::Vector3d x(p.x,p.y,p.z);sum+=x;lo=lo.cwiseMin(x);hi=hi.cwiseMax(x);}
      spatial.stream<<i+1<<','<<b<<','<<2*b+1<<','<<(s.blocks[b]->size()>=10)<<','<<s.blocks[b]->size()<<','<<-180+(2*b+1)*22.5<<','<<-180+(2*b+2)*22.5;
      for(double v:{sum.x(),sum.y(),sum.z()}){spatial.stream<<',';number(spatial.stream,s.blocks[b]->empty()?NAN:v/s.blocks[b]->size());}
      for(int a=0;a<3;++a){spatial.stream<<',';number(spatial.stream,s.blocks[b]->empty()?NAN:lo[a]);}for(int a=0;a<3;++a){spatial.stream<<',';number(spatial.stream,s.blocks[b]->empty()?NAN:hi[a]);}spatial.end();}
  }
  std::cout<<"COVERAGE_PREFLIGHT_PASS fixed100 source_parity100 new_aligns=0\n";
}catch(const std::exception& e){std::cerr<<"STOP "<<e.what()<<'\n';return 1;}}
