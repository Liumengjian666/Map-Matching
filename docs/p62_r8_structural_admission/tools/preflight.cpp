#include "preprocess.hpp"
#include <fstream>
#include <iostream>
#include <sys/stat.h>
using namespace shadow;
template<class T>void field(T& value){std::cin.read(reinterpret_cast<char*>(&value),sizeof(T));require(bool(std::cin),"truncated protocol metadata");}
int main(int argc,char** argv){try{
  require(argc==2,"usage: preflight NEW_OUTPUT_DIR < packed_stream");require(::mkdir(argv[1],0775)==0,"refuse existing preflight output");
  std::ofstream out(std::string(argv[1])+"/FULL_SCAN_ELIGIBILITY.csv");require(bool(out),"CSV creation failed");
  out<<"raw_index,seq,header_ns,point_start_ns,point_end_ns,raw_points,source_points,source_fnv,training_points,val0,val1,val2,val3,val4,val5,val6,val7,min_validation_points,structural_evaluable,has_source_history,causal_initial_legal,expected_points,expected_fnv,source_parity,reason\n";
  int frames=0,mismatches=0;
  while(std::cin.peek()!=std::char_traits<char>::eof()){
    uint32_t index,seq,n,flags,expected_n;uint64_t stamp,start,end,expected_hash;
    field(index);field(seq);field(stamp);field(start);field(end);field(n);field(flags);field(expected_n);field(expected_hash);
    require(index==uint32_t(frames+1)&&n>0&&n<1000000&&start<=end,"invalid stream identity");
    auto cloud=preprocessRaw(std::cin,n);auto split=splitSource(cloud);const std::string hash=cloudHash(cloud);
    std::ostringstream expected;expected<<std::hex<<std::setw(16)<<std::setfill('0')<<expected_hash;
    const bool history=flags&1,initial=flags&2;
    const bool parity=!history||(expected_n==cloud->size()&&expected.str()==hash);if(!parity)++mismatches;
    std::string reason;auto add=[&](const std::string& s){if(!reason.empty())reason+='|';reason+=s;};
    if(split.training->size()<300)add("TRAIN_LT_300");size_t minimum=SIZE_MAX;
    for(int b=0;b<8;++b){minimum=std::min(minimum,split.blocks[b]->size());if(split.blocks[b]->size()<10)add("VAL"+std::to_string(b)+"_LT_10");}
    if(!parity)add("SOURCE_IDENTITY_MISMATCH");if(reason.empty())reason="PASS";
    out<<index<<','<<seq<<','<<stamp<<','<<start<<','<<end<<','<<n<<','<<cloud->size()<<','<<hash<<','<<split.training->size();for(auto& b:split.blocks)out<<','<<b->size();
    out<<','<<minimum<<','<<split.evaluable()<<','<<history<<','<<initial<<',';
    if(history)out<<expected_n;else out<<"NA";out<<','<<(history?expected.str():"NA")<<','<<(history?(parity?"PASS":"FAIL"):"NO_HISTORY")<<','<<reason<<'\n';out.flush();
    ++frames;if(frames%1000==0)std::cout<<"PREFLIGHT "<<frames<<" align=0"<<std::endl;
  }
  require(frames==10905,"full raw sequence incomplete");require(mismatches==0,"Source parity mismatch preserved; no NDT permitted");
  std::cout<<"PREFLIGHT_COMPLETE frames="<<frames<<" history_mismatches="<<mismatches<<" NDT_aligns=0"<<std::endl;
}catch(const std::exception& e){std::cerr<<"STOP "<<e.what()<<std::endl;return 1;}}
