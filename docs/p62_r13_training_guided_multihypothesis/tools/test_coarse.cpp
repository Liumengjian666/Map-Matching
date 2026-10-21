#include "coarse.hpp"
#include "../../p62_r12_feasibility_preserving_pool/tools/pool_v3.hpp"
#include <iostream>
using namespace training_grid;
int main(){try{
  int count=0;auto test=[&](bool yes,const char* why){require(yes,why);++count;};const Pose p=Pose::Identity();
  auto g=grid(p);test(g.size()==45&&g[0].dx==-.4&&g[0].dy==-.4&&g[0].yaw_deg==-10&&g[44].yaw_deg==10,"fixed45/order");
  for(const auto& s:g)test(translation(s.native_pose,p)<=.60&&rotation(s.native_pose,p)<=12.,"actual float bounds");
  const auto& coupled=g[4];test((coupled.pose.block<3,1>(0,3)-Eigen::Vector3d(-.4,-.4,0)).norm()>1e-3,"SE3 uses translation Jacobian, not plain translation");
  Pose moved=p;moved.block<3,3>(0,0)=Eigen::AngleAxisd(.8,Eigen::Vector3d::UnitX()).toRotationMatrix();moved(0,3)=2;
  const auto shifted=grid(moved);test((shifted[4].pose-moved*g[4].pose).norm()<1e-12,"right body perturbation frame order");
  Cloud training;for(int i=0;i<600;++i)training.push_back(pcl::PointXYZ(1,.1f,float(i)*.001f));
  auto sample=subset(training);test(sample.size()==256&&sample.front()==0&&sample.back()==599,"deterministic endpoint subset256");
  Cloud one;one.push_back(training[0]);test(subset(one)==std::vector<size_t>{0},"one sample");
  Cloud heldout;heldout.push_back(pcl::PointXYZ(1,-.1f,0));bool rejected=false;try{subset(heldout);}catch(...){rejected=true;}test(rejected,"validation sector cannot enter training subset");
  size_t queried=0;const auto n=score(g,training,sample,[&](const Eigen::Vector3d&){++queried;return .0625;});
  test(n==45*256&&queried==n&&g[0].loss==.25&&g[0].overlap==1,"squared NN loss .5 scale and query count");
  score(g,training,sample,[](const Eigen::Vector3d&){return .25;});test(g[0].loss==1&&g[0].overlap==1,".5m equality counts as overlap");
  score(g,training,sample,[](const Eigen::Vector3d&){return .5;});test(g[0].loss==1&&g[0].overlap==0&&select(g,p).size()==2,"saturated score can still pick deterministic seeds, not informed evidence");
  test(!locallyExcluded(.25,0)&&!locallyExcluded(0,4)&&locallyExcluded(.249,3.999),"strict local exclusion .25m/4deg boundaries");
  test(sufficientlySeparated(.30,0)&&sufficientlySeparated(0,4)&&!sufficientlySeparated(.299,3.999),"inclusive diversity .30m/4deg boundaries");
  g=grid(p);for(auto& s:g){s.loss=.5;s.overlap=.5;}const auto pick=select(g,p);
  test(pick.size()==2&&pick[0]==0&&pick[1]==1,"equal losses/overlaps use ID then fixed diversity");
  test(g[22].local_excluded&&g[22].selected_rank==0,"identity local seed excluded");
  g=grid(p);for(auto& s:g){s.loss=.5;s.overlap=.5;}g[44].overlap=.6;auto by_overlap=select(g,p);test(by_overlap[0]==44,"overlap secondary descending");
  g[2].loss=.1;auto by_loss=select(g,p);test(by_loss[0]==2,"loss primary ascending");
  auto few=grid(p);for(auto& s:few){s.loss=.5;s.overlap=.5;s.native_pose=p;}
  test(select(few,p).empty(),"all local exclusion gives zero seeds");
  for(auto& s:few)s.native_pose(0,3)=.4;test(select(few,p).size()==1,"no diverse second gives one seed");
  auto bad=grid(p);bad[0].loss=NAN;rejected=false;try{select(bad,p);}catch(...){rejected=true;}test(rejected,"nonfinite score stops");
  g=grid(p);rejected=false;try{score(g,training,sample,[](const Eigen::Vector3d&){return NAN;});}catch(...){rejected=true;}test(rejected,"nonfinite NN stops");
  std::cout<<"R13_COARSE_TESTS_PASS checks="<<count<<" new_ndt_align=0\n";
}catch(const std::exception& e){std::cerr<<e.what()<<'\n';return 1;}}
