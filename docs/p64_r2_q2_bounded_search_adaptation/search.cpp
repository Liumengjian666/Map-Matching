#include "scoring_profile.hpp"
#include <pcl/io/pcd_io.h>
#include <algorithm>
#include <chrono>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <set>
#include <sstream>
#include <stdexcept>
#include <tuple>
#include <unordered_set>

using namespace q1;
namespace fs = std::filesystem;
using Clock = std::chrono::steady_clock;
constexpr double deg = 180.0 / 3.14159265358979323846;

void need(bool value, const std::string& message) {
  if (!value) throw std::runtime_error(message);
}
double elapsed(Clock::time_point start) {return std::chrono::duration<double>(Clock::now()-start).count();}

struct Timings {
  std::size_t scoreCalls=0, transforms=0, nativeQueries=0, parkQueries=0, alignCalls=0, probes=0;
  double transform=0, query=0, native=0, park=0, extract=0, fit=0, io=0;
};

class InstrumentedScorer : public Scorer {
 public:
  using Scorer::Scorer;

  Score profile(const Cloud& source, const Pose& pose, Timings& times) {
    Score score;
    score.total=source.size();
    auto start=Clock::now();
    pcl::transformPointCloud(source, transformed_, pose);
    times.transform += elapsed(start);
    ++times.transforms;
    ++times.scoreCalls;
    times.nativeQueries += source.size();
    times.parkQueries += source.size();

    start=Clock::now();
    cache_.resize(source.size());
    for (std::size_t i=0;i<source.size();++i) {
      const auto& p=transformed_[i];
      target_cells_.getNeighborhoodAtPoint7(p, neighbors_);
      auto& entry=cache_[i];
      entry.count=neighbors_.size();
      if (entry.count>7) throw std::runtime_error("DIRECT7 topology changed");
      std::copy(neighbors_.begin(),neighbors_.end(),entry.cells.begin());
      entry.park=parkMap().lookup(p);
      score.associated += entry.count!=0;
      score.parkAssociated += entry.park!=nullptr;
    }
    times.query += elapsed(start);

    start=Clock::now();
    const Eigen::Matrix<float,4,6> j=Eigen::Matrix<float,4,6>::Zero();
    const Eigen::Matrix<float,24,6> h=Eigen::Matrix<float,24,6>::Zero();
    Vec6 grad=Vec6::Zero();
    Mat6 hess=Mat6::Zero();
    for (std::size_t i=0;i<source.size();++i) {
      const auto xyz=transformed_[i].getVector3fMap().cast<double>().eval();
      const auto& entry=cache_[i];
      double point=0;
      for (std::size_t k=0;k<entry.count;++k) {
        const auto* cell=entry.cells[k];
        point += updateDerivatives(grad,hess,j,h,xyz-cell->getMean(),cell->getInverseCov(),false);
      }
      score.native += point;
    }
    times.native += elapsed(start);

    start=Clock::now();
    for (std::size_t i=0;i<source.size();++i) {
      score.park += ParkMap::evaluate(transformed_[i],cache_[i].park);
    }
    times.park += elapsed(start);
    return score;
  }

 private:
  struct Association {
    std::array<TargetGridLeafConstPtr,7> cells;
    std::size_t count=0;
    const ParkCell* park=nullptr;
  };
  Cloud transformed_;
  std::vector<Association> cache_;
  std::vector<TargetGridLeafConstPtr> neighbors_;
};

struct Frame {std::string id; fs::path map,scan; Pose pred,ndt;};

Pose readPose(std::istream& stream) {
  Pose pose;
  for (int i=0;i<4;++i) for (int j=0;j<4;++j) {
    need(bool(stream>>pose(i,j)),"cannot read pose");
  }
  need(pose.allFinite(),"nonfinite pose");
  return pose;
}

std::vector<Frame> frames(const fs::path& path) {
  std::ifstream in(path);
  std::vector<Frame> data;
  Frame f;
  while (in >> f.id >> f.map >> f.scan) {
    f.pred=readPose(in);
    f.ndt=readPose(in);
    data.push_back(f);
  }
  need(in.eof() && data.size()==12,"expected frozen 12 inputs");
  return data;
}

Cloud::Ptr load(const fs::path& path) {
  Cloud::Ptr result(new Cloud);
  need(pcl::io::loadPCDFile(path.string(),*result)==0 && !result->empty(),"can't load "+path.string());
  return result;
}

std::ofstream file(const fs::path& path,const std::string& header) {
  need(!fs::exists(path),"will not overwrite "+path.string());
  std::ofstream out(path);
  need(out.good(),"can't write "+path.string());
  out<<std::setprecision(17)<<header<<'\n';
  return out;
}

const std::string poseColumns=",T00,T01,T02,T03,T10,T11,T12,T13,T20,T21,T22,T23,T30,T31,T32,T33";

void writePose(std::ostream& out,const Pose& pose) {
  for (int i=0;i<4;++i) for (int j=0;j<4;++j) out<<','<<pose(i,j);
}

auto fields(const std::string& line) {
  std::vector<std::string> result;
  std::stringstream text(line);
  std::string cell;
  while (std::getline(text,cell,',')) result.push_back(cell);
  return result;
}

double heading(const Pose& p) {return std::atan2(p(1,0),p(0,0));}
double angle(const Pose& a,const Pose& b) {
  const Eigen::Matrix3d rel=a.block<3,3>(0,0).cast<double>().transpose()*b.block<3,3>(0,0).cast<double>();
  return std::acos(std::clamp((rel.trace()-1)/2,-1.0,1.0))*deg;
}
double distance(const Pose& a,const Pose& b) {return (a.block<3,1>(0,3)-b.block<3,1>(0,3)).cast<double>().norm();}
bool close(const Pose& a,const Pose& b) {return distance(a,b)<=0.30 && angle(a,b)<=3.0;}

struct Offset {double x=0,y=0,z=0,r=0,p=0,a=0;};

Pose shifted(const Pose& base,const Offset& d) {
  Pose result=base;
  const double h=heading(base);
  result(0,3)+=std::cos(h)*d.x-std::sin(h)*d.y;
  result(1,3)+=std::sin(h)*d.x+std::cos(h)*d.y;
  result(2,3)+=d.z;
  const auto yaw=Eigen::AngleAxisf(d.a/deg,Eigen::Vector3f::UnitZ());
  const auto pitch=Eigen::AngleAxisf(d.p/deg,Eigen::Vector3f::UnitY());
  const auto roll=Eigen::AngleAxisf(d.r/deg,Eigen::Vector3f::UnitX());
  result.topLeftCorner<3,3>()=(yaw*pitch*roll).toRotationMatrix()*base.topLeftCorner<3,3>();
  return result;
}

struct Trial {
  Pose pose;
  Offset offset;
  std::string center,stage;
  bool boundary;
  Score score;
  std::vector<int> adjacent;
  double reusedNative=std::numeric_limits<double>::quiet_NaN();
};

using PoseKey=std::array<long long,12>;
PoseKey key(const Pose& pose) {
  PoseKey result;
  int k=0;
  for (int i=0;i<3;++i) for (int j=0;j<4;++j) result[k++]=std::llround(double(pose(i,j))*1e6);
  return result;
}

class Grid {
 public:
  Grid(InstrumentedScorer& scorer,const Cloud& scan,Timings& time):scorer_(scorer),scan_(scan),time_(time) {}

  int add(const Pose& pose,const Offset& offset,const std::string& center,const std::string& stage,bool boundary) {
    ++time_.probes;
    auto [it,inserted]=keys_.emplace(key(pose),int(trials.size()));
    if (!inserted) {
      trials[it->second].boundary &= boundary;
      return it->second;
    }
    need(trials.size()<4851,"fixed-pose budget exceeded");
    const auto score=scorer_.profile(scan_,pose,time_);
    need(std::isfinite(score.native) && std::isfinite(score.park),"nonfinite score");
    trials.push_back({pose,offset,center,stage,boundary,score,{}});
    return it->second;
  }

  void link(int a,int b) {
    if (a<0 || b<0 || a==b) return;
    trials[a].adjacent.push_back(b);
    trials[b].adjacent.push_back(a);
  }

  void lattice(const Pose& base,const std::string& center,const std::string& stage,
               int halfXY,int halfYaw,double xy,double yaw,bool trim,const Offset& baseOffset={},
               double boundaryYaw=0) {
    const int size=2*halfXY+1;
    const int angles=2*halfYaw+1;
    std::vector<int> index(size*size*angles,-1);
    auto at=[&](int x,int y,int a)->int& {return index[((x+halfXY)*size+(y+halfXY))*angles+a+halfYaw];};
    for (int x=-halfXY;x<=halfXY;++x) for (int y=-halfXY;y<=halfXY;++y) for (int a=-halfYaw;a<=halfYaw;++a) {
      const bool axisEdge=(std::abs(x)==halfXY && y==0)||(std::abs(y)==halfXY && x==0);
      if (trim && ((std::abs(x)==halfXY && std::abs(y)==halfXY) || (std::abs(a)==halfYaw && axisEdge))) continue;
      Offset offset=baseOffset;
      offset.x+=x*xy;offset.y+=y*xy;offset.a+=a*yaw;
      if (stage=="fine" && (std::abs(offset.x)>2.000001 || std::abs(offset.y)>2.000001
                          || std::abs(offset.a)>boundaryYaw+1e-6)) continue;
      const bool edge=stage=="fine"
          ? (std::abs(offset.x)>=2-1e-6||std::abs(offset.y)>=2-1e-6||std::abs(offset.a)>=boundaryYaw-1e-6)
          : (std::abs(x)==halfXY||std::abs(y)==halfXY||std::abs(a)==halfYaw);
      at(x,y,a)=add(shifted(base,offset),offset,center,stage,edge);
      if (x>-halfXY) link(at(x,y,a),at(x-1,y,a));
      if (y>-halfXY) link(at(x,y,a),at(x,y-1,a));
      if (a>-halfYaw) link(at(x,y,a),at(x,y,a-1));
    }
  }

  std::vector<Trial> trials;

 private:
  std::map<PoseKey,int> keys_;
  InstrumentedScorer& scorer_;
  const Cloud& scan_;
  Timings& time_;
};

double value(const Trial& t,int metric) {return metric==0 ? (std::isnan(t.reusedNative) ? t.score.native/t.score.total : t.reusedNative) : t.score.park;}
bool supported(const Trial& t,int metric) {
  return 2*(metric==0 ? t.score.associated : t.score.parkAssociated)>=t.score.total && t.score.total>0;
}
const std::array<std::string,2> metricNames={"native","park"};

std::vector<int> peaks(const std::vector<Trial>& rows,int metric,const std::string& center="") {
  std::vector<int> selected;
  std::vector<bool> visited(rows.size(),false);
  for (std::size_t i=0;i<rows.size();++i) {
    if (visited[i] || !supported(rows[i],metric) || (!center.empty() && rows[i].center!=center)) continue;
    const double height=value(rows[i],metric);
    bool local=true;
    int representative=i;
    std::vector<int> queue{int(i)};
    visited[i]=true;
    for (std::size_t k=0;k<queue.size();++k) {
      const int j=queue[k];
      if (std::make_pair(rows[j].boundary,j)<std::make_pair(rows[representative].boundary,representative)) representative=j;
      for (int n:rows[j].adjacent) {
        if (!supported(rows[n],metric) || (!center.empty() && rows[n].center!=center)) continue;
        const double adjacent=value(rows[n],metric);
        if (adjacent>height) local=false;
        if (adjacent==height && !visited[n]) {visited[n]=true;queue.push_back(n);}
      }
    }
    if (local) selected.push_back(representative);
  }
  std::sort(selected.begin(),selected.end(),[&](int a,int b) {
    return std::make_pair(-value(rows[a],metric),a)<std::make_pair(-value(rows[b],metric),b);
  });
  return selected;
}

std::vector<int> diverse(const std::vector<int>& input,const std::vector<Trial>& rows,std::size_t budget) {
  std::vector<int> result;
  for (int candidate:input) {
    if (std::none_of(result.begin(),result.end(),[&](int prior){return close(rows[candidate].pose,rows[prior].pose);})) {
      result.push_back(candidate);
      if (result.size()==budget) break;
    }
  }
  return result;
}

void makeS3(Grid& grid,const Frame& f) {
  const double yawRange=std::clamp(std::abs(std::remainder((heading(f.pred)-heading(f.ndt))*deg,360.0)),5.0,12.0);
  grid.lattice(f.pred,"pred","coarse",5,4,.4,yawRange/4,false);
  grid.lattice(f.ndt,"ndt","coarse",5,4,.4,yawRange/4,false);
  std::vector<int> anchors;
  for (const std::string center:{"pred","ndt"}) for (int metric=0;metric<2;++metric) {
    for (int chosen:diverse(peaks(grid.trials,metric,center),grid.trials,2)) {
      if (std::find(anchors.begin(),anchors.end(),chosen)==anchors.end()) anchors.push_back(chosen);
    }
  }
  for (const std::string center:{"pred","ndt"}) {
    const int central=grid.add(center=="pred" ? f.pred : f.ndt,{},center,"base",false);
    if (std::find(anchors.begin(),anchors.end(),central)==anchors.end()) anchors.push_back(central);
  }
  need(anchors.size()<=10,"fine anchor budget");
  const Eigen::Matrix3f rotation=f.ndt.topLeftCorner<3,3>()*f.pred.topLeftCorner<3,3>().transpose();
  const double z=std::clamp(std::abs(double(f.ndt(2,3)-f.pred(2,3))),.10,.30);
  const double roll=std::clamp(std::abs(std::atan2(rotation(2,1),rotation(2,2))*deg),2.,6.);
  const double pitch=std::clamp(std::abs(std::atan2(-rotation(2,0),std::hypot(rotation(2,1),rotation(2,2)))*deg),2.,6.);
  for (int index:anchors) {
    const Trial base=grid.trials[index];
    const Pose& center=base.center=="pred" ? f.pred : f.ndt;
    grid.lattice(center,base.center,"fine",2,2,.1,yawRange/8,false,base.offset,yawRange);
    for (double sign:{-1.,1.}) {
      for (double level:{.5,1.}) {
        const double d=sign*level;
        const std::array<Offset,3> perturb={{{0,0,d*z,0,0,0},{0,0,0,d*roll,0,0},{0,0,0,0,d*pitch,0}}};
        for (auto trial:perturb) {
          trial.x=base.offset.x;trial.y=base.offset.y;trial.a=base.offset.a;
          grid.link(index,grid.add(shifted(center,trial),trial,base.center,"tilt",base.boundary));
        }
      }
      for (double second:{-1.,1.}) {
        Offset trial=base.offset;
        trial.r=sign*roll;trial.p=second*pitch;
        grid.link(index,grid.add(shifted(center,trial),trial,base.center,"tilt",base.boundary));
      }
    }
  }
}

Pose csvPose(const std::vector<std::string>& row,std::size_t position) {
  Pose result;
  for (int i=0;i<4;++i) for (int j=0;j<4;++j) result(i,j)=std::stof(row.at(position+i*4+j));
  return result;
}

std::map<std::string,std::vector<Trial>> loadS0(const fs::path& path) {
  std::ifstream in(path);
  need(in.good(),"cannot read frozen S0");
  std::string line;
  std::getline(in,line);
  std::map<std::string,std::vector<Trial>> result;
  while (std::getline(in,line)) {
    auto row=fields(line);
    const int index=std::stoi(row.at(1));
    const int x=std::stoi(row.at(2)),y=std::stoi(row.at(3)),a=std::stoi(row.at(4));
    const auto total=std::stoul(row.at(12));
    Score score{std::stod(row.at(9))*total,std::stod(row.at(10)),std::stoul(row.at(11)),
                std::stoul(row.at(30)),total};
    auto& trials=result[row.at(0)];
    need(std::size_t(index)==trials.size(),"frozen S0 indices invalid");
    trials.push_back({csvPose(row,14),{std::stod(row.at(5)),std::stod(row.at(6)),0,0,0,std::stod(row.at(7))},
                      "ndt","reused",x==0||x==20||y==0||y==20||a==0||a==10,score,{},std::stod(row.at(9))});
    for (int d: {x>0?231:0, y>0?11:0, a>0?1:0}) if (d) {
      trials[index].adjacent.push_back(index-d);
      trials[index-d].adjacent.push_back(index);
    }
  }
  need(result.size()==12,"S0 expected 12 samples");
  for (const auto& item:result) need(item.second.size()==4851,"incomplete S0 grid");
  return result;
}

using Seeds=std::vector<std::pair<int,int>>;

double relative(const Trial& row,const Trial& best,int metric) {
  if (metric==0) return value(row,0)/std::max(1e-12,value(best,0));
  return std::exp(alpha*(value(row,1)-value(best,1)));
}

struct Selection {
  std::array<std::vector<int>,2> c2;
  Seeds seeds;
};

Selection extract(const std::vector<Trial>& rows,const std::string& id,const std::string& method,
                  std::ostream& out,Timings& timer,const std::set<int>& previouslyFit) {
  const auto start=Clock::now();
  Selection result;
  std::array<std::vector<int>,2> c0, c1, local;
  for (int metric=0;metric<2;++metric) {
    local[metric]=peaks(rows,metric);
    auto& pool=local[metric];
    if (pool.empty()) continue;
    c0[metric]=diverse(pool,rows,5);
    c1[metric]=c0[metric];
    auto edge=std::find_if(pool.begin(),pool.end(),[&](int i){return rows[i].boundary;});
    if (edge!=pool.end() && std::none_of(c1[metric].begin(),c1[metric].end(),[&](int i){return rows[i].boundary;})) {
      if (c1[metric].size()==5) c1[metric].pop_back();
      c1[metric].push_back(*edge);
    }
    std::vector<int> plausible;
    std::copy_if(pool.begin(),pool.end(),std::back_inserter(plausible),[&](int k) {
      return relative(rows[k],rows[pool.front()],metric)>=.90;
    });
    result.c2[metric]=diverse(plausible,rows,5);
  }
  for (bool edgePreferred:{true,false}) for (int metric=0;metric<2;++metric) {
    if (method=="S0" && !edgePreferred) continue;
    const auto& kept=result.c2[metric];
    auto next=std::find_if(kept.begin(),kept.end(),[&](int k) {
      return previouslyFit.count(k)==0 && rows[k].boundary==edgePreferred && std::none_of(result.seeds.begin(),result.seeds.end(),[&](auto e){return e.first==k;});
    });
    if (next!=kept.end()) result.seeds.emplace_back(*next,metric);
  }
  if (method!="S0") for (std::size_t rank=0;rank<5 && result.seeds.size()<4;++rank) {
    for (int metric=0;metric<2 && result.seeds.size()<4;++metric) if (rank<result.c2[metric].size()) {
      const int k=result.c2[metric][rank];
      if (std::none_of(result.seeds.begin(),result.seeds.end(),[&](auto e){return e.first==k;})) result.seeds.emplace_back(k,metric);
    }
  }
  timer.extract+=elapsed(start);
  const auto io=Clock::now();
  for (int metric=0;metric<2;++metric) for (int policy=(method=="S0" ? 1:0);policy<4;++policy) {
    const auto& pool=local[metric];
    const auto& kept=policy==0 ? c0[metric] : policy==1 ? c1[metric] : result.c2[metric];
    if (pool.empty()) {
      out<<id<<','<<method<<','<<metricNames[metric]<<",C"<<policy<<",-1,0,0,0,NO_VALID_MODE,no_support,0\n";
      continue;
    }
    for (int k:pool) {
      const auto it=std::find(kept.begin(),kept.end(),k);
      const double rel=relative(rows[k],rows[pool.front()],metric);
      out<<id<<','<<method<<','<<metricNames[metric]<<",C"<<policy<<','<<k<<','
         <<(it==kept.end()?0:std::distance(kept.begin(),it)+1)<<','<<(it!=kept.end())<<','<<rows[k].boundary<<','
         <<(rows[k].boundary?"BOUNDARY_CANDIDATE":"SAMPLED_PEAK")<<','
         <<(it!=kept.end()?"retained":rel<.90&&policy>=2?"relative_score":"diversity_or_cap")<<','<<rel<<'\n';
    }
  }
  timer.io+=elapsed(io);
  return result;
}

void writeGrid(std::ostream& out,const std::string& id,const std::string& method,const std::vector<Trial>& points) {
  for (std::size_t i=0;i<points.size();++i) {
    const auto& t=points[i];
    out<<id<<','<<method<<','<<i<<','<<t.center<<','<<t.stage<<','<<t.offset.x<<','<<t.offset.y<<','
       <<t.offset.z<<','<<t.offset.r<<','<<t.offset.p<<','<<t.offset.a<<','<<value(t,0)<<','<<value(t,1)
       <<','<<t.score.associated<<','<<t.score.parkAssociated<<','<<t.score.total<<','<<t.boundary;
    writePose(out,t.pose);
    out<<'\n';
  }
}

void refine(std::ostream& out, InstrumentedScorer& scorer,const Cloud::ConstPtr& cloud,
            const std::vector<Trial>& trials,const Seeds& seeds,const std::string& id,
            const std::string& method,Timings& timers) {
  need(seeds.size()<=(method=="S0"?2:4),"local optimization budget exceeded");
  scorer.setInputSource(cloud);
  scorer.setNumThreads(1);
  for (const auto& [index,metric]:seeds) {
    Cloud aligned;
    const auto start=Clock::now();
    scorer.align(aligned,trials[index].pose);
    timers.fit+=elapsed(start);
    ++timers.alignCalls;
    const Pose terminal=scorer.getFinalTransformation();
    need(terminal.allFinite(),"nonfinite native terminal");
    Timings scoreTimers;
    const auto score=scorer.profile(*cloud,terminal,scoreTimers);
    const double drift=distance(terminal,trials[index].pose),rotation=angle(terminal,trials[index].pose);
    const bool kept=drift<=.35&&rotation<=4&&scorer.hasConverged()&&scorer.getFinalNumIteration()>0&&
                    2*score.associated>=score.total;
    const auto io=Clock::now();
    out<<id<<','<<method<<','<<index<<','<<metricNames[metric]<<",C3,"<<scorer.hasConverged()<<','
       <<scorer.getFinalNumIteration()<<','<<score.native/score.total<<','<<score.associated<<','<<score.total;
    writePose(out,terminal);
    out<<','<<drift<<','<<rotation<<','<<kept<<','<<score.park<<','<<score.parkAssociated<<','
       <<scoreTimers.transform+scoreTimers.query+scoreTimers.native+scoreTimers.park<<'\n';
    timers.io+=elapsed(io);
  }
}

void gate(const std::vector<Frame>& input,const fs::path& q1,const fs::path& output) {
  auto report=file(output,"sample_id,pose_id,q1_upstream_total,profile_native_sum,native_error,profile_park_log,q1_read_only_park_log,park_error,native_associated,park_associated,pass");
  std::ifstream in(q1);
  need(in.good(),"cannot read Q1 gate");
  std::string line;
  std::getline(in,line);
  std::map<std::string,std::vector<std::vector<std::string>>> cases;
  while (std::getline(in,line)) {
    auto row=fields(line);
    cases[row.at(0)].push_back(std::move(row));
  }
  for (const auto& f:input) {
    const auto source=downsample(load(f.scan));
    InstrumentedScorer scorer(downsample(load(f.map)));
    need(cases.at(f.id).size()==17,"expected 17 gate cases");
    for (const auto& r:cases.at(f.id)) {
      const Pose pose=csvPose(r,13);
      const auto baseline=scorer.fixed(*source,pose);
      Timings t;
      const auto live=scorer.profile(*source,pose,t);
      const double reference=std::stod(r.at(2));
      const bool okay=std::abs(live.native-reference)<=1e-6*std::max(std::abs(reference),1e-12)&&
          live.native==baseline.native&&live.park==baseline.park&&live.associated==baseline.associated&&
          live.parkAssociated==baseline.parkAssociated&&live.associated==std::stoul(r.at(8))&&
          live.total==std::stoul(r.at(10));
      report<<f.id<<','<<r.at(1)<<','<<reference<<','<<live.native<<','<<std::abs(live.native-reference)<<','
            <<live.park<<','<<baseline.park<<','<<std::abs(live.park-baseline.park)<<','<<live.associated<<','
            <<live.parkAssociated<<','<<okay<<'\n';
      report.flush();
      need(okay,"Q1 scoring gate failed "+f.id+" "+r.at(1));
    }
  }
}

void search(const std::vector<Frame>& input,const fs::path& baseline,const fs::path& directory,
            const fs::path& previous) {
  auto frozen=loadS0(baseline);
  std::ifstream old(previous);
  need(old.good(),"cannot read sealed Q1 refinement list");
  std::map<std::string,std::set<int>> existing;
  std::string line;
  std::getline(old,line);
  while (std::getline(old,line)) {
    const auto row=fields(line);
    existing[row.at(0)].insert(std::stoi(row.at(1)));
  }
  need(!fs::exists(directory),"fresh run directory required");
  fs::create_directories(directory);
  auto scores=file(directory/"grid.csv","sample_id,method,index,center,stage,dx,dy,dz,droll,dpitch,dyaw,native,park,associated,park_associated,total,boundary"+poseColumns);
  auto retained=file(directory/"retained.csv","sample_id,method,metric,policy,index,rank,retained,boundary,peak_class,filter_reason,relative_score");
  auto fits=file(directory/"fit.csv","sample_id,method,index,metric,policy,reported_converged,iterations,native,associated,total"+poseColumns+
                 ",drift_translation_m,drift_rotation_deg,mode_preserved,park,park_associated,postfit_score_s");
  auto timing=file(directory/"runtime_breakdown.csv","sample_id,method,score_calls,native_calls,park_calls,transform_calls,native_queries,park_queries,align_calls,transform_s,query_s,native_s,park_s,extract_s,fit_s,io_s,grid_s,unique_poses,proposals,duplicates,preprocess_s,postfit_score_calls,postfit_native_queries,postfit_park_queries");
  for (const auto& f:input) {
    const auto prep=Clock::now();
    const auto cloud=downsample(load(f.scan));
    InstrumentedScorer scorer(downsample(load(f.map)));
    const double preprocess=elapsed(prep);
    for (const std::string method:{"S0","S1","S2","S3"}) {
      auto begin=Clock::now();
      Timings times;
      Grid grid(scorer,*cloud,times);
      if (method=="S1") grid.lattice(f.pred,"pred","base",10,5,.2,1.,false);
      if (method=="S2") {
        grid.lattice(f.pred,"pred","base",7,5,2./7,1.,true);
        grid.lattice(f.ndt,"ndt","base",7,5,2./7,1.,true);
      }
      if (method=="S3") makeS3(grid,f);
      const auto& trials=method=="S0" ? frozen.at(f.id) : grid.trials;
      const double gridTime=elapsed(begin);
      need(trials.size()<=4851 && !trials.empty(),"invalid grid count");
      if (method=="S1") need(trials.size()==4851,"S1 grid is incomplete");
      if (method=="S2") need(times.probes==4846,"S2 raw budget mismatch");
      if (method=="S3") need(times.probes<=3590,"S3 raw budget mismatch");
      const auto selection=extract(trials,f.id,method,retained,times,method=="S0" ? existing[f.id] : std::set<int>{});
      refine(fits,scorer,cloud,trials,selection.seeds,f.id,method,times);
      const auto io=Clock::now();
      writeGrid(scores,f.id,method,trials);
      scores.flush();retained.flush();fits.flush();
      times.io+=elapsed(io);
      timing<<f.id<<','<<method<<','<<times.scoreCalls<<','<<times.scoreCalls<<','<<times.scoreCalls<<','
           <<times.transforms<<','<<times.nativeQueries<<','<<times.parkQueries<<','<<times.alignCalls<<','
           <<times.transform<<','<<times.query<<','<<times.native<<','<<times.park<<','<<times.extract<<','
           <<times.fit<<','<<times.io<<','<<gridTime<<','<<trials.size()<<','<<times.probes<<','
           <<(times.probes-times.scoreCalls)<<','<<preprocess<<','<<times.alignCalls<<','
           <<times.alignCalls*cloud->size()<<','<<times.alignCalls*cloud->size()<<'\n';
      timing.flush();
    }
  }
}

int main(int argc,char** argv) {
  try {
    need(argc>=5,"usage: search gate/search online.tsv q1.csv fresh_output [q1_fit.csv]");
    const auto input=frames(argv[2]);
    const std::string mode=argv[1];
    if (mode=="gate" && argc==5) gate(input,argv[3],argv[4]);
    else if (mode=="search" && argc==6) search(input,argv[3],argv[4],argv[5]);
    else throw std::runtime_error("unknown mode");
    return 0;
  } catch (const std::exception& e) {
    std::cerr<<e.what()<<'\n';
    return 2;
  }
}
