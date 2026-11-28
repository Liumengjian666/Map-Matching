// Scores the linear probe using the unchanged Q1 fixed-pose oracle.
#define P64_DIRECTIONAL_TOOL
#include "pipeline.cpp"
#undef P64_DIRECTIONAL_TOOL

int main(int argc,char** argv) {
  try {
    need(argc==6,"native_curve SOURCE MAP POSE_WITH_AXIS REWARDS_BIN CELLS_PCD");
    const auto cloud=load(argv[1]);
    InstrumentedScorer scorer(downsample(load(argv[2])));
    Pose seed;
    Eigen::Vector3f axis;
    std::ifstream in(argv[3]);
    for(int r=0;r<4;++r)
      for(int c=0;c<4;++c)
        need(bool(in>>seed(r,c)),"pose missing");
    need(bool(in>>axis.x()>>axis.y()>>axis.z())&&seed.allFinite()&&
         std::abs(axis.norm()-1)<1e-4,"invalid pose or axis");
    std::ofstream out(argv[4],std::ios::binary);
    need(bool(out),"cannot open rewards");
    for (int k=-30;k<=30;++k) {
      Pose pose=seed;
      pose.topRightCorner<3,1>()+=(.01f*k)*axis;
      std::vector<q1::PointScore> points;
      scorer.fixed(*cloud,pose,&points);
      for (const auto& point:points) {
        double reward=point.total;
        out.write(reinterpret_cast<const char*>(&reward),sizeof reward);
      }
    }
    need(bool(out),"failed to write rewards");
    Cloud centers;
    for (const auto& item:scorer.cells()) {
      const auto& leaf=item.second;
      if(leaf.nr_points<6) continue;
      const auto& mu=leaf.getMean();
      Point point;
      point.x=mu.x(); point.y=mu.y(); point.z=mu.z();
      centers.push_back(point);
    }
    if(!centers.empty()) pcl::io::savePCDFileBinary(argv[5],centers);
  } catch(const std::exception& error) {
    std::cerr<<error.what()<<'\n';
    return 1;
  }
}
