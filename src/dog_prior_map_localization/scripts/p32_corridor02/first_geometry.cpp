// One official-seed geometry check, not a search or runtime NDT modification.
#include "../p23_budgeted_coupled/frozen_ndt_rules.hpp"
#include <pcl/io/pcd_io.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/registration/ndt.h>
#include <chrono>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <sys/resource.h>
#include <fcntl.h>
#include <unistd.h>

using Cloud = pcl::PointCloud<pcl::PointXYZ>;

void writeMatrix(std::ostream& out, const Eigen::Matrix4d& matrix) {
  out << '[';
  for (int i=0; i<4; ++i) {
    if (i) out << ',';
    out << '[';
    for (int j=0; j<4; ++j) { if (j) out << ','; out << matrix(i,j); }
    out << ']';
  }
  out << ']';
}

struct Geometry { double median, p90, inlier; };
Geometry measure(const Cloud::Ptr& source, const Eigen::Matrix4d& transform,
                 pcl::KdTreeFLANN<pcl::PointXYZ>& tree) {
  std::vector<double> distances;
  std::vector<int> indices(1); std::vector<float> squared(1);
  int inliers=0;
  for (const auto& p : *source) {
    const Eigen::Vector4d q=transform*Eigen::Vector4d(p.x,p.y,p.z,1);
    if (tree.nearestKSearch(pcl::PointXYZ(q.x(),q.y(),q.z()),1,indices,squared) != 1)
      throw std::runtime_error("NN_QUERY_FAILED");
    const double distance=std::sqrt(squared[0]);
    distances.push_back(distance); if (distance<=.5) ++inliers;
  }
  std::sort(distances.begin(),distances.end());
  if (distances.empty()) throw std::runtime_error("EMPTY_SOURCE");
  return {distances[distances.size()/2], distances[static_cast<size_t>(.9*(distances.size()-1))],
          double(inliers)/distances.size()};
}

int main(int argc, char** argv) {
  if (argc != 4) { std::cerr << "map.pcd probe-directory expected-raw-point-count\n"; return 2; }
  try {
    const std::string directory=argv[2];
    const int fd=::open((directory+"/geometry.started").c_str(),O_CREAT|O_EXCL|O_WRONLY,0600);
    if (fd<0) throw std::runtime_error("GEOMETRY_ALREADY_ATTEMPTED");
    ::close(fd);
    Eigen::Matrix4d seed;
    std::ifstream matrix(directory+"/seed.f64",std::ios::binary);
    for (int i=0;i<4;++i) for(int j=0;j<4;++j)
      if (!matrix.read(reinterpret_cast<char*>(&seed(i,j)),sizeof(double)))
        throw std::runtime_error("SEED_READ_FAILED");
    if (!seed.allFinite() || (seed.row(3)-Eigen::RowVector4d(0,0,0,1)).norm()>1e-12
        || seed.block<3,3>(0,0).determinant()<=0
        || (seed.block<3,3>(0,0).transpose()*seed.block<3,3>(0,0)-Eigen::Matrix3d::Identity()).norm()>1e-8)
      throw std::runtime_error("SEED_NOT_FINITE_RIGID");
    if (matrix.peek()!=std::ifstream::traits_type::eof()) throw std::runtime_error("SEED_SIZE_INVALID");
    Cloud::Ptr raw(new Cloud), finite(new Cloud), scan(new Cloud);
    if (pcl::io::loadPCDFile(argv[1],*raw)) throw std::runtime_error("MAP_READ_FAILED");
    for (const auto& p : *raw)
      if (std::isfinite(p.x)&&std::isfinite(p.y)&&std::isfinite(p.z)) finite->push_back(p);
    p23::FrozenNdtRules rules;
    const auto map=rules.voxelDown(finite,.15,.15,0);
    const auto target=rules.voxelDown(map,.15,.15,0);
    std::ifstream input(directory+"/first_source.xyz",std::ios::binary|std::ios::ate); float xyz[3];
    const long long expected=std::stoll(argv[3]);
    if (!input || expected<=0 || input.tellg()!=expected*static_cast<long long>(sizeof(xyz)))
      throw std::runtime_error("SOURCE_SIZE_INVALID");
    input.seekg(0);
    while (input.read(reinterpret_cast<char*>(xyz),sizeof(xyz))) {
      const double range=std::sqrt(xyz[0]*xyz[0]+xyz[1]*xyz[1]+xyz[2]*xyz[2]);
      if (std::isfinite(xyz[0])&&std::isfinite(xyz[1])&&std::isfinite(xyz[2])&&range>=.5&&range<=80.)
        scan->push_back(pcl::PointXYZ(xyz[0],xyz[1],xyz[2]));
    }
    const auto source=rules.voxelDown(scan,.25,.25,1400);
    if (source->size()<50 || target->empty()) throw std::runtime_error("INSUFFICIENT_POINTS");
    pcl::KdTreeFLANN<pcl::PointXYZ> tree; tree.setInputCloud(target);
    const Geometry initial=measure(source,seed,tree);
    pcl::NormalDistributionsTransform<pcl::PointXYZ,pcl::PointXYZ> ndt;
    ndt.setInputTarget(target); ndt.setInputSource(source);
    ndt.setResolution(.8); ndt.setStepSize(.08); ndt.setTransformationEpsilon(.001);
    ndt.setMaximumIterations(40);
    Cloud aligned;
    const auto tick=std::chrono::steady_clock::now();
    ndt.align(aligned,seed.cast<float>()); // The sole complete align in this program.
    const double elapsed=std::chrono::duration<double,std::milli>(std::chrono::steady_clock::now()-tick).count();
    const Eigen::Matrix4d result=ndt.getFinalTransformation().cast<double>();
    if (!result.allFinite()) throw std::runtime_error("NONFINITE_NDT_RESULT");
    const Geometry final=measure(source,result,tree);
    const double fitness=ndt.getFitnessScore();
    if (!std::isfinite(fitness)) throw std::runtime_error("NONFINITE_NDT_FITNESS");
    const double dt=(result.block<3,1>(0,3)-seed.block<3,1>(0,3)).norm();
    const double dr=Eigen::AngleAxisd(seed.block<3,3>(0,0).transpose()*result.block<3,3>(0,0)).angle()*180/M_PI;
    const bool pass=ndt.hasConverged()&&std::isfinite(fitness)&&fitness<=10&&dt<=2&&dr<=20
      &&final.median<=.5&&final.p90<=1&&final.inlier>=.5;
    std::ofstream out(directory+"/geometry.json");
    out << std::setprecision(17) << "{\"GT_LOADED\":false,\"full_align_calls\":1,\"source_points\":"
      << source->size() << ",\"raw_map_points\":" << raw->size() << ",\"target_points\":" << target->size()
      << ",\"converged\":" << (ndt.hasConverged()?"true":"false") << ",\"iterations\":" << ndt.getFinalNumIteration()
      << ",\"fitness\":" << fitness << ",\"align_ms\":" << elapsed << ",\"seed_correction_m\":" << dt
      << ",\"seed_correction_deg\":" << dr << ",\"initial_nn_median_m\":" << initial.median
      << ",\"initial_nn_p90_m\":" << initial.p90 << ",\"initial_inlier_0_5m\":" << initial.inlier
      << ",\"final_nn_median_m\":" << final.median << ",\"final_nn_p90_m\":" << final.p90
      << ",\"final_inlier_0_5m\":" << final.inlier << ",\"result_T\":";
    writeMatrix(out,result);
    rusage usage; getrusage(RUSAGE_SELF,&usage);
    out << ",\"peak_rss_kib\":" << usage.ru_maxrss << ",\"engineering_gate\":\"" << (pass?"PASS":"FAIL") << "\"}\n";
    std::cout << "geometry=" << (pass?"PASS":"FAIL") << " fitness=" << fitness
      << " median=" << final.median << " p90=" << final.p90 << " inlier=" << final.inlier << '\n';
    return 0; // Completed check; scientific gate is recorded independently.
  } catch (const std::exception& error) { std::cerr << error.what() << '\n'; return 1; }
}
