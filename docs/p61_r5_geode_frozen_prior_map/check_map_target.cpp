// Frozen-map construction check only: no source, pose search or align invocation.
#include <pcl/io/pcd_io.h>
#include <pcl/registration/ndt.h>
#include <iostream>
#include <stdexcept>

struct InspectTarget : pcl::NormalDistributionsTransform<pcl::PointXYZI,pcl::PointXYZI> {
    void print() {
        const auto leaf=target_cells_.getLeafSize();
        if ((leaf.head<3>()-Eigen::Vector3f::Constant(.8f)).norm()>1e-7)
            throw std::runtime_error("actual Gaussian leaf size is not 0.8m");
        std::cout << "{\"actual_gaussian_leaf_m\":[" << leaf[0] << ',' << leaf[1] << ',' << leaf[2]
                  << "],\"searchable_gaussian_cells\":" << target_cells_.getCentroids()->size()
                  << ",\"all_grid_leaves\":" << target_cells_.getLeaves().size()
                  << ",\"align_calls\":0,\"test_pose_coverage_certified\":false}\n";
    }
};

int main(int argc,char** argv) {
    if (argc!=2) return 2;
    auto cloud=boost::make_shared<pcl::PointCloud<pcl::PointXYZI>>();
    if (pcl::io::loadPCDFile(argv[1],*cloud)!=0 || cloud->empty()) return 3;
    InspectTarget target;
    target.setResolution(.8f); // Must precede setInputTarget, as in the corrected R5 grid contract.
    target.setInputTarget(cloud);
    target.print();
}
