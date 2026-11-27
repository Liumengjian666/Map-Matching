#include <algorithm>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <map>
#include <stdexcept>
#include <vector>

#include <dog_prior_map_localization/livox_imu_deskewer.hpp>
#include <pcl/io/pcd_io.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>

namespace fs=std::filesystem;
using dog_prior_map_localization::LivoxDeskewConfig;
using dog_prior_map_localization::LivoxDeskewReport;
using dog_prior_map_localization::LivoxImuDeskewer;

ros::Time as_time(uint64_t ns) {
    ros::Time t;
    return t.fromNSec(ns);
}

int main(int argc, char** argv) {
    if(argc!=6)return 2;
    rosbag::Bag bag(argv[1],rosbag::bagmode::Read);
    const uint64_t lo=std::stoull(argv[2]),hi=std::stoull(argv[3]);
    const fs::path out=argv[5];
    if(!fs::create_directory(out))return 2;
    fs::copy_file(argv[4],out/"anchor.tsv");

    LivoxDeskewConfig config;
    config.translation_enable=false; // No synchronized causal velocity input in the Q5 window.
    config.wait_timeout=0.;
    LivoxImuDeskewer deskewer(config);
    rosbag::View all(bag);
    rosbag::View init(bag,rosbag::TopicQuery("/livox/imu"),all.getBeginTime(),
                      all.getBeginTime()+ros::Duration(3));
    size_t initialization_count=0;
    double max_init_gyro=0;
    for(const auto& message:init) {
        auto imu=message.instantiate<sensor_msgs::Imu>();
        if(!imu||initialization_count>=200)break;
        max_init_gyro=std::max(max_init_gyro, std::hypot(imu->angular_velocity.x,
                    std::hypot(imu->angular_velocity.y,imu->angular_velocity.z)));
        deskewer.addImu(imu);
        ++initialization_count;
    }
    if(initialization_count<200||max_init_gyro>.2)throw std::runtime_error("invalid boot bias interval");

    rosbag::View window(bag,rosbag::TopicQuery(std::vector<std::string>{"/livox/lidar","/livox/imu"}),
                        as_time(lo-1000000000),as_time(hi+1000000000));
    std::map<uint64_t,sensor_msgs::ImuConstPtr> imus;
    std::map<uint64_t,livox_ros_driver2::CustomMsgConstPtr> scans;
    for(const auto& message:window) {
        if(auto imu=message.instantiate<sensor_msgs::Imu>()) {
            if(imu->header.frame_id!="livox_frame")throw std::runtime_error("unexpected IMU frame");
            imus.emplace(imu->header.stamp.toNSec(),imu);
        } else if(auto scan=message.instantiate<livox_ros_driver2::CustomMsg>()) {
            if(scan->header.frame_id!="livox_frame")throw std::runtime_error("unexpected LiDAR frame");
            const uint64_t ns=scan->header.stamp.toNSec();
            if(ns>=lo&&ns<=hi)scans.emplace(ns,scan);
        }
    }
    if(scans.size()!=50)throw std::runtime_error("expected 50 historical scans");

    std::ofstream scan_file(out/"scans.tsv"),imu_file(out/"imu.tsv"),audit(out/"deskew.csv");
    imu_file<<std::setprecision(16);
    audit<<std::setprecision(16);
    audit<<"frame,header_ns,timebase_minus_header_ns,duration_s,imu_bracket_after_ns,applied,reason,corrected_points,mean_correction_m,max_correction_m,translation_applied,max_boot_gyro_rad_s\n";
    auto incoming=imus.cbegin();
    unsigned frame=0;
    for(const auto& [ns,scan]:scans) {
        unsigned max_ns=0;
        for(const auto& p:scan->points)max_ns=std::max(max_ns,p.offset_time);
        const uint64_t end=ns+max_ns;
        const auto bracket=imus.lower_bound(end);
        if(bracket==imus.end()||bracket->first-end>10000000)throw std::runtime_error("unbracketed IMU");
        while(incoming!=imus.end()&&incoming->first<=bracket->first) {
            deskewer.addImu(incoming->second);
            ++incoming;
        }
        LivoxDeskewReport report;
        const auto cloud=deskewer.deskew(scan,report);
        if(!report.applied||cloud->empty())throw std::runtime_error(report.reason);
        const auto name=std::to_string(frame)+".pcd";
        if(pcl::io::savePCDFileBinary((out/name).string(),*cloud)<0)throw std::runtime_error("PCD write");
        scan_file<<ns<<' '<<name<<'\n';
        audit<<frame<<','<<ns<<','<<static_cast<int64_t>(scan->timebase)-static_cast<int64_t>(ns)
             <<','<<max_ns*1e-9<<','<<bracket->first-end<<','<<report.applied<<','<<report.reason
             <<','<<report.corrected_points<<','<<report.mean_point_correction<<','
             <<report.max_point_correction<<','<<report.translation_applied<<','<<max_init_gyro<<'\n';
        ++frame;
    }
    for(const auto& [ns,imu]:imus)if(ns>scans.begin()->first&&ns<=scans.rbegin()->first) {
        const auto& a=imu->linear_acceleration;
        const auto& g=imu->angular_velocity;
        imu_file<<ns<<' '<<9.80665*a.x<<' '<<9.80665*a.y<<' '<<9.80665*a.z
                <<' '<<g.x<<' '<<g.y<<' '<<g.z<<'\n';
    }
}
