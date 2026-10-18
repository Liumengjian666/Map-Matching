// Observation/export only. Included after the author's existing mapper globals/functions.
#include <std_msgs/UInt64.h>
#include <functional>
#include <iomanip>
#include <stdexcept>
#include <sys/resource.h>

struct P61Scope {
    std::function<void()> finish;
    ~P61Scope() { finish(); }
};

struct P61Record {
    std::ofstream rows;
    ros::Publisher ack;
    std::string directory;
    uint64_t count = 0;
    void start(ros::NodeHandle& nh) {
        nh.getParam("p61_output_directory", directory);
        if (directory.find("/home/jian/livox_ws/p61_r5_geode_runs/") != 0)
            throw std::runtime_error("P61 requires new internal run directory");
        rows.open(directory + "/mapping_trajectory.csv", std::ios::out);
        if (!rows) throw std::runtime_error("P61 CSV open failed");
        rows << "sequence,scan_start_sec,scan_end_sec,stage,raw_points,deskew_points,imu_count,imu_first_sec,imu_last_sec,imu_received_until_sec,deskew_delta_mean_m,deskew_delta_max_m,processing_ms,map_points,effective_features";
        for (const auto* prefix : {"T_M_I", "T_I_L", "T_M_L"})
            for (int r=0; r<4; ++r) for (int c=0; c<4; ++c) rows << ',' << prefix << '_' << r << c;
        for (const auto* prefix : {"velocity", "bg", "ba", "gravity"})
            for (int c=0; c<3; ++c) rows << ',' << prefix << '_' << c;
        rows << '\n' << std::setprecision(17);
        ack = nh.advertise<std_msgs::UInt64>("/p61_mapping_ack", 10, true);
    }
    void record(const std::string& stage, double began) {
        auto s = kf.get_x();
        Eigen::Matrix4d mi=Eigen::Matrix4d::Identity(), il=mi;
        mi.block<3,3>(0,0)=s.rot.toRotationMatrix(); mi.block<3,1>(0,3)=s.pos;
        il.block<3,3>(0,0)=s.offset_R_L_I.toRotationMatrix(); il.block<3,1>(0,3)=s.offset_T_L_I;
        Eigen::Matrix4d ml=mi*il;
        double sum=0, maximum=0;
        if (stage!="FIRST_SCAN" && feats_undistort->size()==Measures.lidar->size()) {
            auto raw=*Measures.lidar;
            std::sort(raw.begin(), raw.end(), time_list);
            for (size_t i=0; i<raw.size(); ++i) {
                const double d=(raw[i].getVector3fMap()-(*feats_undistort)[i].getVector3fMap()).norm();
                sum+=d; maximum=std::max(maximum,d);
            }
            sum/=std::max(size_t(1),raw.size());
        }
        rows << ++count << ',' << Measures.lidar_beg_time << ',' << Measures.lidar_end_time << ',' << stage
             << ',' << Measures.lidar->size() << ',' << (stage=="FIRST_SCAN"?0:feats_undistort->size())
             << ',' << Measures.imu.size() << ',' << (Measures.imu.empty()?0:Measures.imu.front()->header.stamp.toSec())
             << ',' << (Measures.imu.empty()?0:Measures.imu.back()->header.stamp.toSec()) << ',' << last_timestamp_imu
             << ',' << sum << ',' << maximum << ',' << (omp_get_wtime()-began)*1000
             << ',' << ikdtree.validnum() << ',' << (stage=="MAPPED"?effct_feat_num:0);
        for (const auto& m : {mi,il,ml}) for (int r=0;r<4;++r) for(int c=0;c<4;++c) rows << ',' << m(r,c);
        for (const Eigen::Vector3d v : {Eigen::Vector3d(s.vel),Eigen::Vector3d(s.bg),Eigen::Vector3d(s.ba),Eigen::Vector3d(s.grav)})
            for(int c=0;c<3;++c) rows << ',' << v[c];
        rows << '\n'; rows.flush();
        std_msgs::UInt64 done; done.data=count; ack.publish(done);
    }
    void save_map() {
        PointVector points;
        ikdtree.frozen_snapshot(points);
        if (points.empty()) throw std::runtime_error("P61 final map is empty");
        PointCloudXYZI cloud; cloud.points=points; cloud.width=points.size(); cloud.height=1;
        if (pcl::io::savePCDFileBinary(directory+"/frozen_map.pcd",cloud)!=0)
            throw std::runtime_error("P61 final map export failed");
        struct rusage usage;
        getrusage(RUSAGE_SELF,&usage);
        std::ofstream resource(directory+"/mapper_resources.json");
        resource << std::setprecision(17) << "{\"peak_rss_kib\":" << usage.ru_maxrss
                 << ",\"user_cpu_sec\":" << usage.ru_utime.tv_sec+usage.ru_utime.tv_usec*1e-6
                 << ",\"system_cpu_sec\":" << usage.ru_stime.tv_sec+usage.ru_stime.tv_usec*1e-6
                 << ",\"exported_points\":" << cloud.size() << "}\n";
        rows.close();
    }
};
