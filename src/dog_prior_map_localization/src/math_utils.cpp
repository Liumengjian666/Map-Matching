#include "dog_prior_map_localization/dog_prior_map_ekf_node.hpp"

namespace dog_prior_map_localization
{

// 构造叉乘矩阵，使 skew(v) * x 等价于 v.cross(x)。
Eigen::Matrix3d skew(const Eigen::Vector3d &v)
{
Eigen::Matrix3d m;
m << 0.0, -v.z(), v.y(),
     v.z(), 0.0, -v.x(),
    -v.y(), v.x(), 0.0;
return m;
}

// 按 Z-Y-X 顺序组合欧拉角，生成机体系到目标坐标系的旋转矩阵。
Eigen::Matrix3d rpyDegToRot(const std::vector<double> &rpy_deg)
{
const double roll = rpy_deg.size() > 0 ? rpy_deg[0] * M_PI / 180.0 : 0.0;
const double pitch = rpy_deg.size() > 1 ? rpy_deg[1] * M_PI / 180.0 : 0.0;
const double yaw = rpy_deg.size() > 2 ? rpy_deg[2] * M_PI / 180.0 : 0.0;
return (Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()) *
        Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY()) *
        Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitX())).toRotationMatrix();
}

// 对修正向量进行模长限幅，避免单次更新过大导致状态发散。
Eigen::Vector3d limitVector(const Eigen::Vector3d &v, double max_norm)
{
const double n = v.norm();
if (max_norm > 0.0 && n > max_norm)
{
  return v * (max_norm / std::max(n, 1e-12));
}
return v;
}

}  // namespace dog_prior_map_localization
