#include "dog_prior_map_localization/dog_prior_map_ekf_node.hpp"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "dog_prior_map_ekf");
  try
  {
    dog_prior_map_localization::DogPriorMapEkfNode node;
    ros::spin();
  }
  catch (const std::exception &e)
  {
    ROS_FATAL("[DogPriorMap C++] node exited unexpectedly: %s", e.what());
    return 1;
  }
  return 0;
}
