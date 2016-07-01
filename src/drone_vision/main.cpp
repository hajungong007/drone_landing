#include "droneVision.hpp"
int main(int argc, char** argv)
{
  ros::init(argc, argv, "droneVision");
  droneVision drone;
  ros::spin();
  return 0;
}

