#include "tiling_arm_control/tiling_arm_control.h"

using namespace tiling_arm_control;

int main(int argc, char** argv)
{
  ros::init(argc, argv, "tiling_arm_control_node");
  TilingArmControl tiling_arm_control;
  //tiling_arm_control.runLoop();
  ros::spin();
  return 0;
}
