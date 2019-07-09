#ifndef DOWN_TOUCH_ROBOT_SIDE_BY_SIDE_H_
#define DOWN_TOUCH_ROBOT_SIDE_BY_SIDE_H_
#include "ros/ros.h"
#include "actionlib_msgs/GoalStatusArray.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include "geometry_msgs/PoseStamped.h"
#include "std_msgs/String.h"
#include "tf/transform_listener.h"

namespace robot_side_by_side
{
class RobotSideBySide
{
public:
  RobotSideBySide(ros::NodeHandle &nh);
  ~RobotSideBySide();
private:
  ros::NodeHandle nh_;
  tf::TransformListener listener_;
  ros::Subscriber base_pose_sub_;
  ros::Subscriber slave_pose_sub_;
  ros::Subscriber base_reached_target_sub_;
  ros::Subscriber slave_base_reached_target_sub_;
  ros::Publisher base_move_goal_pub_;
  ros::Subscriber start_side_by_side_sub_;
  ros::Publisher slave_base_move_goal_pub_;
  ros::Subscriber test_slave_move_sub_;
  typedef geometry_msgs::PoseWithCovarianceStamped Pose;
  geometry_msgs::PoseStamped base_at_map_pose_;
  geometry_msgs::PoseStamped slave_base_at_map_pose_;
  geometry_msgs::PoseStamped base_at_room_pose_;
  geometry_msgs::PoseStamped slave_base_at_room_pose_;
  double move_dist_;

  bool base_reached_;
  int curr_base_reached_state_;
  int last_base_reached_state_;

  bool slave_base_reached_;
  int curr_slave_base_reached_state_;
  int last_slave_base_reached_state_;

private:
  void basePoseSubCallback(const Pose::ConstPtr &msg);
  void slaveBasePoseSubCallback(const Pose::ConstPtr &msg);
  void startSideBySideSubCallback(const std_msgs::String::ConstPtr &msg);
  void baseReachedTargetSubCallback(const actionlib_msgs::GoalStatusArray::ConstPtr &msg);
  void slaveBaseReachedTargetCallback(const actionlib_msgs::GoalStatusArray::ConstPtr &msg);
  void testSlaveMoveCallback(const std_msgs::String::ConstPtr &msg);
  bool transformPose(std::string input_frame, std::string target_frame,
                     geometry_msgs::PoseStamped& input_pose,
                     geometry_msgs::PoseStamped& output_pose);
  void generatePose(geometry_msgs::PoseStamped& ,
                    geometry_msgs::PoseStamped map_pose,
                    geometry_msgs::PoseStamped& room_pose);
  bool getBaseReachedState();
  bool getSlaveBaseReachedState();
};
}
#endif // ROBOT_SIDE_BY_SIDE_H
