#include "tiling_arm_control/robot_side_by_side.h"


using namespace robot_side_by_side;

RobotSideBySide::RobotSideBySide(ros::NodeHandle &nh):
  nh_(nh)

{
  curr_base_reached_state_ = 0;
  curr_slave_base_reached_state_ = 0;
  last_base_reached_state_ = 0;
  last_slave_base_reached_state_ = 0;
  nh_.param("move_dist", move_dist_, -0.6);
  base_pose_sub_ = nh_.subscribe<geometry_msgs::PoseWithCovarianceStamped>("/amcl_pose", 1,
                                                                           &RobotSideBySide::basePoseSubCallback, this);
  slave_pose_sub_ = nh_.subscribe<geometry_msgs::PoseWithCovarianceStamped>("/slave/amcl_pose", 1,
                                                                            &RobotSideBySide::slaveBasePoseSubCallback, this);
  start_side_by_side_sub_ = nh_.subscribe<std_msgs::String>("/start_side_by_side", 1, &RobotSideBySide::startSideBySideSubCallback, this);

  base_move_goal_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("/move_base_simple/goal", 1);
  slave_base_move_goal_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("/slave/move_base_simple/goal", 1);
  base_reached_target_sub_ = nh_.subscribe<actionlib_msgs::GoalStatusArray>("/move_base/status", 1,
                                                                            &RobotSideBySide::baseReachedTargetSubCallback, this);
  slave_base_reached_target_sub_ = nh_.subscribe<actionlib_msgs::GoalStatusArray>("/slave/move_base/status", 1,
                                                                                &RobotSideBySide::slaveBaseReachedTargetCallback, this);
  test_slave_move_sub_ = nh_.subscribe<std_msgs::String>("test_slave_move", 1, &RobotSideBySide::testSlaveMoveCallback, this);
}


RobotSideBySide::~RobotSideBySide()
{

}

void RobotSideBySide::startSideBySideSubCallback(const std_msgs::String::ConstPtr &msg)
{
  if (msg->data == "start")
  {
   //生成大工要去的点
    ROS_INFO("Now,start to robot side by side!");
    geometry_msgs::PoseStamped base_target_pose;
    generatePose(base_target_pose, base_at_map_pose_, base_at_room_pose_);
    base_move_goal_pub_.publish(base_target_pose);
  }
}

void RobotSideBySide::basePoseSubCallback(const Pose::ConstPtr &msg)
{
  geometry_msgs::PoseStamped pose;
  pose.header = msg->header;
  pose.pose = msg->pose.pose;
  base_at_map_pose_ = pose;
}

void RobotSideBySide::slaveBasePoseSubCallback(const Pose::ConstPtr &msg)
{
  geometry_msgs::PoseStamped pose;
  pose.header = msg->header;
  pose.pose = msg->pose.pose;
  slave_base_at_map_pose_ = pose;
}

bool RobotSideBySide::transformPose(std::string input_frame, std::string target_frame,
                                        geometry_msgs::PoseStamped& input_pose,
                                        geometry_msgs::PoseStamped& output_pose)
{
  input_pose.header.stamp = ros::Time(0);
  output_pose.header.frame_id = target_frame;
  try
  {
    listener_.waitForTransform(target_frame, input_frame, ros::Time(0), ros::Duration(5.0));
    listener_.transformPose(target_frame, input_pose, output_pose);
    return true;
  }
  catch(tf::TransformException &ex)
  {
    ROS_ERROR("%s", ex.what());
    ros::Duration(1.0).sleep();
    return false;
  }
}

void RobotSideBySide::generatePose(geometry_msgs::PoseStamped& pose,
                                       geometry_msgs::PoseStamped map_pose,
                                       geometry_msgs::PoseStamped& room_pose)
{

  bool success = transformPose("/map", "/room", map_pose, room_pose);
  if (success)
  {
    room_pose.pose.position.x += move_dist_;
  }
  bool is_success = transformPose("/room", "/map", room_pose, pose);
  if (is_success)
  {
    ROS_INFO("generate pose succeed!");
  }
  else
  {
    ROS_ERROR("generate pose failed!");
  }
}

void RobotSideBySide::baseReachedTargetSubCallback(const actionlib_msgs::GoalStatusArray::ConstPtr &msg)
{
  ROS_DEBUG("base reached msg callback!");
  if (msg->status_list.size() != 0)
  {
    curr_base_reached_state_ = msg->status_list.end()->status;
  }
  else
  {
    ROS_DEBUG("no move_base status!");
  }

  if (getBaseReachedState()) //大车到达目标点
  {
    ROS_INFO("base go to target, start to publish slave target_pose!");
    geometry_msgs::PoseStamped target_pose;
    generatePose(target_pose, slave_base_at_map_pose_, slave_base_at_room_pose_);
    //给小工发目标点
    slave_base_move_goal_pub_.publish(target_pose);
  }
   last_base_reached_state_ = curr_base_reached_state_;
}

bool RobotSideBySide::getBaseReachedState()
{
  if (last_base_reached_state_ != 3 && curr_base_reached_state_ == 3)
  {
    ROS_INFO("base got target pose!");
    return true;
  }
  else
  {
    return false;
  }
}

void RobotSideBySide::slaveBaseReachedTargetCallback(const actionlib_msgs::GoalStatusArray::ConstPtr &msg)
{
  ROS_WARN("slave base reached msg callback!");
  if (msg->status_list.size() != 0)
  {
    curr_slave_base_reached_state_ = msg->status_list.end()->status;
  }
  else
  {
    ROS_DEBUG("no move_base status!");
  }

  if (getSlaveBaseReachedState())
  {
    ROS_INFO("slave base go to target!");
  }
  last_slave_base_reached_state_ = curr_slave_base_reached_state_;
}

bool RobotSideBySide::getSlaveBaseReachedState()
{
  if (last_slave_base_reached_state_ != 3 && curr_slave_base_reached_state_ == 3)
  {
    ROS_INFO("slave base got target pose!");
    return true;
  }
  else
  {
    return false;
  }
}

void RobotSideBySide::testSlaveMoveCallback(const std_msgs::String::ConstPtr &msg)
{
  if (msg->data == "start")
  {
    ROS_INFO("start to test slave move");
    geometry_msgs::PoseStamped target_pose;
    generatePose(target_pose, slave_base_at_map_pose_, slave_base_at_room_pose_);
    base_move_goal_pub_.publish(target_pose);
  }
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "robot_side_by_side");
  ros::NodeHandle nh;
  RobotSideBySide side_by_side(nh);
  ros::spin();
  return 0;
}
