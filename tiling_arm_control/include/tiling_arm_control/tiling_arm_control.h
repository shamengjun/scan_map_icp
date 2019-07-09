#ifndef DOWN_TOUCH_TILING_ARM_CONTROL_H_
#define DOWN_TOUCH_TILING_ARM_CONTROL_H_

#include "ros/ros.h"
#include "std_msgs/String.h"
#include "geometry_msgs/Pose.h"
#include "tiling_arm_control/rp_common.h"
#include <jsoncpp/json/json.h>
#include "tf/LinearMath/Matrix3x3.h"
#include "tile_one_unit/tile_one_unit.h"
//#include "tile_interface/tile_interface.h"

namespace tiling_arm_control
{
class TilingArmControl
{
public:
  TilingArmControl();
  ~TilingArmControl();
private:
  ros::NodeHandle nh_;
  ros::Subscriber tiling_event_sub_;
  ros::Subscriber slave_tiling_event_sub_;
  ros::Subscriber android_tiling_event_sub_;
  ros::Publisher tiling_event_pub_;
  ros::Publisher slave_tiling_event_pub_;
  ros::Publisher android_tiling_event_pub_;
  //贴砖控制命令发布
  ros::Publisher tile_cmd_pub_;

  ros::Subscriber tiling_base_sub_;
  ros::Publisher tiling_arm_pub_;
 // test接口
  ros::Subscriber test_pub_msg_sub_;

  int current_event_state_; //当前大工的事件状态
  int current_event_state_slave_; //当前小工的事件状态
  int current_event_state_android_; //当前平板端的事件状态
  std::string tiling_control_msg_; //tiling_control的msg

  unsigned int tiling_num_; //贴了多少块转
  bool one_unit_;
  bool one_row_;
  bool is_finished_;
  bool is_start_;
  //需要从参数服务器上读取
//  double tile_length_;
//  double tile_width_;
//  double tile_thickness_;
  TileInchInfo tile_inch_;
  TileTaskInfo tile_task_;

//  int one_unit_tile_num_; //一个单元贴多少块砖
//  int one_row_tile_num_; //一排贴多少块砖

  int row_num_;  //第几排
  int unit_num_; //第几个单元
  int tile_num_; //单元内的第几块砖
  int total_num_; //总共要贴的块数
  int finished_num_;//已经贴了多少块砖

  int current_row_order_;
  int current_unit_order_;
  int one_unit_tile_state_;

  geometry_msgs::PoseStamped first_suck_pose_;//第一块的吸转位置
  geometry_msgs::PoseStamped first_tile_pose_;//第一块砖放的位置
  geometry_msgs::PoseStamped last_unit_first_pose_;//上一单元的第一快转的位置
  double bounce_dis_; //回弹距离
  double tool_height_; //工具头的高度

  bool is_base_move_finished_;
  std::shared_ptr<tile_one_unit::TileOneUnit> tile_unit_; //实例化一个当前贴砖的单元指针
  std::shared_ptr<std::thread> tile_state_thread_; //创建一个获取贴砖状态的指针
  std::shared_ptr<std::thread> tile_thread_; //总的贴砖流程控制
  std::shared_ptr<std::thread> tile_control_thread_;
private:
  void tilingBaseEventSubCallback(const std_msgs::String::ConstPtr& msg);
  void tilingSlaveBaseEventSubCallback(const std_msgs::String::ConstPtr& msg);
  void androidTilingEventSubCallback(const std_msgs::String::ConstPtr& msg);

  void testPubMsgSubCallback(const std_msgs::String::ConstPtr& msg);

  void tilingBaseSubCallback(const std_msgs::String::ConstPtr& msg);

  void initial();
  //初始化第一块砖的位置
  void initialFirstTilepose();
  //初始化第一块的吸砖位置
  void initialFirstSuckPose();

  void loadParam();
  //first step
  void upArm();
  //广播等待水泥喷嘴的位置事件
  void publishWaitNozzelPoseEvent();
  //second step，获取水泥喷嘴的位置
  bool getNozzelPose(const geometry_msgs::Pose&);
  //广播水泥喷嘴位置的事件
  void publishNozzelPoseEvent(const geometry_msgs::Pose&);
  //机械臂带着水泥喷嘴到达单元初始抹水泥位置
  bool armToInitialMortarPlace();
  //广播等待小工喷水泥事件
  void publishWaitSlaveMortarEvent();
  //按照蛇行的方式铺水泥
  bool snakeMortar();
  //机械臂将水泥喷嘴板放回原处
  bool placeNozzelOriginal();
  //机械臂贴一个单元的砖
  bool tiling();
  //发布贴砖状态，贴了多少块,是否贴满一行，是否贴满一个单元，是否全部贴满
  bool tilingOneUnit();
  bool tilingOneRow();
  bool tilingAllFinish();
  void publishTilingState();
  //发布补充水泥和砖的事件
  void publishAddMortarAndTile();
  //处理开始贴砖
  void handleStartTile();
  //处理暂停贴砖
  void handlePauseTile();
  //处理恢复贴砖
  void handleResumeTile();
  //处理取消/结束贴砖
  void handleCancelTile();
  //获取贴砖状态
  void getTileState();
  //贴砖
  void tileThread();
  //一个单元贴完给底盘发消息,告诉是哪排哪个单元
  void publishBaseEvent(int row, int unit);
  //贴一个单元函数封装
  void tileOneUnit(int row, int unit);
  //发布单元内贴砖控制命令
  void publishTileCmd(std::string cmd);
  //事件处理函数
  void handleEvent();
};
}
#endif
