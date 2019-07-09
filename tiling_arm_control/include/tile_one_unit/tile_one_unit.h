#ifndef TLING_CONTROL_TILE_ONE_UNIT_H_
#define TLING_CONTROL_TILE_ONE_UNIT_H_

#include "iostream"
#include "stdio.h"
#include "tiling_arm_control/rp_common.h"
#include "tile_interface/tile_interface.h"
#include "geometry_msgs/PoseStamped.h"
#include "thread"
#include "std_msgs/String.h"

using namespace tile_interface;

namespace tile_one_unit
{
class TileOneUnit
{
public:
  TileOneUnit(int row_order, int unit_order);
  ~TileOneUnit();
//  inline void setTilingDirection(std::string& direction) { tiling_direction_ = direction;}//需要客户端告诉贴砖方向
//  inline std::string getTilingDirection() { return tiling_direction_;}
  inline bool getOneUnitFinished() { return one_unit_tile_finished_;}
  inline bool getOneTileFinished() { return one_tile_finished_;}
  inline int getTileState() { return tiling_state_;}
  inline tiling_arm_control::TileStateInfo getTileStateInfo() {return tile_state_info_;}
  //void tiling(int row_order, int unit_order); //对外提供接口，要贴砖所在的row unit,用来计算坐标
  void tilingControlCmd(std::string); //贴砖逻辑控制，开始，暂停，恢复
  void startTiling(); //开始贴砖
  inline void setTileInchInfo(tiling_arm_control::TileInchInfo& inch){ tile_inch_info_ = inch;}
  inline void setTileTaskInfo(tiling_arm_control::TileTaskInfo& task){ tile_task_info_ = task;}
  inline geometry_msgs::PoseStamped getFirstRoomPose(){ return first_room_pose_;}
  inline void setFirstRoomPose(const geometry_msgs::PoseStamped& pose) {first_room_pose_ = pose;}
  void initTilingArmParams(geometry_msgs::PoseStamped suckPose, geometry_msgs::PoseStamped tilingPose,
                                  double bounceDis, double tool_height);
  //one unit finish, let arm back to initial pose
  void letArmBackToIntialPose();
private:
  //int one_unit_length_num_; //一个单元在x方向贴几块
 // int one_unit_width_num_; //一个单元在y方向贴几块
  //int tile_order_; //表示单元内的某块的order
  //int finished_unit_num_; //已经完成的单元数，这个主要判断是否是第一个单元，因为第一个单元的第一块和其他单元的第一块不一样
  bool one_unit_tile_finished_ = false;//一个单元是否贴完
  bool one_tile_finished_ = false; //某块砖是否贴完
  //std::string tiling_direction_; //贴砖方向，决定砖块的order，沿x方向还是y方向
  int row_order_; //本单元标识的row order
  int unit_order_; //本单元标识的unit order
  int tile_order_; //本单元第几块砖
  tiling_arm_control::TileStateInfo tile_state_info_;
  tiling_arm_control::TileInchInfo tile_inch_info_;//砖的尺寸信息，长宽高缝隙
  tiling_arm_control::TileTaskInfo tile_task_info_;//贴砖的任务信息，贴几排，每行贴几个单元，一个单元贴几块，贴砖方向
  int tiling_state_; //贴砖状态，start,tiling,cancel,pause,resume,初始状态为idle
  int current_tile_num_; //当前贴到多少块，记录pause后的tile_num
  int current_row_num_; //当前贴的是哪一排
  int current_unit_num_; //当前帖的是哪一个单元
  geometry_msgs::PoseStamped first_room_pose_;
  std::string tile_cmd_; //贴砖命令，开始，暂停，恢复
private:
  std::shared_ptr<tile_interface::TileInterface> tile_interface_;
  ros::NodeHandle nh_;
  ros::Subscriber tile_cmd_sub_;
private:
  void handleStartTiling();
  void handlePauseTiling();
  void handleResumeTiling();
  void handleCancelTiling();
  //第一单元的第一块不调用此方法
  geometry_msgs::PoseStamped getIdealTilingPos(unsigned int row, unsigned int unit, unsigned int order,
                                                            bool first_ref);
  //获取单元内其他块相对于其他砖的偏移块数
  std::array<int, 2> getOffsetOrder(int order);
  //获取某排某个单元某个块相对于第一排第一个单元第一块的偏移块数
  std::array<int, 2> getOffsetRowUnit(int row, int unit, int order);
  //获取偶数排的第一块的order
  int getEvenRowFirstOrder();
  //x y方向的异或运算 1 -1计算出来是1， 1 1 计算出来是-1
  int getSignOfDirection(int x, int y);
  //计算任意两个order,一个相对另一个在x 方向和 y方向偏了多少块
  std::array<int, 2> getTwoRelativeOrder(int base_order, int target_order);
  //计算偶数排某个单元每行或者每列的第一个贴砖的order索引
  int getUnitFirstTileOrder(int order);
  //贴奇数排的砖块
  void tileOddRow();
  //贴偶数排的砖块
  void tileEvenRow();
  //获取奇数排的单元内的第一块相对于上单元第一块的位置和
  //单元内其他块相对第一块的位置
  std::array<int, 2> getOddRowTileOffset(int row, int unit, int order);
  //获取偶数排的单元内的第一块相对于上单元第一块的位置和
  //单元内其他块相对第一块的位置
  std::array<int, 2> getEvenRowTileOffset(int row, int unit, int order);
  // initial tile state
  void initialTileState();
  // tile_cmd subcallback
  void tileCmdSubCallback(const std_msgs::String::ConstPtr& msg);
};
}
#endif
