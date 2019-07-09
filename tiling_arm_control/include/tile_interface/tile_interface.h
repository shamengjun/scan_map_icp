#ifndef DOWN_TOUCH_TILE_INTERFACE_H_
#define DOWN_TOUCH_TILE_INTERFACE_H_

#include "tf/transform_datatypes.h"
#include "tile_search/tile_search.h"
#include "tiling_arm/tiling_arm.h"
#include "string"
#include "tiling_arm_control/rp_common.h"

namespace tile_interface
{
class TileInterface
{
public:
  TileInterface();
  virtual ~TileInterface();

  void setTcp(const std::string& tcp);//设置当前需移动的TCP

  //设置当前移动需要基于的坐标系,坐标系参数 "room"房间坐标系 "base_link"机器人基坐标系
  //"base"底盘坐标系 "ee_link"机器末端中心点坐标系
  //"ee_link_xxx"自定义工具坐标系 "user_xxx"自定义用户坐标系
  void setFrame(const std::string& frame);

  // 依据机器人当前位置再移动
  bool moveRelativePos(geometry_msgs::PoseStamped p);

 // 基于@setCordinate()指定的坐标系，@setTool指定TCP，移动该TCP为到指定坐标系的位姿,并能避开障碍物
  bool moveToPos(geometry_msgs::PoseStamped p);

  //机械臂沿TCP的Z方向一边慢慢移动一边检测是否会碰到障碍物，碰到就停止移动，到达移动距离也停止
  //在给定距离内碰到障碍物停止返回true，到达移动最大值没有碰障就返回false
  bool downTouch(double x);

  // 吸第一单元第一块砖包括路径规划
  bool suckUpFirstTile();

  //吸砖包括路径规划（除第一块）
  void suckUpTile();

  // suck up first tile
  void suckPerUnitFirstTile();

  // 回机械臂初始点
  bool backToInitPose();

  //触发对应IO口将物件释放，目前是砖
  bool releaseObject();

  //检测砖是否盖住
  bool isTileCovered();

  //获取大边小边的误差
  vector<double> getBiasOfAdjust();

  //获取大边小边的斜率
  double getKofAdjust();
  vector<double> getInfoOfAdjust();
  vector<double> getInfoOfFirstAdjust();

  //获取视觉调节误差，角度+距离，要调两次，应该还是分两个接口较好, num应该是角点的个数
  vector<double> getInfoOfAdjust(int num, int which_to_use);

  //获取当前机械臂末端在room坐标系下的值
  geometry_msgs::PoseStamped getCurrentRoomPose();

  //获取当前机械臂末端在base坐标系下的值
  geometry_msgs::PoseStamped getCurrentLocalPose();

  //从初始点到吸砖到放砖理论值附近点
  void moveToUnrectTilPose(geometry_msgs::PoseStamped p);

  //放砖到回到初始点
  geometry_msgs::PoseStamped moveToRectTilPose();

  //将control层处理的视觉调节值传给机械臂，让它去移动
  void adjustArm(geometry_msgs::PoseStamped p);

  //补偿机械臂定位，用户调用完此方法后最好将累计误差清零clearCumulateBias()
  bool compensateRobotArmLoca(geometry_msgs::Pose compensate);

  //补偿底盘定位，用户调用完此方法后最好将累计误差清零clearCumulateBias()
  bool compensateBaseLoca(geometry_msgs::Pose compensate);

  //是否能吸到砖中心区域
  bool isCenterSuck();

  //初始化一些机械臂需要的参数值
  void initValue(geometry_msgs::PoseStamped suckPose, geometry_msgs::PoseStamped tilingPose,
                 double bounceDis, double tool_height);

  //将RPY坐标转化为Pose数据格式
  geometry_msgs::Pose rpyToPose(const std::vector<double>& value);

  //将RPY坐标转化为Pose数据格式
  std::vector<double> poseToRPY(const geometry_msgs::Pose& value);

  //放第一单元第一块砖
  bool releaseFirstTile();

  //放砖
  bool releaseTile(geometry_msgs::PoseStamped p, int row, int unit, int order);

  //获取当前机械臂末端在room坐标系下的位置值，和在机械臂坐标系下的方向值
  void getPose();

  //根据大边小边调整放砖位姿,ps第一排第一单元第一块不需要调用此方法，需要用到砖块的尺寸信息
  void adjust(int row, int unit, int order);

  //检测是否盖住，若盖住尝试微小移开，直到不被盖住才结束返回true
  bool checkCoverEvent(unsigned int row, int unit, int order);

  //获取理想放砖位置
  geometry_msgs::PoseStamped getIdealTilingPos(unsigned int row, unsigned int unit, unsigned int order, bool first_ref);

  //设置贴砖方向，用于计算理想贴砖位置
  void setTileDirection(tiling_arm_control::TileDirection&);

  //获取砖块的尺寸信息，用来在调整时使用
  void setTileInchInfo(tiling_arm_control::TileInchInfo&);

  //获取第一块转的位置
  geometry_msgs::PoseStamped getFirstTileRoomPose();
  //设置第一块砖块的位置
  void setFirstTileRoomPose(const geometry_msgs::PoseStamped& pose)
  {
    first_room_pose_ = pose;
  }

  //当吸盘喷到要吸的砖后，视觉校准获取真正要吸的砖的位置值，基于卡槽用户坐标系，只有x,y
  geometry_msgs::Pose getCaliPoseOfSuckTile();

  //当砖放好时调用此接口可以获取当前这块砖实际放置的位姿，相对于room坐标系
  geometry_msgs::PoseStamped getRealPosOfReleaseTile();

  //斜率
  double getKofReleaseTile();

  //通过imu检测到机械末端与地面不平时，需调整机械末端与地板平行的位姿，并返回基于机械臂base_link坐标系的位姿值
  geometry_msgs::PoseStamped getParallelPoseOfReleaseTile();

  //通过imu检测地板与机械末端是否平行
  bool isParallel();

  //拍照获取放好的砖相对于room的误差
  geometry_msgs::Point getBiasOfReleaseTile();
private:
  std::shared_ptr<tiling_arm::TilingArm> tiling_arm_;
  std::shared_ptr<tile_search::TileSearch> vision_;
  tiling_arm_control::TileInchInfo tile_inch_info_;
  tiling_arm_control::TileDirection tile_direction_;
  geometry_msgs::PoseStamped first_room_pose_;
  geometry_msgs::PoseStamped first_lcoal_pose_;
};
}
#endif
