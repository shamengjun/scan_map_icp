#include "tile_interface/tile_interface.h"

using namespace tile_interface;

TileInterface::TileInterface()
{
  tiling_arm_ = std::make_shared<tiling_arm::TilingArm>();
  vision_ = std::make_shared<tile_search::TileSearch>();
}
TileInterface::~TileInterface()
{
}
void TileInterface::initValue(geometry_msgs::PoseStamped suckPose, geometry_msgs::PoseStamped tilingPose, double bounceDis, double tool_height)
{
  std::string move_group_name = "manipulator";
  ROS_INFO("tiling_arm_->initialize before");
  //tiling_arm_->initialize(move_group_name);
  ROS_INFO("tiling_arm_->initialize after");
  tiling_arm_->setFirstTilePos(tilingPose.pose.position.x, tilingPose.pose.position.y);
  tiling_arm_->setSuckPanPos(suckPose.pose.position.x, suckPose.pose.position.y, suckPose.pose.position.z);
//  double Rx, Ry, Rz;
//  ros::NodeHandle nh_yaml_;
//  std::string name = "xiong/";
//  nh_yaml_.param(name + "first_suck_tile_pose_pose_orientation_Rx", Rx, 1.3418);
//  nh_yaml_.param(name + "first_suck_tile_pose_pose_orientation_Ry", Ry, 2.8594);
//  nh_yaml_.param(name + "first_suck_tile_pose_pose_orientation_Rz", Rz, -0.0125);
//  ROS_INFO("otherHelp initValue %f %f %f bounce =%f", Rx, Ry, Rz, bounceDis);
  //@todo四元数转为欧拉角
  geometry_msgs::Pose tmp = suckPose.pose;
  std::vector<double> rpy = poseToRPY(tmp);
  ROS_WARN("first tile pose rpy:%f %f %f",rpy[3], rpy[4], rpy[5]);
  tiling_arm_->setSuckPanOri(rpy[3], rpy[4], rpy[5]);

  tiling_arm_->setBounceDis(bounceDis);
//  double tool_height;
//  nh_yaml_.param(name + "tool_height", tool_height, 0.5);
  tiling_arm_->setToolSize(tool_height, 0, 0);
}

void TileInterface::setTcp(const std::string& tcp)
{
}

void TileInterface::setFrame(const std::string& frame)
{
  //armController_.setCordinate(frame);
}

geometry_msgs::PoseStamped TileInterface::getCurrentRoomPose()
{
  //return tiling_arm_->setRoomTarget();
  return tiling_arm_->getRealRoomPose();
}

geometry_msgs::PoseStamped TileInterface::getCurrentLocalPose()
{
  return tiling_arm_->setLocalTarget();
}


bool TileInterface::moveRelativePos(geometry_msgs::PoseStamped p)
{
  // tiling_arm_->moveXH(p);
  tiling_arm_->adjustWithCam(p);
  return true;
}

bool TileInterface::moveToPos(geometry_msgs::PoseStamped p)
{
  // tiling_arm_->moveXH(p);
  tiling_arm_->adjustWithCam(p);
  return true;
}

bool TileInterface::downTouch(double x)
{
  return true;
  // return armController_.downTouch(x);
}

bool TileInterface::suckUpFirstTile()
{
  // tiling_arm_->firstTileBeforeCam();
  tiling_arm_->firstTileMoveToUnrectTilPose();
  return true;
}
void TileInterface::moveToUnrectTilPose(geometry_msgs::PoseStamped p)
{
  // tiling_arm_->beforeCam(p);
  tiling_arm_->moveToUnrectTilPose(p);
}

geometry_msgs::PoseStamped TileInterface::moveToRectTilPose()
{
  // tiling_arm_->afterCam();
  return tiling_arm_->moveToRectTilPose();
}

bool TileInterface::backToInitPose()
{
  tiling_arm_->backToInitPose();
  ROS_INFO("TileInterface::backToInitPose");
  return true;
}

//void TileInterface::suckUpTile()
//{
//  ROS_INFO("TileInterface::suckUpTile before");
//  tiling_arm_->suckOneTile();
//  ROS_INFO("TileInterface::suckUpTile after");
//}

void TileInterface::adjustArm(geometry_msgs::PoseStamped p)
{
  tiling_arm_->adjustArm(p);
}

bool TileInterface::releaseObject()
{
  return true;
  /// return armController_.releaseObject();
}

bool TileInterface::isTileCovered()
{
  return vision_->isTileCovered();
}


vector<double> TileInterface::getBiasOfAdjust()
{
  vector<double> a;
  return a; // vision_->getDistanceofAdjust();
}

/*
*获取视觉调节值，斜率Rz， 大边小边值x，y
*x:与上一块砖的缝隙
*y:与左边或右边砖的缝隙
* @param num 照片需要识别的角点数
* @param which_to_use 用于调试，以后可能会不需要
*/
vector<double> TileInterface::getInfoOfAdjust(int num, int which_to_use)
{
  return vision_->getAdjustInfo(num, which_to_use);
}

vector<double> TileInterface::getInfoOfFirstAdjust()
{
  return vision_->getFirstTileInfo();
}

//斜率，因为底层只开放一个接口getInfoOfAdjust(),斜率包括在里面,但其实使用的时候斜率和大边小边值是分开取用的，
//但为便于联调暂时不更换原先接口，所有这里并没被使用
double TileInterface::getKofAdjust()
{
  //vector<double> adjust = vision_->getAdjustInfo(num, which_to_use)
  return 0;
}


//当吸盘喷到要吸的砖后，视觉校准获取真正要吸的砖的位置值，基于卡槽用户坐标系，只有x,y
geometry_msgs::Pose TileInterface::getCaliPoseOfSuckTile()
{
}

//通过imu检测到机械末端与地面不平时，需调整机械末端与地板平行的位姿，并返回基于机械臂base_link坐标系的位姿值
geometry_msgs::PoseStamped TileInterface::getParallelPoseOfReleaseTile()
{
}

//通过imu检测地板与机械末端是否平行
bool TileInterface::isParallel()
{
  return true;
}

//拍照获取放好的砖相对于room的误差
geometry_msgs::Point TileInterface::getBiasOfReleaseTile()
{
}

//斜率
double TileInterface::getKofReleaseTile()
{
  return 0;
}

//当砖放好时调用此接口可以获取当前这块砖实际放置的位姿，相对于room坐标系
geometry_msgs::PoseStamped TileInterface::getRealPosOfReleaseTile()
{
//  //前两个的组合
}

//补偿机械臂定位，用户调用完此方法后最好将累计误差清零clearCumulateBias()
bool TileInterface::compensateRobotArmLoca(geometry_msgs::Pose compensate)
{
  return true;
}

//补偿底盘定位，用户调用完此方法后最好将累计误差清零clearCumulateBias()
bool TileInterface::compensateBaseLoca(geometry_msgs::Pose compensate)
{
  return true;
}

//是否能吸到砖中心区域
bool TileInterface::isCenterSuck()
{
  return true;
}

//将RPY坐标转化为Pose数据格式
geometry_msgs::Pose TileInterface::rpyToPose(const std::vector<double>& value)
{
  geometry_msgs::Pose move_pose;
  tf::Quaternion q;
  // set args
  q.setRPY(value[3], value[4], value[5]);

  move_pose.orientation.w = q.w();
  move_pose.orientation.x = q.x();
  move_pose.orientation.y = q.y();
  move_pose.orientation.z = q.z();
  move_pose.position.x = value[0];
  move_pose.position.y = value[1];
  move_pose.position.z = value[2];

  return move_pose;
}

//将RPY坐标转化为Pose数据格式
std::vector<double> TileInterface::poseToRPY(const geometry_msgs::Pose& value)
{
  tf::Quaternion qua(value.orientation.x, value.orientation.y, value.orientation.z, value.orientation.w);
  tf::Matrix3x3 mat(qua);
  std::vector<double> rpy = {value.position.x, value.position.y, value.position.z, 0.0, 0.0, 0.0};
  mat.getRPY(rpy[3], rpy[4], rpy[5]);
  return rpy;
}

void  TileInterface::suckUpTile()
{
  backToInitPose();
  tiling_arm_->suckOneTile();
}

void TileInterface::suckPerUnitFirstTile()
{
  tiling_arm_->suckOneTile();
}
//void TileInterface::backToInitPose()
//{
//  tiling_arm_->backToInitPose();
//}

bool TileInterface::releaseFirstTile()
{
  geometry_msgs::PoseStamped p1;
  ROS_INFO("releaseFirstTile");
  std::vector<double> adjust = getInfoOfAdjust(1, 0); // 0 angle adjust
  ROS_INFO("start get camera angle value:%f", adjust[0]);
  p1.header.frame_id = "/room";
  p1.pose.orientation.z = adjust[0];
  //调整
  adjustArm(p1);
  geometry_msgs::PoseStamped finished_tile = moveToRectTilPose();
  ROS_INFO_STREAM("finished_tile: x:" << finished_tile.pose.position.x << "y:" << finished_tile.pose.position.y << "z:" << finished_tile.pose.position.z);
  //all_real_tiling_pose_.push_back(finished_tile);
  return true;
}

bool TileInterface::releaseTile(geometry_msgs::PoseStamped p, int row, int unit, int order)
{
  ROS_INFO("xh  releaseTile moveToUnrectTilPose ");
  moveToUnrectTilPose(p);
  // checkCoverEvent(1,unit,order);
  // checkImu();
  adjust(row, unit, order);
  geometry_msgs::PoseStamped finished_tile = moveToRectTilPose();
  ROS_INFO_STREAM("finished_tile: x:" << finished_tile.pose.position.x << "y:" << finished_tile.pose.position.y << "z:" << finished_tile.pose.position.z);
  //all_real_tiling_pose_.push_back(finished_tile);
  ROS_INFO("releaseTile succeed");
  return true;
}

void TileInterface::adjust(int row, int unit, int order)
{
  geometry_msgs::PoseStamped p1, p11, p2, p22;
  int num; //相机拍照需要识别的角点个数
  //计算角点个数，只适用于2×n的单元铺砖形式
  if (unit == 1 && (order == 1 || order == 2) || row == 1 && order % 2 != 0)
  {
    num = 2;
  }
  else
  {
    num = 4;
  }
  //even row
  if (row == 2)
  {
    if (order % 2 == 1 && order != 1)
    {
      num = 3;
    }
    else
    {
      num = 2;
    }
  }

  std::vector<double> adjust1 = getInfoOfAdjust(num, 0); // 0 angle adjust
  ROS_INFO("start get camera angle value:%f", adjust1[0]);
  p1.header.frame_id = "/world";
  p2.header.frame_id = "/world";
  p11.header.frame_id = "/world";
  p22.header.frame_id = "/world";

  p1.pose.orientation.z = adjust1[0];
  adjustArm(p1);
  ROS_WARN("adjust angle first time %f", adjust1[0]); // first time parallel;

  std::vector<double> bias = getInfoOfAdjust(num, 1); // 1 dis adjust
  ROS_INFO("start get camerza dis value:%f %f %f", bias[0], bias[1], bias[2]);
  double x = bias[1];
  double y = bias[2];
//  if (unit == 1 && order == 2) // to do ziyi
//  {
//    y = y + tile_inch_info_.tile_gap;
//  }
//  else
//  {
//    x = x - tile_inch_info_.tile_gap;
//  }
  p2.pose.position.x = x;
  p2.pose.position.y = y;
  ROS_INFO("xh camera xh dis value x y:%f %f", x, y);
  adjustArm(p2);
  ROS_INFO("xh adjust distance first time succeed"); // first time distance

  //std::vector<double> adjust2 = getInfoOfAdjust(num, 0); // 0 angle adjust
  //ROS_INFO("xh start get camera angle value:%f", adjust1[0]);
  //p11.pose.orientation.z = adjust2[0];
 // adjustArm(p11);
 // ROS_WARN("xh adjust angle second time %f", adjust2[0]); // second time parallel

  //std::vector<double> bias1 = getInfoOfAdjust(num, 1); // 1 dis adjust
  //ROS_INFO("xh start get camerza dis value:%f %f %f", bias1[0], bias1[1], bias1[2]);
  //double x1 = bias1[1], y1 = bias1[2];
//  if (unit == 1 && order == 2) // to do ziyi
//  {
//    y1 = y1 + tile_inch_info_.tile_gap;
//  }
//  else
//  {
//    x1 = x1 - tile_inch_info_.tile_gap;
//  }

  //p22.pose.position.x = x1;
  //p22.pose.position.y = y1;
  //ROS_INFO("xh camera xh dis value x y:%f %f", x1, y1);

  //adjustArm(p22);
  //ROS_WARN("xh adjust distance second time succeed"); // second time distance
}

//获取当前机械臂末端在room坐标系下的位置值，和在机械臂坐标系下的方向值
void TileInterface::getPose()
{
  first_room_pose_ = getCurrentRoomPose();
  first_lcoal_pose_ = getCurrentLocalPose();
}


//bool TileInterface::moveToUnrectTilPose(geometry_msgs::PoseStamped p)
//{
//  moveToUnrectTilPose(p);
//}

void TileInterface::setTileDirection(tiling_arm_control::TileDirection &direction)
{
  tile_direction_ = direction;
}

void TileInterface::setTileInchInfo(tiling_arm_control::TileInchInfo &inch)
{
  tile_inch_info_ = inch;
}

geometry_msgs::PoseStamped TileInterface::getFirstTileRoomPose()
{
  return first_room_pose_;
}

