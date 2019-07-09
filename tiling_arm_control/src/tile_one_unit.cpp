#include "tile_one_unit/tile_one_unit.h"



using namespace tile_one_unit;
using namespace tiling_arm_control;

TileOneUnit::TileOneUnit(int row_order, int unit_order):
  row_order_(row_order),
  unit_order_(unit_order)
{
  //tiling_arm_ = std::make_shared<tiling_arm::TilingArm>();
  //tiling_vision_ = std::make_shared<tile_search::TileSearch>();
  ROS_INFO("tile identity [%d %d]", row_order_, unit_order_);
  tile_interface_ = std::make_shared<tile_interface::TileInterface>();
  tiling_state_ = IDLED; //初始状态为空闲
  one_tile_finished_ = false;
  initialTileState();
  tile_cmd_sub_ = nh_.subscribe<std_msgs::String>("tile_cmd", 1, &TileOneUnit::tileCmdSubCallback, this);
}

TileOneUnit::~TileOneUnit()
{

}

void TileOneUnit::initialTileState()
{
  tile_state_info_.row_order = 0;
  tile_state_info_.unit_order = 0;
  tile_state_info_.tile_order = 0;
}

void TileOneUnit::initTilingArmParams(geometry_msgs::PoseStamped suckPose,
                                      geometry_msgs::PoseStamped tilingPose, double bounceDis, double tool_height)
{
  tile_interface_->initValue(suckPose, tilingPose, bounceDis, tool_height);
}
//void TileOneUnit::setTileInchInfo(std::vector<double>& tile_info)
//{
//  ROS_INFO("set tile inch info: [%f %f %f %f]", tile_info[0], tile_info[1], tile_info[2], tile_info[3]);
//  tile_inch_info_.tile_length = tile_info[0];
//  tile_inch_info_.tile_width = tile_info[1];
//  tile_inch_info_.tile_height = tile_info[2];
//  tile_inch_info_.tile_gap = tile_info[3];
//}

//void TileOneUnit::tiling()
//{
//  if (tiling_state_ == START)
//  {
//    // 贴完一个单元
//    ROS_INFO("%d unit has tiled!", unit_order_);
//    tiling_state_ = IDLED;
//  }
//  else
//  {
//    ROS_ERROR("tiling has not start!");
//  }
//}

void TileOneUnit::tileCmdSubCallback(const std_msgs::String::ConstPtr &msg)
{
  ROS_INFO("this is tile cmd cb!");
  if (msg->data == "pause")
  {
    tiling_state_ = PAUSE;
  }
  else if (msg->data == "resume")
  {
    tiling_state_ = RESUME;
  }
  else if (msg->data == "cancel")
  {
    tiling_state_ = CANCEL;
  }
  else
  {
    ROS_ERROR("invalid tile cmd!");
  }
}


void TileOneUnit::tilingControlCmd(std::string control_cmd)
{
  tile_cmd_ = control_cmd;
}

void TileOneUnit::startTiling()
{
  ROS_INFO("start to tile!");
  if (tiling_state_ != IDLED)
  {
    ROS_ERROR("tiling_state is not idle, so can't handle start to tile");
  }
  else
  {
    tiling_state_ = TILING;
    //要判断该单元是奇数排还是偶数排
    if (row_order_ % 2 != 0) //奇数排
    {
      tileOddRow();
    }
    else //偶数排
    {
      tileEvenRow();
    }
    //正常完成贴砖
    tiling_state_ = FINISHED;
  }
}

void TileOneUnit::handleStartTiling()
{
  ROS_INFO("handle start tile!");
  if (tiling_state_ != IDLE)
  {
    ROS_ERROR("tile state is not idle, so can't start to tile!");
  }
  else
  {
    tiling_state_ = START;
  }
}

void TileOneUnit::handlePauseTiling()
{
  ROS_INFO("handle pause tile!");
  if (tiling_state_ != TILING)
  {
    ROS_ERROR("the tile state is not tiling, so can't pause tile!");
  }
  else
  {
    tiling_state_ = PAUSE;
  }
}

void TileOneUnit::handleResumeTiling()
{
  ROS_INFO("handle resume tile");
  if (tiling_state_ != PAUSE)
  {
    ROS_ERROR("tiling state is not pause, so can't resume !");
  }
  else
  {
    tiling_state_ = RESUME;
  }
}

void TileOneUnit::handleCancelTiling()
{
  ROS_INFO("handle cancel tile!");
  if (tiling_state_ != START)
  {
    ROS_INFO("cancel this unit tile!");
  }
  else
  {
    ROS_ERROR("tile state is START, can't cancel tile!");
  }
}

//获取x y 方向偏离本单元第一块砖的块数 test pass
std::array<int, 2> TileOneUnit::getOffsetOrder(int order)
{
  std::array<int, 2> offset = {0 , 0};
  //这里以room的x y方向为参考
  if (tile_task_info_.tile_direction.direction == 0) //沿x方向贴 pass
  {
    ROS_INFO("ENTER THIS BRANCH!");
    offset.at(0) = (order - 1) / tile_task_info_.y_tiles_per_unit * tile_task_info_.tile_direction.direction_x;
    offset.at(1) = -1 * (order - 1) % tile_task_info_.y_tiles_per_unit * tile_task_info_.tile_direction.direction_y;
  }
  else //沿y方向贴 pass
  {
    offset.at(0) = -(order - 1) % tile_task_info_.x_tiles_per_unit * tile_task_info_.tile_direction.direction_x;
    offset.at(1) = (order - 1) / tile_task_info_.x_tiles_per_unit * tile_task_info_.tile_direction.direction_y;
  }
  return offset;
}
//获取某排某个单元某个块相对于第一排第一个单元第一块的偏移块数，test pass
std::array<int, 2> TileOneUnit::getOffsetRowUnit(int row, int unit, int order)
{
  std::array<int, 2> offset = {0 , 0};
  if (tile_task_info_.tile_direction.direction == 0)
  {
    // x方向只看偏了几个单元
    offset.at(0) = tile_task_info_.x_tiles_per_unit * (unit - 1) * tile_task_info_.tile_direction.direction_x
        + getOffsetOrder(order).at(0);
    // y方向只看偏了几排
    offset.at(1) = -(row - 1) * tile_task_info_.y_tiles_per_unit * tile_task_info_.tile_direction.direction_y
        + getOffsetOrder(order).at(1);
  }
  else
  {
    //x方向只看偏了几排
    offset.at(0) = -(row - 1) * tile_task_info_.x_tiles_per_unit * tile_task_info_.tile_direction.direction_x
        + getOffsetOrder(order).at(0);
    //y方向只看偏了几个单元
    offset.at(1) = tile_task_info_.y_tiles_per_unit * (unit - 1) * tile_task_info_.tile_direction.direction_y
        + getOffsetOrder(order).at(1);
  }
  return offset;
}


//获取偶数排的第一块的order
int TileOneUnit::getEvenRowFirstOrder()
{
  int order;
  if (tile_task_info_.tile_direction.direction == 0)
  {
    order = (tile_task_info_.y_tiles_per_unit - 1) * tile_task_info_.x_tiles_per_unit + 1;
  }
  else
  {
    order = (tile_task_info_.x_tiles_per_unit - 1) * tile_task_info_.y_tiles_per_unit + 1;
  }
  ROS_INFO("even row first tile order: %d", order);
  return order;
}

//x y方向的异或运算 1 -1计算出来是1， 1 1 计算出来是-1
int TileOneUnit::getSignOfDirection(int x, int y)
{
  int result;
  if (x == y)
  {
    result = -1;
  }
  else
  {
    result = 1;
  }
  return result;
}
std::array<int, 2> TileOneUnit::getTwoRelativeOrder(int base_order, int target_order)
{
  //分别计算base_order target_order所在的行和列
  std::array<int, 2> result = {0, 0};
  if (tile_task_info_.tile_direction.direction == 0)
  {
    //从x方向贴砖，计算x方向的偏移，先计算在哪一列
    //           计算y方向的偏移，计算在哪一行
    std::array<int, 2> line = {0 ,0};
    line.at(0) = (base_order - 1) / tile_task_info_.y_tiles_per_unit + 1;
    line.at(1) = (target_order - 1) / tile_task_info_.y_tiles_per_unit + 1;
    std::array<int, 2> row = {0 ,0};
    row.at(0) = (base_order - 1) % tile_task_info_.x_tiles_per_unit + 1;
    row.at(1) = (target_order - 1) % tile_task_info_.x_tiles_per_unit + 1;
    result.at(0) = (line.at(1) - line.at(0)) * tile_task_info_.tile_direction.direction_x;
    result.at(1) = -(row.at(1) - row.at(0)) * tile_task_info_.tile_direction.direction_y;
  }
  else
  {
    std::array<int, 2> line = {0 ,0};
    line.at(0) = (base_order - 1) / tile_task_info_.x_tiles_per_unit + 1;
    line.at(1) = (target_order - 1) /tile_task_info_.x_tiles_per_unit + 1;
    std::array<int, 2> row = {0 ,0};
    row.at(0) = (base_order - 1) % tile_task_info_.y_tiles_per_unit + 1;
    row.at(1) = (target_order - 1) % tile_task_info_.y_tiles_per_unit + 1;
    result.at(0) = -(line.at(1) - line.at(0))* tile_task_info_.tile_direction.direction_x;
    result.at(1) = (row.at(1) - row.at(0)) * tile_task_info_.tile_direction.direction_y;
  }
  return result;
}
//获取理想的贴砖位置，贴第一单元第一块不调用，first_ref true为所有单元每一块都根据第一块来贴，false为每个单元第一块根据上一单元的第一块来贴，该单元其他按照该单元第一块来贴
geometry_msgs::PoseStamped TileOneUnit::getIdealTilingPos(unsigned int row, unsigned int unit, unsigned int order, bool first_ref)
{
  geometry_msgs::PoseStamped p = first_room_pose_;
  //ROS_WARN_STREAM("getIdealPose before first room pose " << p);
  if (first_ref)
  {
    if (tile_task_info_.tile_direction.direction == 0) //沿着x方向贴
    {
      p.pose.position.x += getOffsetRowUnit(row, unit, order).at(0) * (tile_inch_info_.tile_width + tile_inch_info_.tile_gap);
      p.pose.position.y += getOffsetRowUnit(row, unit, order).at(1) * (tile_inch_info_.tile_length + tile_inch_info_.tile_gap);
    }
    else//沿着y方向贴
    {
      p.pose.position.x += getOffsetRowUnit(row, unit, order).at(0) * (tile_inch_info_.tile_length + tile_inch_info_.tile_gap);
      p.pose.position.y += getOffsetRowUnit(row, unit, order).at(1) * (tile_inch_info_.tile_width + tile_inch_info_.tile_gap);
    }

  }
  else
  {
    std::array<int, 2> offset;
    if ( row % 2 != 0) //奇数排
    {
      offset = getOddRowTileOffset(row, unit, order);
    }
    else //偶数排
    {
      offset = getEvenRowTileOffset(row, unit, order);
    }
    ROS_WARN("[r%d u%d o%d] offset [%d %d]", row, unit, order, offset[0], offset[1]);
    if (tile_task_info_.tile_direction.direction == 0)
    {
      p.pose.position.x += offset.at(0) * (tile_inch_info_.tile_length + tile_inch_info_.tile_gap);
      p.pose.position.y += offset.at(1) * (tile_inch_info_.tile_width + tile_inch_info_.tile_gap);
    }
    else
    {
      p.pose.position.x += offset.at(0) * (tile_inch_info_.tile_width + tile_inch_info_.tile_gap);
      p.pose.position.y += offset.at(1) * (tile_inch_info_.tile_length + tile_inch_info_.tile_gap);
    }
  }
  return p;
}

void TileOneUnit::tileEvenRow()
{
  //分x 方向和 y方向贴砖
  if (tile_task_info_.tile_direction.direction == 0) //x方向
  {
    for (int i = 1; i <= tile_task_info_.x_tiles_per_unit; ++i)
    {
      for (int j = getEvenRowFirstOrder() - (i - 1) * tile_task_info_.y_tiles_per_unit;
           j <= getEvenRowFirstOrder() + (2 - i) * tile_task_info_.y_tiles_per_unit - 1 ; ++j)
      {
        one_tile_finished_ = false;
        if (j == getEvenRowFirstOrder())
        {
          //tile_interface_->suckPerUnitFirstTile();
            ros::Duration(1.0).sleep();
        }
        else
        {
          //tile_interface_->suckUpTile();
           ros::Duration(1.0).sleep();
        }
        ROS_INFO("suck up tile...");
        //ros::Duration(1.0).sleep();
        geometry_msgs::PoseStamped release_pose;
        release_pose = getIdealTilingPos(row_order_, unit_order_, j, false);
       // tile_interface_->releaseTile(release_pose, row_order_, unit_order_, j);
        ROS_INFO("release tile......");
        //ros::Duration(2.0).sleep();
        ros::Duration(20.0).sleep();
        ROS_INFO_STREAM("evn ideal pose:" << release_pose);
        //单元第一块更新坐标
        if (j == getEvenRowFirstOrder())
        {
         // tile_interface_->getPose();
         // first_room_pose_ = release_pose;
         // tile_interface_->getPose();
         // first_room_pose_ = tile_interface_->getFirstTileRoomPose();
        }
        one_tile_finished_ = true;
        //更新贴砖状态
        tile_state_info_.row_order = row_order_;
        tile_state_info_.tile_order = j;
        tile_state_info_.unit_order = unit_order_;
        ROS_ERROR("tile_state: [%d %d %d]", row_order_, unit_order_, j);
        //等待给android发贴砖状态
        if (j != (getEvenRowFirstOrder() + (2 - tile_task_info_.x_tiles_per_unit) * tile_task_info_.y_tiles_per_unit - 1))
        {
          ros::Duration(0.01).sleep();
        }
        //考虑到暂停发生在贴完一块后，在贴砖过程中无法进行暂停，所以在这里做判断
        if (tiling_state_ == PAUSE && j != (getEvenRowFirstOrder() + (2 - tile_task_info_.x_tiles_per_unit) * tile_task_info_.y_tiles_per_unit - 1))
        {
          one_tile_finished_ = false;
          ROS_INFO("now, pause tile, waiting for resume ......");
          while (tiling_state_ != RESUME)
          {
            ros::Duration(0.01).sleep();
          }
        }
      }
    }
    one_unit_tile_finished_ = true;
   // one_tile_finished_ = false;
  }
  else //y方向
  {
    for (int i = 1; i <= tile_task_info_.y_tiles_per_unit; ++i)
    {
      for (int j = getEvenRowFirstOrder() - (i - 1) * tile_task_info_.x_tiles_per_unit;
           j <= getEvenRowFirstOrder() + (2 - i) * tile_task_info_.x_tiles_per_unit - 1; ++j)
      {
        one_tile_finished_ = false;
        if (j == getEvenRowFirstOrder())
        {
          tile_interface_->suckPerUnitFirstTile();
        }
        else
        {
          tile_interface_->suckUpTile();
        }
        geometry_msgs::PoseStamped release_pose;
        release_pose = getIdealTilingPos(row_order_, unit_order_, j, false);
        tile_interface_->releaseTile(release_pose, row_order_, unit_order_, j);
        one_tile_finished_ = true;
        ////单元第一块更新坐标
        if (j == getEvenRowFirstOrder())
        {
          tile_interface_->getPose();
          first_room_pose_ = tile_interface_->getFirstTileRoomPose();
        }
        //考虑到暂停发生在贴完一块后，在贴砖过程中无法进行暂停，所以在这里做判断
        if (tiling_state_ == PAUSE && j != (getEvenRowFirstOrder() + (2 - i) * tile_task_info_.x_tiles_per_unit - 1))
        {
          one_tile_finished_ = false;
          ROS_INFO("now, pause tile, waiting for resume ......");
          while (tiling_state_ != RESUME)
          {
            ros::Duration(0.01).sleep();
          }
        }
      }
    }
  }
}
//贴奇数排
void TileOneUnit::tileOddRow()
{
  for (int i = 1; i <= tile_task_info_.x_tiles_per_unit * tile_task_info_.y_tiles_per_unit; ++i)
  {
    tiling_state_ = TILING;
    one_tile_finished_ = false;
    if (unit_order_ == 1 && i == 1) //第一单元第一块
    {
      ROS_INFO("start tile 1 unit 1 tile,suck up tile......");
     // tile_interface_->suckUpFirstTile();
      ros::Duration(1.0).sleep();
      ROS_INFO("release 1th unit 1th tile");
      ros::Duration(10.0).sleep();
    //  tile_interface_->releaseFirstTile();
     // one_tile_finished_ = true;
     // ros::Duration(0.01).sleep();
    }
    else
    {
      if ( i == 1)
      {
       // tile_interface_->suckPerUnitFirstTile();
        ros::Duration(1.0).sleep();
      }
      else
      {
       // tile_interface_->suckUpTile();
        ros::Duration(1.0).sleep();
      }
      ROS_INFO("suck up tile [%d %d]", unit_order_, i);
      geometry_msgs::PoseStamped release_pose;
      release_pose = getIdealTilingPos(row_order_, unit_order_, i, false);
//      if (i == 1)
//      {
//        //first_room_pose_ = release_pose;
//        tile_interface_->getPose();
//        first_room_pose_ = tile_interface_->getFirstTileRoomPose();
//      }
      ROS_INFO_STREAM("ideal tiling pose:" << release_pose);
    //  tile_interface_->releaseTile(release_pose, row_order_, unit_order_, i);
      ros::Duration(10.0).sleep();
      ROS_INFO("release tile");
    //  one_tile_finished_ = true;
    //  ros::Duration(0.01).sleep();
    }
    //
    if (i == 1)
    {
       //first_room_pose_ = release_pose;
     // tile_interface_->getPose();
     // first_room_pose_ = tile_interface_->getFirstTileRoomPose();
    }

    //更新贴砖状态
    tile_state_info_.row_order = row_order_;
    tile_state_info_.tile_order = i;
    tile_state_info_.unit_order = unit_order_;
    ROS_WARN("UPDATE TILE STATE: ROW:%d TILE:%d UNIT:%d", row_order_, i, unit_order_);
    ROS_ERROR("UPDATE TILE STATE: ROW:%d TILE:%d UNIT:%d", row_order_, i, unit_order_);
    one_tile_finished_ = true;
    //本单元最后一块
    if (i != tile_task_info_.x_tiles_per_unit * tile_task_info_.y_tiles_per_unit)
    {
      ros::Duration(0.01).sleep();
    }
    //考虑到暂停发生在贴完一块后，在贴砖过程中无法进行暂停，所以在这里做判断
    if (tiling_state_ == PAUSE && i != tile_task_info_.x_tiles_per_unit * tile_task_info_.y_tiles_per_unit )
    {
      ROS_INFO("now, pause tile, waiting for resume ......");
//      current_tile_num_ = i;
//      current_row_num_ = row_order_;
//      current_unit_num_ = unit_order_;
//      break;
      one_tile_finished_ = false;
      while (tiling_state_ != RESUME)
      {
        ros::Duration(0.01).sleep();
      }
    }
    //
    if (tiling_state_ == CANCEL)
    {
      ROS_INFO("now, cancel tile!");
      break;
    }
  }
  one_unit_tile_finished_ = true;
  //one_tile_finished_ = false;
  //ros::Duration(0.01).sleep();
}

//获取奇数排的单元内的第一块相对于上单元第一块的位置和
//单元内其他块相对第一块的位置
std::array<int, 2> TileOneUnit::getOddRowTileOffset(int row, int unit, int order)
{
  std::array<int, 2> offset = {0, 0};
  if (unit != 1 && order == 1/*&& row != 1*/) //不是第一排，不是第一个单元，但是第一块
  {
    if (tile_task_info_.tile_direction.direction == 0) //沿着x方向贴
    {
       offset.at(0) = tile_task_info_.x_tiles_per_unit * tile_task_info_.tile_direction.direction_x;
       offset.at(1) = 0;
    }
    else //沿着y方向贴
    {
      offset.at(0) = 0;
      offset.at(1) = tile_task_info_.y_tiles_per_unit * tile_task_info_.tile_direction.direction_y;
    }
  }
  else if (unit == 1 && order == 1 && row != 1) //不是第一排,但是第一个单元，第一块
  {
    //沿着x方向贴
    //x方向 正负号取决于x负方向
    //y方向 正负号取决于y负方向
    if (tile_task_info_.tile_direction.direction == 0)
    {
       offset.at(0) = - (tile_task_info_.x_tiles_per_unit - 1) * tile_task_info_.tile_direction.direction_x;
       offset.at(1) = - (tile_task_info_.y_tiles_per_unit - 1) * tile_task_info_.tile_direction.direction_y;
    }
    else //沿着y方向贴, x方向正负号取决于x y方向的异或， y方向正负号取决于y方向的负号
    {
      offset.at(0) = (tile_task_info_.x_tiles_per_unit - 1) * getSignOfDirection(
            tile_task_info_.tile_direction.direction_x, tile_task_info_.tile_direction.direction_y);
      offset.at(1) = - (tile_task_info_.y_tiles_per_unit - 1) * tile_task_info_.tile_direction.direction_y;
    }
  }
  else
  {
    offset = getOffsetOrder(order);
  }
  return offset;
}

//获取偶数排的单元内的第一块相对于上单元第一块的位置和
//单元内其他块相对第一块的位置
std::array<int, 2> TileOneUnit::getEvenRowTileOffset(int row, int unit, int order)
{
  std::array<int, 2> offset = {0, 0};
  ROS_WARN("order:%d even_first_order:%d", order, getEvenRowFirstOrder());
  if (order == getEvenRowFirstOrder() && unit == 1) //偶数排第一单元第一块
  {
    if (tile_task_info_.tile_direction.direction == 0) //沿着x方向贴
    {
      offset.at(0) = (tile_task_info_.x_tiles_per_unit - 1) * tile_task_info_.tile_direction.direction_x;
      offset.at(1) = -(tile_task_info_.y_tiles_per_unit) * tile_task_info_.tile_direction.direction_y;
    }
    else //沿着y方向贴
    {
      offset.at(0) = (tile_task_info_.x_tiles_per_unit) * getSignOfDirection(
            tile_task_info_.tile_direction.direction_x, tile_task_info_.tile_direction.direction_y);
      offset.at(1) = (tile_task_info_.y_tiles_per_unit - 1) * tile_task_info_.tile_direction.direction_y;
    }
  }
  else if (order == getEvenRowFirstOrder() && unit != 1)
  {
    if (tile_task_info_.tile_direction.direction == 0)
    {
      offset.at(0) = -(tile_task_info_.x_tiles_per_unit) * tile_task_info_.tile_direction.direction_x;
      offset.at(1) = 0;
    }
    else
    {
      offset.at(0) = 0;
      offset.at(1) = -(tile_task_info_.y_tiles_per_unit) * tile_task_info_.tile_direction.direction_y;
    }
  }
  else //偶数排内的其他块
  {
     offset = getTwoRelativeOrder(getEvenRowFirstOrder(), order);
  }
  return offset;
}

void TileOneUnit::letArmBackToIntialPose()
{
  tile_interface_->backToInitPose();
}
