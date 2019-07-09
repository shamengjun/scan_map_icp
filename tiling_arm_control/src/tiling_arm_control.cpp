#include "tiling_arm_control/tiling_arm_control.h"

using namespace tiling_arm_control;

TilingArmControl::TilingArmControl()
{
  initial();
}

TilingArmControl::~TilingArmControl()
{

}

void TilingArmControl::initial()
{
  loadParam();
  initialFirstSuckPose();
  initialFirstTilepose();
 //std::shared_ptr<tile_interface::TileInterface> intial =
  //    std::make_shared<tile_interface::TileInterface>();
   //intial->initValue(first_suck_pose_, first_tile_pose_, bounce_dis_, tool_height_);
  //intial->setFirstTileRoomPose(first_tile_pose_);
  ros::AsyncSpinner spinner(4);
  spinner.start();
  tiling_event_sub_ = nh_.subscribe<std_msgs::String>("tiling_event_base", 1, &TilingArmControl::tilingBaseEventSubCallback, this);
  slave_tiling_event_sub_ = nh_.subscribe<std_msgs::String>("tiling_event_base_slave", 1, &TilingArmControl::tilingSlaveBaseEventSubCallback, this);
  android_tiling_event_sub_ = nh_.subscribe<std_msgs::String>("tiling_event_android", 1, &TilingArmControl::androidTilingEventSubCallback, this);
  tiling_event_pub_ = nh_.advertise<std_msgs::String>("tiling_control", 1);
  tiling_base_sub_ = nh_.subscribe<std_msgs::String>("tiling_base_event", 1, &TilingArmControl::tilingBaseSubCallback, this);
  tiling_arm_pub_ = nh_.advertise<std_msgs::String>("tiling_arm_event", 1);
  test_pub_msg_sub_ = nh_.subscribe<std_msgs::String>("test_pub", 1, &TilingArmControl::testPubMsgSubCallback, this);
  //单元内贴砖控制命令发布
  tile_cmd_pub_ = nh_.advertise<std_msgs::String>("tile_cmd", 1);
  is_finished_ = false;
  one_row_ = false;
  one_unit_ = false;
  is_start_ = false;
  is_base_move_finished_ = false;
  current_row_order_ = 0;
  current_unit_order_ = 0;
  finished_num_ = 0;

  tile_thread_ = std::make_shared<std::thread>(&TilingArmControl::tileThread, this);
  tile_state_thread_ = std::make_shared<std::thread>(&TilingArmControl::getTileState, this);
// tile_control_thread_ = std::make_shared<std::thread>(&TilingArmControl::runLoop, this);
//  tile_thread_->join();
//  tile_state_thread_->join();
//  tile_control_thread_->join();
}

void TilingArmControl::loadParam()
{
  nh_.param("tile_info_tile_length", tile_inch_.tile_length, 0.305);
  nh_.param("tile_info_tile_width", tile_inch_.tile_width, 0.305);
  nh_.param("tile_info_tile_height", tile_inch_.tile_height, 0.1);
  nh_.param("tile_info_tile_gap", tile_inch_.tile_gap, 0.005);
  //nh_.param("one_unit_tile_num", one_unit_tile_num_, 4);
  nh_.param("tile_rows", tile_task_.rows, 2);
  nh_.param("tile_units_per_row", tile_task_.units_per_row, 4);
  nh_.param("tile_x_tiles_per_unit", tile_task_.x_tiles_per_unit, 2);
  nh_.param("tile_x_tiles_per_unit", tile_task_.y_tiles_per_unit, 2);
  nh_.param("tile_direction", tile_task_.tile_direction.direction, 0); // x 方向
  nh_.param("tile_direction_x", tile_task_.tile_direction.direction_x, -1); // x 负方向
  nh_.param("tile_direction_y", tile_task_.tile_direction.direction_y, -1); // y 正方向
  //nh_.param("one_row_tile_num", one_row_tile_num_, 4);
  //@todo 更多的参数
 // nh_.param("down_value_", down_value_, 0.03);
 // nh_.param("adjust_value_", adjust_value_, 0.01);
 // nh_.param("near_value_", near_value_, 0.2);
  nh_.param("bounce_dis", bounce_dis_, 0.002);
  nh_.param("tool_height", tool_height_, 0.5);
 // nh_.param("deviance", deviance_, 0.015);


}

void TilingArmControl::initialFirstSuckPose()
{
  ROS_INFO("initial first suck pose");
  nh_.param("first_suck_tile_pose_header_frame_id", first_suck_pose_.header.frame_id, std::string("/world"));
  nh_.param("first_suck_tile_pose_pose_position_x", first_suck_pose_.pose.position.x, -0.30735);
  nh_.param("first_suck_tile_pose_pose_position_y", first_suck_pose_.pose.position.y, 1.12083);
  nh_.param("first_suck_tile_pose_pose_position_z", first_suck_pose_.pose.position.z, -0.450);
  double Rx, Ry, Rz;
  nh_.param("first_suck_tile_pose_pose_orientation_Rx", Rx, 1.3418);
  nh_.param("first_suck_tile_pose_pose_orientation_Ry", Ry, 2.8594);
  nh_.param("first_suck_tile_pose_pose_orientation_Rz", Rz, -0.0125);
  first_suck_pose_.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(Rx, Ry, Rz);
  ROS_INFO("TilingControl readInitPose suck_tile_pose %lf %lf %lf %lf %lf %lf",
           first_suck_pose_.pose.position.x, first_suck_pose_.pose.position.y, first_suck_pose_.pose.position.z, Rx, Ry, Rz);

}

void TilingArmControl::initialFirstTilepose()
{
  nh_.param("first_tiling_pos_header_frame_id", first_tile_pose_.header.frame_id, std::string("/world"));
  nh_.param("first_tiling_pos_pose_position_x", first_tile_pose_.pose.position.x, 0.51);
  nh_.param("first_tiling_pos_pose_position_y", first_tile_pose_.pose.position.y, 0.0);
  nh_.param("first_tiling_pos_pose_position_z", first_tile_pose_.pose.position.z, 0.7);
  double Rx, Ry, Rz;
  nh_.param("first_tiling_pos_pose_orientation_Rx", Rx, 3.103);
  nh_.param("first_tiling_pos_pose_orientation_Ry", Ry, 0.019);
  nh_.param("first_tiling_pos_pose_orientation_Rz", Rz, 2.294);
  ROS_INFO("TilingControl readInitPose tiling_pos %lf %lf %lf %lf %lf %lf",
           first_tile_pose_.pose.position.x, first_tile_pose_.pose.position.y, first_tile_pose_.pose.position.z, Rx, Ry, Rz);
  first_tile_pose_.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(Rx, Ry, Rz);
  //last_unit_first_pose_ = first_tile_pose_;
}

void TilingArmControl::upArm()
{

}

void TilingArmControl::publishWaitNozzelPoseEvent()
{
  Json::Value json_msg;
  json_msg["event_type"] = "control";
  json_msg["event_code"] = WAITING_NOZZEL_POSE;
  json_msg["event_context"] = "waiting for detecting nozzel's pose";
  std_msgs::String msg;
  msg.data = json_msg.toStyledString();
  tiling_event_pub_.publish(msg);
}

void TilingArmControl::publishNozzelPoseEvent(const geometry_msgs::Pose &pose)
{
  Json::Value root;
  Json::Value event_msg;
  event_msg["event_type"] = "control";
  event_msg["event_code"] = NOZZEL_POSE;
  event_msg["event_context"] = "publish nozzel's pose";

  Json::Value nozzel_pose;
  Json::Value pose_msg;
  Json::Value position;
  position["x"] = pose.position.x;
  position["y"] = pose.position.y;
  position["z"] = pose.position.z;

  Json::Value orientation;
  double roll, pitch, yaw;
  tf::Matrix3x3 mat(tf::Quaternion(pose.orientation.x,
                                   pose.orientation.y,
                                   pose.orientation.z,
                                   pose.orientation.w));
  mat.getRPY(roll, pitch, yaw);
  orientation["roll"] = roll;
  orientation["pitch"] = pitch;
  orientation["yaw"] = yaw;

  pose_msg["position"].append(position);
  pose_msg["orientation"].append(orientation);

  nozzel_pose["nozzel_pose"].append(pose_msg);

  root.append(event_msg);
  root.append(nozzel_pose);

  std_msgs::String msg;
  msg.data = root.toStyledString();
  tiling_event_pub_.publish(msg);
}

void TilingArmControl::publishWaitSlaveMortarEvent()
{
  //Json::Value root;
  Json::Value json_msg;
  json_msg["event_type"] = "control";
  json_msg["event_code"] = REQUEST_INJECT_MORTAR;
  json_msg["event_context"] = "requst slave base to inject mortar";

  std_msgs::String msg;
  msg.data = json_msg.toStyledString();
  tiling_event_pub_.publish(msg);
}

void TilingArmControl::publishAddMortarAndTile()
{
  Json::Value json_msg;
  json_msg["event_type"] = "control";
  json_msg["event_code"] = ADD_MORTAR_AND_TILE;
  json_msg["event_context"] = "requst to add mortar and tile";

  std_msgs::String msg;
  msg.data = json_msg.toStyledString();
  tiling_event_pub_.publish(msg);
}


void TilingArmControl::publishTilingState()
{
  Json::Value json_msg;
  json_msg["event_type"] = "control";
  json_msg["event_code"] = TILING_STATE;
  json_msg["event_context"] = "tiling state description";
  Json::Value status_msg;
  Json::Value tiling_state_context;
  tiling_state_context["total_num"] = total_num_;
  tiling_state_context["finished_num"] = finished_num_;

  Json::Value finish_state_context;
  finish_state_context["row"] = row_num_;
  finish_state_context["unit"] = unit_num_;
  finish_state_context["tile"] = tile_num_;
  //tiling_state_context["finished_state"].append(finish_state_context);
  tiling_state_context["finished_state"] = finish_state_context;

  tiling_state_context["is_one_unit"] = one_unit_;
  tiling_state_context["is_one_row"] = one_row_;
  tiling_state_context["is_finished"] = is_finished_;

  //json_msg["tiling_state"].append(tiling_state_context);
  json_msg["tiling_state"] = tiling_state_context;

  std_msgs::String msg;
  msg.data = json_msg.toStyledString();
  tiling_event_pub_.publish(msg);

}


bool TilingArmControl::tilingOneUnit()
{
//  if ( finished_num_ % one_unit_tile_num_ == 0)
//  {
//    ROS_INFO("one unit has tiled finish!");
//    one_unit_ = true;
//  }
//  else
//  {
//    ROS_WARN("one unit has not finished!");
//    one_unit_ = false;
//  }
  return one_unit_;
}

bool TilingArmControl::tilingOneRow()
{
//  if (finished_num_ % one_row_tile_num_ == 0)
//  {
//    ROS_INFO("one row has tiled finish!");
//    one_row_ = true;
//  }
//  else
//  {
//    ROS_WARN("one row has not finished!");
//    one_row_ = false;
//  }
  return one_row_;
}

bool TilingArmControl::tilingAllFinish()
{
//  if (finished_num_ == total_num_)
//  {
//    ROS_INFO("good! tile finish!");
//    is_finished_ = true;
//  }
//  else
//  {
//    ROS_WARN("all are not finished! come on!");
//    is_finished_ = false;
//  }
  return is_finished_;
}

void TilingArmControl::androidTilingEventSubCallback(const std_msgs::String::ConstPtr &msg)
{
//  std::string json_msg = msg->data;
//  ROS_INFO("receive from android: %s", json_msg.c_str());
//  Json::Reader reader;
//  Json::Value json_value;
//  reader.parse(json_msg, json_value);
//  if (json_value["event_type"].asString() != "android")
//  {
//    ROS_WARN("auth identity failed!, are you true this message from android but from %s", json_value["event_type"].asString().c_str());
//    return;
//  }
//  else
//  {
//    ROS_INFO("auth identity succeed!, this message from andoid");
//    current_event_state_ = json_value["event_code"].asInt();
//    ROS_INFO("event_code: %d", current_event_state_);
//  }
  ROS_INFO("receive msg %s", msg->data.c_str());
  if (msg->data == "1000")
  {
    ROS_INFO(":::");
    current_event_state_ = START_TILING;
    ROS_INFO("current_event_code:%d", current_event_state_);
  }
  else if (msg->data == "1003")
  {
    ROS_INFO(":::");
    current_event_state_ = PAUSE_TILING;
    ROS_INFO("current_event_code:%d", current_event_state_);
  }
  else if (msg->data == "1004")
  {
    ROS_INFO(":::");
    current_event_state_ = RESUME_TILING;
    ROS_INFO("current_event_code:%d", current_event_state_);
  }
  handleEvent();
}

void TilingArmControl::tilingBaseEventSubCallback(const std_msgs::String::ConstPtr &msg)
{
  std::string json_msg = msg->data;
  ROS_INFO("receive from tiling_base: %s", json_msg.c_str());
  Json::Reader reader;
  Json::Value json_value;
  reader.parse(json_msg, json_value);
  if (json_value["event_type"].asString() != "base")
  {
    ROS_WARN("auth identity failed!, are you true this message from tiling_base but from %s", json_value["event_type"].asString().c_str());
    return;
  }
  else
  {
    ROS_INFO("auth identity succeed!, this message from tiling_base");
    current_event_state_ = json_value["event_code"].asInt();
    //@todo handle
    handleEvent();
  }
}

void TilingArmControl::tilingSlaveBaseEventSubCallback(const std_msgs::String::ConstPtr &msg)
{
  std::string json_msg = msg->data;
  ROS_INFO("receive from tiing_base_slave: %s", json_msg.c_str());
  Json::Reader reader;
  Json::Value json_value;
  reader.parse(json_msg, json_value);
  if (json_value["event_type"].asString() != "base_slave")
  {
    ROS_WARN("auth identity failed!, are you true this message from android but from %s", json_value["event_type"].asString().c_str());
    return;
  }
  else
  {
    ROS_INFO("auth identity succeed!, this message from andoid");
    current_event_state_ = json_value["event_code"].asInt();
    //@todo handle
    /*
    switch (current_event_state_)
    {
    case SLAVE_MOVE_ASIDE: //小工移到大工旁边
      upArm();
      break;
    case INJECTING_MORTAR: //小工开始喷水泥
      snakeMortar();
      break;
    case SLAVE_MOVE_READY: //小工移到下一个目标点就位
      upArm();
      break;
    default:
      break;
    }
    */
    handleEvent();
  }
}

bool TilingArmControl::tiling()
{
  //贴一个单元的砖逻辑控制
}


void TilingArmControl::handleEvent()
{
  ROS_INFO("hanle tile event");
  //ROS_INFO("current event code:%d",current_event_state_);
  switch (current_event_state_)
  {
  case START_TILING:  //开始贴砖,最大问题该函数还没有返回，下面是没法调用的
    handleStartTile();
    break;
  case CANCEL_TILING: //取消贴砖
    handleCancelTile();
    break;
  case PAUSE_TILING: //暂停贴砖
    handlePauseTile();
    break;
  case RESUME_TILING: //恢复贴砖
    handleResumeTile();
    break;
  case SLAVE_MOVE_ASIDE: //小工移到大工旁边
    upArm();
    break;
  case INJECTING_MORTAR:  //小工开始喷水泥
    snakeMortar();
    break;
  case SLAVE_MOVE_READY: //小工移到下一个目标点就位
    upArm();
    break;
  default:
    break;
  }
}

bool TilingArmControl::snakeMortar()
{

}

void TilingArmControl::tileOneUnit(int row, int unit)
{
  //创建一个贴一个单元的对象指针
  std::shared_ptr<tile_one_unit::TileOneUnit> tile_one_unit = std::make_shared<tile_one_unit::TileOneUnit>(row, unit);
  tile_unit_ = tile_one_unit;
  //tile_one_unit->setTilingDirection(tile_task_.tile_direction);
  tile_one_unit->initTilingArmParams(first_suck_pose_, first_tile_pose_, bounce_dis_, tool_height_);
  tile_one_unit->setFirstRoomPose(last_unit_first_pose_);
  tile_one_unit->setTileTaskInfo(tile_task_);
  tile_one_unit->setTileInchInfo(tile_inch_);
  tile_one_unit->startTiling();
  //tile_one_unit->tilingControlCmd("start");
  if (tile_one_unit->getOneUnitFinished())
  {
    ROS_INFO("r%d u%d has finished tile!", row, unit);
    one_unit_ = true;
    //等待更新一个单元完成状态
    ros::Duration(0.01).sleep();
  }
  //一个单元贴完，获取本单元的第一块的位置
  last_unit_first_pose_ = tile_one_unit->getFirstRoomPose();
  // let arm back to initial pose
  //tile_one_unit->letArmBackToIntialPose();
  //销毁对象
  tile_unit_ = nullptr;
}

void TilingArmControl::tileThread()
{
  // @todo 任务校验
  ROS_INFO("this is tile thread!");
  while (ros::ok())
  {
    if (is_start_ && !is_finished_)
    {
      ROS_INFO("print this tile task's info list: \n "
               "you want to tile %d rows \n"
               "%d units per rows \n"
               "%d * %d per units\n"
               "so total want to tile %d tiles", tile_task_.rows, tile_task_.units_per_row
               , tile_task_.x_tiles_per_unit, tile_task_.y_tiles_per_unit,
               tile_task_.x_tiles_per_unit * tile_task_.y_tiles_per_unit * tile_task_.units_per_row * tile_task_.rows);
      total_num_ = tile_task_.x_tiles_per_unit * tile_task_.y_tiles_per_unit * tile_task_.units_per_row * tile_task_.rows;
      //本次贴之前，将is_finished置为false
      is_finished_ = false;
      for (int i = 1; i <= tile_task_.rows; ++i)
      {
        //每贴一排前，将one_row置为false
        one_row_ = false;
        for (int j = 1; j <= tile_task_.units_per_row; ++j)
        {
          ROS_INFO("start to tile r%d u%d", i, j);
          //每贴完一个单元前，one_unit置为false
          one_unit_ = false;
          //收到底盘消息后，再开始一个单元贴砖,不然一直等待base move finishe
          bool print_once = true;
          while (!is_base_move_finished_)
          {
            if (print_once)
            {
              ROS_ERROR("waiting for base move finished......");
              print_once = false;
            }
            ros::Duration(0.5).sleep();
          }
          is_base_move_finished_ = false;
          //贴一个单元
          tileOneUnit(i, j);
          //wait for arm move to initial
          ros::Duration(2.0).sleep();
          //一个单元贴完，通知base开始走
          if (j != tile_task_.units_per_row)
          {
            publishBaseEvent(i, j);
          }
          // 只能在贴完一个单元后进行暂停
          if (current_event_state_ == PAUSE_TILING)
          {
            ROS_INFO("finish unit %d , pause tile!");
            //等待resume贴砖
            while (current_event_state_ != RESUME_TILING)
            {
              ros::Duration(0.01).sleep();
            }
          }
        }
        // 一排贴完
        one_row_ = true;
        //发布消息
        if (i != tile_task_.rows)
        {
          publishBaseEvent(i, tile_task_.units_per_row);
        }
      }
      //所有排贴完
      is_finished_ = true;
      //等待更新所有排完成更新状态
      ros::Duration(0.01).sleep();
      is_start_ = false;
    }
    ros::Duration(0.01).sleep();
  }
}

void TilingArmControl::handleStartTile()
{
  //ROS_INFO("start tile!");
  if (!is_finished_)
  {
    is_start_ = true;
  }

}
void TilingArmControl::handlePauseTile()
{
  ROS_INFO("pause tile!");
  // 确保在贴砖过程中调用此方法
  if (tile_unit_ != nullptr)
  {
    //tile_unit_->tilingControlCmd("pause");
    publishTileCmd("pause");
  }
  else
  {
    ROS_ERROR("has not tile_unit object!");
  }
}

void TilingArmControl::handleResumeTile()
{
  ROS_INFO("resume tile!");
  // 确保在贴砖过程中调用此方法
  if (tile_unit_ != nullptr)
  {
    //tile_unit_->tilingControlCmd("resume");
    publishTileCmd("resume");
  }
  else
  {
    ROS_ERROR("has not tile_unit object!");
  }
}

void TilingArmControl::handleCancelTile()
{
  ROS_INFO("cancel tile!");
  // 确保在贴砖过程中调用此方法
  if (tile_unit_ != nullptr)
  {
    //tile_unit_->tilingControlCmd("resume");
    publishTileCmd("cancel");
  }
  else
  {
    ROS_ERROR("has not tile_unit object!");
  }
}

void TilingArmControl::testPubMsgSubCallback(const std_msgs::String::ConstPtr &msg)
{
  ROS_INFO("[****this is the pub control msg test cb!****]");
  if (msg->data == "tiling_state")
  {
    total_num_ = 16;
    finished_num_ = 12;

    row_num_ = 1;
    unit_num_ = 4;
    tile_num_ = 3;

    one_row_ = tilingOneRow();
    one_unit_ = tilingOneUnit();
    is_finished_ = tilingAllFinish();
    //发布
    publishTilingState();
  }
  else if (msg->data == "add")
  {
    publishAddMortarAndTile();
  }
  else
  {
    ROS_ERROR("woah! error!");
  }
}

void TilingArmControl::getTileState()
{
  while (ros::ok())
  {
    if (tile_unit_ != nullptr)
    {
      bool tile_finish_state = tile_unit_->getOneTileFinished();
      if (tile_finish_state) //当一块贴完时发布贴砖状态
      {
        //状态返回
        TileStateInfo state_info = tile_unit_ ->getTileStateInfo();
        //ROS_ERROR("tile state info: row:%d unit:%d tile:%d", state_info.row_order,
        //          state_info.unit_order, state_info.tile_order);
        finished_num_++;
        row_num_ = state_info.row_order;
        unit_num_ = state_info.unit_order;
        tile_num_ = state_info.tile_order;
        publishTilingState();
        tile_finish_state = false;
      }
      one_unit_tile_state_ = tile_unit_->getTileState();
      if (is_finished_)
      {
        //if (tile_unit_ != nullptr)
        //{
        tile_unit_ = nullptr;
        //}
      }
    }
    ros::Duration(0.01).sleep();
  }

}

void TilingArmControl::publishBaseEvent(int row, int unit)
{
  Json::Value json_msg;
  if (one_unit_ && !one_row_)
  {
    json_msg["event"] = "unit_finish";
  }
  else if (one_unit_ && one_row_)
  {
    json_msg["event"] = "row_finish";
  }
  else
  {
    ROS_ERROR("one unit or one row has not finished!");
    return;
  }
  ROS_WARN("publish one row finish msg!");
  Json::Value data;
  data["unit_num"] = unit;
  data["row_num"] = row;
  json_msg["data"].append(data);
  std_msgs::String msg;
  msg.data = json_msg.toStyledString();
  tiling_arm_pub_.publish(msg);
}

void TilingArmControl::tilingBaseSubCallback(const std_msgs::String::ConstPtr& msg)
{
  ROS_INFO("subscribe base event!");
  Json::Reader reader;
  Json::Value json_value;
  if (!reader.parse(msg->data, json_value))
  {
    ROS_ERROR("json parse failed!");
  }
  if (json_value["event"].asString() == "unit_move_finish")
  {
    is_start_ = true;
    is_base_move_finished_ = true;
  }
  else
  {
    ROS_ERROR("invalid msgs!");
  }
}

void TilingArmControl::publishTileCmd(std::string cmd)
{
  std_msgs::String msg;
  msg.data = cmd;
  tile_cmd_pub_.publish(msg);
}
