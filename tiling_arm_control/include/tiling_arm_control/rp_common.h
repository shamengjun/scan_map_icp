#ifndef DOWN_TOUCH_RP_COMMON_H_
#define DOWN_TOUCH_RP_COMMON_H_
#include "string"

namespace tiling_arm_control
{
//定义事件code
enum EventCode
{
  START_TILING  = 1001,
  CANCEL_TILING = 1002,
  PAUSE_TILING = 1003,
  RESUME_TILING = 1004,
  FINISH_TILING = 1005,

  SLAVE_MOVE_READY = 2001,
  SLAVE_MOVE_AWAY = 2002,
  SLAVE_MOVE_ASIDE = 2003,
  MASTER_READY_READY = 2004,

  WAITING_NOZZEL_POSE = 3001,
  NOZZEL_POSE = 3002,
  REQUEST_INJECT_MORTAR = 3003,
  INJECTING_MORTAR = 3004,
  REQUEST_STOP_INJECT_MORTAR = 3005,
  FINISHED_INJECT_MORTAR = 3006,
  ADD_MORTAR_AND_TILE = 3007,
  ADD_FINISHED = 3008,

  TILING_STATE = 4001
  //后面再添加扩展
};
//定义机器人的状态
enum RobotState
{
  EMPTY = 0, //水泥和砖块用尽
  FULL = 1, //水泥和砖块充足
  MOVE = 1, //移动
  IDLE = 1, //空闲
  ENABLE = 1,  //使能
  DISENABLE = 2, //未使能
  UNKNOWN = -1 //未知
};
//定义error code
enum ErrorCode
{
  WHEEL_NOT_MOVE = 1000,
  ODOM_LOST = 1001,
  IMU_LOST = 1002,
  SCAN_LOST = 1003,

  LOCALIZATION_LOST = 2000,
  MOVE_BASE_FAILED = 2001,

  ARM_MOVE_FAILED = 3000,
  ARM_START_FAILED = 3001,

  CAMERA_LOST = 4000,
  CAMETA_DETECT_LOST = 4001
  //后面在继续扩展
};

//定义贴砖的状态
enum TileState
{
  IDLED = 10,
  START = 20,
  TILING = 30,
  PAUSE = 40,
  CANCEL = 50,
  RESUME = 60,
  FINISHED = 70
};

//定义贴砖方向
struct TileDirection
{
  int direction = 0; //默认是沿着x方向贴, 1为沿着y方向贴,标识别车的朝向
  int direction_x = -1; //-1 代表x的负方向， 1 代表x的正方向
  int direction_y = 1; //-1 代表y的负方向， 1 代表y的正方向
};

//定义地砖的信息
struct TileInchInfo
{
  double tile_length; //地砖长度
  double tile_width;  //地砖宽度
  double tile_height; //地砖厚度
  double tile_gap; //地砖间的缝隙宽度
};

//定义每次贴砖的任务信息
struct TileTaskInfo
{
  int rows; //要贴多少排
  int units_per_row;//每排贴多少单元
  int x_tiles_per_unit; //每个单元x方向贴几块
  int y_tiles_per_unit; //每个单元y方向贴几块
  TileDirection tile_direction; //贴砖方向"x" or "y"
};

struct TileStateInfo
{
  int row_order; //贴到多少排
  int unit_order; //贴到多少单元
  int tile_order; //贴到多少块
};

}
#endif // RP_COMMON_H
