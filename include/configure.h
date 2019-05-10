/**
 * @file configure.h
 * @author GCUROBOT-Visual-Group (GCUROBOT_WOLF@163.com)
 * @brief RM2019 步兵视觉配置文件
 * @version 1.1
 * @date 2019-05-01
 * @copyright Copyright (c) 2019 GCU Robot Lab. All Rights Reserved.
 */
#ifndef CONFIGURE_H
#define CONFIGURE_H

#include "CameraApi.h"
#include <iostream>
#include <opencv2/video/tracking.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/core.hpp>
#include <math.h>
#include <iostream>
#include <fcntl.h>  //文件控制定义
#include <termios.h>   //POSIX终端控制定义
#include <unistd.h>    //UNIX标准定义
#include <errno.h>     //ERROR数字定义
#include <sys/select.h>

using namespace std;
using namespace cv;

#define ISOPEN_INDUSTRY_CAPTURE 1
/**
  @brief: 是否使用工业相机
  @param: 0 使用工业相机
  @param: 1 使用普通USB相机
*/

#define CAPTURE_DEFULT "/home/hzh/视频/camera_13.avi"
/**
  @brief: 相机的默认值
  @note: 使用普通USB相机时，Opencv的Videoture借口的值
*/

#define ARMOR_COLOR 0
/**
  @brief: 选择敌方阵营
  @param: 0 敌方为红色
  @param: 1 敌方为蓝色
*/

#define SERIAL_IS_OPEN 1
/**
  @brief: 是否启用串口
  @param: 0 不启用
  @param: 1 启用
*/

#define ALL_DEFAULT 1

/**
 * @brief 数据位选择
 */
enum bits_No{
    mode_case = 1,
    armor_size_case = 2,
};

/**
 * @brief 装甲板区分
 * @note 默认为小装甲
 */
enum armor_size{
    little_armor = 1,
    big_armor = 2,
    armor_default = little_armor,
};

/**
 * @brief 步兵视觉模式切换
 * @note 默认为辅助瞄准模式
 */
enum receive_serial_type{
    support_shooting_mode = 1,
    energy_agency_mode = 2,
    mode_default = support_shooting_mode,
};

/**
 * @brief 大神符部分枚举
 */
enum big_chrysanthemum{
    //*quadrant_name
    first_quadrant = 1,
    second_quadrant = 2,
    third_quadrant = 3,
    fourth_quadrant = 4,
    origin = 5,

    //*change_follow_fixed_mode
    fixed_mode = 6,
    follow_mode = 7,

    //*send_mode
    send_none = 8,
    send_fixed_center = 9,
    send_shoot_point = 10,
    send_back_to_center = 11,

    //*execution_mode
    fixed_center = 12,
    shoot_point_quadrant = 13,
    Refresh_quadrant = 14,
};

#endif // CONFIGURE_H