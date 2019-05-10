/**
 * @file rm_link.h
 * @author GCUROBOT-Visual-Group (GCUROBOT_WOLF@163.com)
 * @brief RM 2019 步兵视觉各接口链接头文件
 * @version 1.1
 * @date 2019-05-01
 * @copyright Copyright (c) 2019 GCU Robot Lab. All Rights Reserved.
 */
#ifndef RM_LINK_H
#define RM_LINK_H

#include "configure.h"
#include "rm_videocapture.h"
#include "serialport.h"
#include "rm_armorfitted.h"
#include "rm_bigchrysanthemum.h"

class RM_Vision_Init
{
public:
    RM_Vision_Init();
    void Run();
    bool is_exit();

private:
    /** Camera Srart **/
    VideoCapture capture;
    RM_VideoCapture cap;
    /** Camera Srart **/

    /** param initial **/
    Mat src_img;
    /** param initial **/

    /** function initial **/
    RM_ArmorFitted armor;
    RM_BigChrysanthemum agency;
    /** function initial **/

    int energy_refresh_count = 0;
};

#endif // RM_LINK_H
