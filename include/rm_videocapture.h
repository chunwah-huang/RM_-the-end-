/**
 * @file rm_videocapture.h
 * @author GCUROBOT-Visual-Group (GCUROBOT_WOLF@163.com)
 * @brief RM 2019 步兵视觉相机配置头文件
 * @version 1.1
 * @date 2019-05-01
 * @copyright Copyright (c) 2019 GCU Robot Lab. All Rights Reserved.
 */
#ifndef RM_VIDEOCAPTURE_H
#define RM_VIDEOCAPTURE_H

#include "configure.h"

class RM_VideoCapture
{
public:
    unsigned char *g_pRgbBuffer; //处理后数据缓存区

    int                     iCameraCounts = 1;
    int                     iStatus=-1;
    tSdkCameraDevInfo       tCameraEnumList;
    int                     hCamera;
    tSdkCameraCapbility     tCapability;      //设备描述信息
    tSdkFrameHead           sFrameInfo;
    BYTE*			        pbyBuffer;
    IplImage                *iplImage = NULL;
    int                     channel=3;
    BOOL                    AEstate=FALSE;

    bool                    iscamera0_open = false;

public:
    RM_VideoCapture(int cameramode);

    bool isindustryimgInput();
    void cameraReleasebuff();
    int cameraSet();

public:
    ~RM_VideoCapture();
};

#endif // RM_VIDEOCAPTURE_H
