/**
 * @file rm_link.cpp
 * @author GCUROBOT-Visual-Group (GCUROBOT_WOLF@163.com)
 * @brief RM 2019 步兵视觉各入口链接部分源文件
 * @version 1.1
 * @date 2019-05-01
 * @copyright Copyright (c) 2019 GCU Robot Lab. All Rights Reserved.
 */
#include "rm_link.h"

RM_Vision_Init::RM_Vision_Init():capture(CAPTURE_DEFULT),cap(ISOPEN_INDUSTRY_CAPTURE){
}

void RM_Vision_Init::Run()
{
    /** Param Set **/
    if(cap.isindustryimgInput())
    {
        src_img = cvarrToMat(cap.iplImage,true);//这里只是进行指针转换，将IplImage转换成Mat类型
    }
    else
    {
        capture >> src_img;
    }
    resize(src_img,src_img,Size(640,480),INTER_NEAREST);
    SerialPort::RMreceiveData();
    int case_mode = mode_case;
    unsigned int receive_serial_data_No_ = SerialPort::RMdatabitProcessing(case_mode);
    cout<<"mode: "<<receive_serial_data_No_<<endl;
    /** Change Mode　**/
    switch (receive_serial_data_No_ = energy_agency_mode)
    {
    /**-Support Shooting mode-**/
    case support_shooting_mode:
    {
        imshow("src_img",src_img);
        armor.imageProcessing(src_img);
        armor.armorFitted();
        if(energy_refresh_count == 0){
            energy_refresh_count += 1;
            agency.chrysanthemum_paramRefresh();
        }
    }   /**-Support Shooting mode-**/
        break;
    /** Energy Agency Mode **/
    case energy_agency_mode:
    {
        imshow("src_img",src_img);
        agency.imageProcessing(src_img);
        agency.find_energy_agency();
        energy_refresh_count = 0;
    }   /** Energy Agency Mode **/
        break;
    /**-Empty mode-**/
    default:
        imshow("src_img",src_img);
        /**-Empty mode-**/
        break;
    }   /** Change Mode　**/
    cap.cameraReleasebuff();
}

bool RM_Vision_Init::is_exit()
{
    bool exit = false;
    int key = waitKey(1);
    if(char(key) == 27)
    {
        exit = true;
    }
    else
    {
        exit = false;
    }
    return exit;
}
