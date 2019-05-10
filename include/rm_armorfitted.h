/**
 * @file rm_armorfitted.h
 * @author GCUROBOT-Visual-Group (GCUROBOT_WOLF@163.com)
 * @brief RM 2019 步兵视觉装甲识别部分头文件
 * @version 2.0
 * @date 2019-05-01
 * @copyright Copyright (c) 2019 GCU Robot Lab. All Rights Reserved.
 */
#ifndef RM_ARMORFITTED_H
#define RM_ARMORFITTED_H
#include "configure.h"
#include "serialport.h"
#include "rm_kalmanfilter.h"

/** ------------------------------------------------------**
  @brief:　轮廓匹配部分参数
*/

/**
  @param: AREA_RATIO_MIN  面积之比的最小阈值
  @param: AREA_RATIO_MAX  面积之比的最大阈值
*/
#define AREA_RATIO_MIN 0.301
#define AREA_RATIO_MAX 1.010

/**
  @param: LOW_MULTIPLE  中点距离最小阈值的倍率参数
  @param: HIGH_MULTIPLE  中点距离最大阈值的倍率参数
*/
#define LOW_MULTIPLE 2.110
#define HIGH_MULTIPLE 28.888

/**
  @param: CENTER_SLPOE_MAX  中点间斜率的阈值
*/
#define CENTER_SLPOE_MAX 0.25

/**
 * @brief 死区椭圆尺寸
 */
#define ELLIPSE_SIZE 80.0
/** ------------------------------------------------------**/

/** ------------------------------------------------------**
  @brief　测距部分参数
  @note 单位像素宽/高(cm/pixel)
*/
#define UNIT_PIXEL 0.000001

/**
 * @brief 摄像头的焦距
 */
#define FOCAL_LENGTH 0.0012

/**
 * @brief 测距部分各数据的真实长度
 * @param BIG_ARMOR_TRUE_LENGTH  大装甲的真实宽度
 * @param LITTLE_ARMOR_TRUE_LENGTH  小装甲的真实宽度
 * @param LIGHT_TRUE_LENGTH  装甲上灯条的真实长度
*/
#define BIG_ARMOR_TRUE_LENGTH 0.230
#define LITTLE_ARMOR_TRUE_LENGTH 0.135
#define LIGHT_TRUE_LENGTH 0.055

/** ------------------------------------------------------**/

class RM_ArmorFitted
{
private:
    /** param initial **/
    Mat src_img;    //原图
    Mat gray_img;   //灰度图
    Mat hsv_img;    //hsv图
    Mat bin_img;    //二值图
    Mat dst_img;    //输出图
    const unsigned int THRESHOLD_VALUE_BLUE = 20; //蓝色装甲的阈值
    const unsigned int THRESHOLD_VALUE_RED = 20;  //红色装甲的阈值
    /** param initial **/
    RM_kalmanfilter kf;

    unsigned short int new_frame_none_count = 0;  //空帧计数器
    bool last_frame_has_data = false;   //上一帧数据判断位
    bool new_frame_has_data = false;    //当前帧数据判断位
    
    int x_buff = 320;
    int y_buff = 240;

    unsigned short int sendbuff_count = 0;
    const unsigned int buff_time = 10;
    bool buff_frame_data = false;
    
    /**
     * @brief 数据发送类型
     */
    enum send_type{
        data_none = 0,
        data_in_center = 1,
        data_normal = 2,
    };

public:

    /**
     * @brief 实例化函数
     */
    RM_ArmorFitted() :kf()
    {
        cout<<"Armor is ready"<<endl;
    }
    /**
     * @brief 图像预处理函数
     * @param frame 输入的原图
     * @authors: Rcxxx
     */
    void imageProcessing(Mat frame);

    /**
     * @brief 识别部分接口函数
     * @authors :Rcxxx
     *          :jiajia
     */
    void armorFitted();

private:
    /**
     * @brief 计算buff递减为中点
     * @param buff 
     * @param value 
     * @param flag 选择x轴或是y轴
     * @return int 返回的buff值
     */
    int bufferReturn(unsigned int buff,unsigned int value,unsigned int flag);

    /**
     * @brief 计算两点间的距离
     * @param p1 
     * @param p2 
     * @return float 
     */
    float centerDistance(Point p1,Point p2);

    /**
     * @brief 模糊测距函数
     * @param distance 图像中的长度
     * @param armor 大小装甲标记
     * @return float 返回的深度值　（单位: m ）
     * @note: @define:FOCAL_LENGTH　相机的焦距
     *        @define:UNIT_PIXEL　每个像素的单位长度
     *        @define:LIGHT_TRUE_LENGTH 灯条的实际长度
     *        @define:BIG_ARMOR_TRUE_LENGTH 大装甲板的实际长度
     *        @define:LITTLE_ARMOR_TRUE_LENGTH 小装甲板的实际长度
     *　      使用此函数时注意修改各个宏定义的参数　测距公式　(Ｆ/D = W'/W)
    * @authors: Rcxxx
    */
    float distancetoCamera(float distance,int armor);

    /**
     * @brief :无效区域判断函数
     * @param point 当前点坐标值
     * @return true :目标点处于无效区域
     * @return false :目标点处于有效区域
     * @authors: Rcxxx
     */
    bool iscentral_region(Point point);

    /**
     * @brief 
     * @param rect1 需要匹配的两个　minAreaRect
     * @param rect2 需要匹配的两个　minAreaRect
     * @return true 匹配成功
     * @return false 匹配失败
     * @authors :Rcxxx
     *          :jiajia
     */
    bool can_contour_match(const RotatedRect &rect1, const RotatedRect &rect2, int &armor, int &true_depth);
};

#endif // RM_ARMORFITTED_H
