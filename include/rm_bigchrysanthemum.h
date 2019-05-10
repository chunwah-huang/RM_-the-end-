/**
 * @file rm_bigchrysanthemum.h
 * @author ZhenHua (961716598@qq.com)
 * @brief 

 * @version 0.99
 * @date 2019-04-27
 * @copyright Copyright (c) 2019 GCU Robot Lab. All Rights Reserved.
*/
#ifndef RM_BIGCHRYSANTHEMUM_H
#define RM_BIGCHRYSANTHEMUM_H

#include "configure.h"

#include "serialport.h"

// /**
//  * @brief 旋转矩形筛选参数
//  * 
//  */
// #define MAX_RECT_ELLIPSE 6100/* 流水灯旋转矩形面积 */
// #define MIN_RECT_ELLIPSE 3200
// #define MAX_RECT_RATIO_ELLIPSE 0.69/* 流水灯旋转矩形宽高比 */
// #define MIN_RECT_RATIO_ELLIPSE 0.40

// #define MAX_RECT_CIRCLE 380/* 圆心旋转矩形面积 */
// #define MIN_RECT_CIRCLE 90
// #define MAX_RECT_RATIO_CIRCLE 1.30/* 圆心旋转矩形宽高比 */
// #define MIN_RECT_RATIO_CIRCLE 0.77

// #define MAX_RECT_RECT 2500/* 识别待击打装甲板旋转矩形面积 */
// #define MIN_RECT_RECT 780
// #define MAX_RECT_RATIO_RECT 1.20/* 识别待击打装甲板旋转矩形宽高比 */
// #define MIN_RECT_RATIO_RECT 0.50

// /**
//  * @brief 轮廓包围筛选参数
//  * 
//  */
// #define MAX_CONTOUR_ELLIPSE 3500/* 流水灯轮廓包围面积 */
// #define MIN_CONTOUR_ELLIPSE 800
// #define MAX_COMPLEXITY_ELLIPSE 65/* 流水灯复杂度 */
// #define MIN_COMPLEXITY_ELLIPSE 25

// #define MAX_CONTOUR_CIRCLE 350/* 圆心轮廓包围面积 */
// #define MIN_CONTOUR_CIRCLE 60
// #define MAX_COMPLEXITY_CIRCLE 28/* 圆心复杂度 */
// #define MIN_COMPLEXITY_CIRCLE 8

// /**
//  * @brief 逻辑部分限制参数
//  * 
//  */
// #define AIM_COUNT 100//判断旋转方向同事瞄准圆心

// #define JUMP_VALUE 30//跳动数值判断

// #define COMPARE_VECTOR 10//对比向量差

// #define DIS_CIRCLE_ELLIPSE 130//流水灯中心到圆心距离

// #define PREDICT_ADJUST 0.36//弧度


/**
 * @brief 旋转矩形筛选参数
 * 
 */
#define MAX_RECT_ELLIPSE 6100/* 流水灯旋转矩形面积 */
#define MIN_RECT_ELLIPSE 3200
#define MAX_RECT_RATIO_ELLIPSE 0.69/* 流水灯旋转矩形宽高比 */
#define MIN_RECT_RATIO_ELLIPSE 0.40

#define MAX_RECT_CIRCLE 380/* 圆心旋转矩形面积 */
#define MIN_RECT_CIRCLE 90
#define MAX_RECT_RATIO_CIRCLE 1.30/* 圆心旋转矩形宽高比 */
#define MIN_RECT_RATIO_CIRCLE 0.77

#define MAX_RECT_RECT 2000/* 识别待击打装甲板旋转矩形面积 */
#define MIN_RECT_RECT 680
#define MAX_RECT_RATIO_RECT 1.00/* 识别待击打装甲板旋转矩形宽高比 */
#define MIN_RECT_RATIO_RECT 0.58

/**
 * @brief 轮廓包围筛选参数
 * 
 */
#define MAX_CONTOUR_ELLIPSE 2500/* 流水灯轮廓包围面积 */
#define MIN_CONTOUR_ELLIPSE 1400
#define MAX_COMPLEXITY_ELLIPSE 46/* 流水灯复杂度 */
#define MIN_COMPLEXITY_ELLIPSE 30

#define MAX_CONTOUR_CIRCLE 350/* 圆心轮廓包围面积 */
#define MIN_CONTOUR_CIRCLE 60
#define MAX_COMPLEXITY_CIRCLE 24/* 圆心复杂度 */
#define MIN_COMPLEXITY_CIRCLE 8

/**
 * @brief 逻辑部分限制参数
 * 
 */
#define AIM_COUNT 50//判断旋转方向同事瞄准圆心

#define JUMP_VALUE 30//跳动数值判断

#define COMPARE_VECTOR 10//对比向量差

#define DIS_CIRCLE_ELLIPSE 130//流水灯中心到圆心距离

#define PREDICT_ADJUST 0.36//弧度

/**
 * @brief 大神符函数以及参数
 * 
 */
class RM_BigChrysanthemum {
    public:
    RM_BigChrysanthemum();
    void imageProcessing(Mat frame);
    void find_energy_agency();
    void chrysanthemum_paramRefresh();

    private:
    bool is_target_rect(RotatedRect & rect);
    double dis_circle_rect(Point2f & p3, Point2f & p4);
    bool dis_circle_ellipse(Point2f & p1, Point2f & p2, double & radius);
    Point2f follow_Shoot(Point2f & track_point, Point2f & circle_point, int & mode);
   
    int find_condition = 0;
    enum find_condition {
        find_ellipse = 1,
        find_circle = 2,
        find_rect = 3,
    };

    int find_direction = 0;
    enum find_direction {
        good = 4,
        poor = 5,
    };

    RotatedRect last_take_rect;
    vector < int > dis_count;

    Mat src_img;
    Mat hsv_img;
    Mat mix_img_1;
    Mat mix_img_2;
    Mat bin_img_1;
    Mat bin_img_2;

    Mat element_1;
    Mat element_2;
    Mat element_3;
    Mat element_4;

    int ans_count = 0;
    int good_count = 0;
    int poor_count = 0;

    int last_rect_circle = 0;
};
#endif // RM_BIGCHRYSANTHEMUM_H