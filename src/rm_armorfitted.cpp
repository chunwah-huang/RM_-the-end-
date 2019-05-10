/**
 * @file rm_armorfitted.cpp
 * @author GCUROBOT-Visual-Group (GCUROBOT_WOLF@163.com)
 * @brief RM 2019 步兵视觉装甲识别部分源文件
 * @version 2.0
 * @date 2019-05-01
 * @copyright Copyright (c) 2019 GCU Robot Lab. All Rights Reserved.
 */
#include "rm_armorfitted.h"

int RM_ArmorFitted::bufferReturn(unsigned int buff, unsigned int value, unsigned int flag) {
    unsigned int back_value;
    unsigned int max_value;
    if (flag == 0) {
        max_value = src_img.cols / 2;
    } else {
        max_value = src_img.rows / 2;
    }

    if (buff < max_value) {
        back_value = buff + value / (buff_time - sendbuff_count);
    } else {
        back_value = buff - value / (buff_time - sendbuff_count);
    }

    return back_value;
}

float RM_ArmorFitted::centerDistance(Point p1, Point p2) {
    float D = sqrt(pow(p1.x - p2.x, 2) + pow(p1.y - p2.y, 2));
    return D;
}

float RM_ArmorFitted::distancetoCamera(float distance,int armor) {
    float true_distance;
    float piex_dis;
    if(armor == 3){
        //计算成像距离
        piex_dis = (distance-10) * UNIT_PIXEL;
        true_distance = LIGHT_TRUE_LENGTH;
    }else{
        //计算成像距离
        piex_dis = (distance) * UNIT_PIXEL;
        if (armor == big_armor){
            true_distance = BIG_ARMOR_TRUE_LENGTH;
        }else{
            true_distance = LITTLE_ARMOR_TRUE_LENGTH;
        }
    }
    //计算距离
    float return_distance = true_distance * FOCAL_LENGTH / piex_dis;
    return return_distance;
}

bool RM_ArmorFitted::iscentral_region(Point point) {
    bool iscentral_region_point;
    Point2f center = (Point2f) Point(src_img.cols / 2, src_img.rows / 2);
    float ellipse_size = ELLIPSE_SIZE;
    Point2f left_focus = Point2f(center.x - (ellipse_size * 0.5 * 0.8), center.y);
    Point2f right_focus = Point2f(center.x + (ellipse_size * 0.5 * 0.8), center.y);
    float PF1 = centerDistance(point, left_focus);
    float PF2 = centerDistance(point, right_focus);
    if ((PF1 + PF2) <= ellipse_size) {
        iscentral_region_point = true;
    } else {
        iscentral_region_point = false;
    }
    ellipse(dst_img, point, Size(ellipse_size * 0.5, sqrt(pow(ellipse_size * 0.5, 2) - pow(ellipse_size * 0.5 * 0.8, 2))), 0, 0, 360, Scalar(255, 129, 0), 2, 8);
    ellipse(dst_img, center, Size(ellipse_size * 0.5, sqrt(pow(ellipse_size * 0.5, 2) - pow(ellipse_size * 0.5 * 0.8, 2))), 0, 0, 360, Scalar(255, 129, 20), 2, 8);
    return iscentral_region_point;
}

bool RM_ArmorFitted::can_contour_match(const RotatedRect & rect1,const RotatedRect & rect2, int &armor, int &true_depth) {
    bool iscontour_matching = false;
    float distance_temp = centerDistance(rect1.center, rect2.center);
    float depth_armor = 0;
    float depth_light = 0;

    /** get area ratio **/
    float area_rect1 = (float)rect1.size.width*(float)rect1.size.height;
    float area_rect2 = (float)rect2.size.width*(float)rect2.size.height;
    float area_ratio = area_rect1>area_rect2 ? (area_rect2/area_rect1):(area_rect1/area_rect2);

    bool is_little_armor_area_ratio = (AREA_RATIO_MIN < area_ratio && area_ratio < AREA_RATIO_MAX);
    bool is_big_armor_area_ratio = (AREA_RATIO_MIN < area_ratio && area_ratio < AREA_RATIO_MAX);

    //float w1 = (rect1.size.height > rect1.size.width ? rect1.size.width:rect1.size.height);
    float h1 = (rect1.size.height > rect1.size.width ? rect1.size.height:rect1.size.width);
    //float w2 = (rect2.size.height > rect2.size.width ? rect2.size.width:rect2.size.height);
    float h2 = (rect2.size.height > rect2.size.width ? rect2.size.height:rect2.size.width);
    /** exchange width and height **/
    /** get center **/
    float x1 = rect1.center.x;
    float y1 = rect1.center.y;
    float x2 = rect2.center.x;
    float y2 = rect2.center.y;
    /** get center **/
    float center_slope = fabs((y1-y2)/(x1-x2));

    bool is_target_slope = (center_slope < CENTER_SLPOE_MAX || center_slope == 0);

    float center_distance = sqrt(pow(x1-x2,2)+pow(y1-y2,2));
    float max_h = max(h1,h2);

    bool is_little_armor_dis_maxH_ratio = (1.f < center_distance/max_h && center_distance/max_h < 3.8f);
    bool is_big_armor_dis_maxH_ratio = (4.4f<center_distance/max_h && center_distance/max_h< 5.8f);

    if/* Little Armor */(is_little_armor_area_ratio
            && is_target_slope
            && is_little_armor_dis_maxH_ratio
            && (armor == little_armor))
    {
        iscontour_matching = true;
        depth_armor = distancetoCamera(distance_temp,armor);
        depth_light = distancetoCamera(distance_temp,3);
        cout<<"d_1:x "<<depth_armor << "  d2_y: "<<depth_light<<endl;
        true_depth = int((depth_armor + depth_light) * 0.5 * 100);
    }
    else if/* Big Armor*/(is_big_armor_area_ratio
            && is_target_slope
            && is_big_armor_dis_maxH_ratio
            && (armor == big_armor))
    {
        iscontour_matching = true;
        depth_armor = distancetoCamera(distance_temp,armor);
        depth_light = distancetoCamera(distance_temp,3);
        true_depth = int((depth_armor + depth_light) * 0.5 * 100);
    }
    else
    {
        iscontour_matching = false;
    }
    return iscontour_matching;
}

void RM_ArmorFitted::imageProcessing(Mat frame) {
    src_img = frame;
    int threshold_Value;
    if (ARMOR_COLOR == 0) {
        threshold_Value = THRESHOLD_VALUE_RED;
    } else {
        threshold_Value = THRESHOLD_VALUE_BLUE;
    }
    //resize(src_img,src_img,Size(640,480),INTER_NEAREST);
    src_img.copyTo(dst_img);
    /** -------------Image Segmentation	---------------**/
    cvtColor(src_img,hsv_img,COLOR_BGR2HSV);
    cvtColor(src_img,gray_img,COLOR_BGR2GRAY);
    Mat bin_img_1 = Mat::zeros(480,640,CV_8UC1);
    Mat bin_img_2 = Mat::zeros(480,640,CV_8UC1);
    threshold(gray_img, bin_img_2, threshold_Value, 255, THRESH_BINARY);

    for(int x = 0;x < hsv_img.cols; ++x)
    {
        for(int y = 0;y < hsv_img.rows; ++y)
        {
            float H = hsv_img.at<Vec3b> (y,x)[0];
            float S = hsv_img.at<Vec3b> (y,x)[1];
            float V = hsv_img.at<Vec3b> (y,x)[2];
            //red
            if(ARMOR_COLOR == 0)
            {
                if(((H>=145 && H<180)||(H>=0 && H<=13)) && (S >= 135 && S <= 255) && (V > 148 && V <= 255)){
                    bin_img_1.at<uchar>(y,x) = 255;
                }else{
                    bin_img_1.at<uchar>(y,x) = 0;
                }
            }
            //blue
            else
            {
                if((H>=75 && H<=130) && (S >= 195 && S <= 255) && (V >= 185 && V <= 255)){
                    bin_img_1.at<uchar>(y,x) = 255;
                }else{
                    bin_img_1.at<uchar>(y,x) = 0;
                }
            }
        }
    }
    Mat element = getStructuringElement(MORPH_RECT, Size(5, 5));
    dilate(bin_img_1, bin_img_1, element);
    bin_img = Mat::zeros(480,640,CV_8UC1);
    for(int x = 0;x < bin_img_1.cols; ++x)
    {
        for(int y = 0;y < bin_img_1.rows; ++y)
        {
            if((bin_img_1.at<uchar>(y,x) == bin_img_2.at<uchar>(y,x)) && (bin_img_1.at<uchar>(y,x) == 255) && ( bin_img_2.at<uchar>(y,x) == 255)){
                bin_img.at<uchar>(y,x) = 255;
            }else{
                bin_img.at<uchar>(y,x) = 0;
            }
        }
    }
    element = getStructuringElement(MORPH_RECT, Size(5, 7));
    dilate(bin_img, bin_img, element);
    //Canny(bin_img,bin_img,120,240);
    /** -------------Image Segmentation	---------------**/
}

void RM_ArmorFitted::armorFitted() {
    int x_widht = src_img.cols * 0.5; //声明数据量
    int y_height = src_img.rows * 0.5;
    float test_fuzzy_distance = 0.f;
    unsigned int Recoginition_success_flag = 0; //初始化发送类型标记 0表示发送丢失判断数据
    new_frame_has_data = false; //新帧数据标记刷新

    vector < vector < Point > > contours;
    vector < Rect > boundRect;
    vector < RotatedRect > rotateRect;
    vector < Vec4i > hierarchy;
    vector < Point2f > midPoint(2);
    vector < vector < Point2f > > midPoint_pair;
    vector < float > fuzzy_distance;
    findContours(bin_img, contours, hierarchy, RETR_EXTERNAL, CHAIN_APPROX_NONE, Point(0, 0));
    for (int i = 0; i < (int) contours.size(); ++i) {
        if (contours.size() <= 1)
            break;
        Rect B_rect_i = boundingRect(contours[i]);
        RotatedRect R_rect_i = minAreaRect(contours[i]);
        if (B_rect_i.height >= B_rect_i.width) {
            boundRect.push_back(B_rect_i);
            rotateRect.push_back(R_rect_i);
        }
    }

    float distance_max = 0.f;
    for (int k1 = 0; k1 < (int) rotateRect.size(); ++k1) {
        if (rotateRect.size() <= 1)
            break;
        for (int k2 = k1 + 1; k2 < (int) rotateRect.size(); ++k2) {
            int true_depth = 0;
            int case_armor_size = armor_size_case;
            int armor = SerialPort::RMdatabitProcessing(case_armor_size);
            cout<<"  armor: "<<armor<<endl;
            if (can_contour_match(rotateRect[k1], rotateRect[k2],armor,true_depth)) {
                float distance_temp = centerDistance(rotateRect[k1].center,rotateRect[k2].center);
                /** change max distance **/
                if (distance_temp >= distance_max) {
                    distance_max = distance_temp;
                }
                /** change max distance **/
                rectangle(dst_img, boundRect[k1].tl(), boundRect[k1].br(), Scalar(0, 255, 255), 2, 8, 0);
                rectangle(dst_img, boundRect[k2].tl(), boundRect[k2].br(), Scalar(0, 255, 255), 2, 8, 0);
                midPoint[0] = rotateRect[k1].center;
                midPoint[1] = rotateRect[k2].center;
                midPoint_pair.push_back(midPoint);
                fuzzy_distance.push_back(true_depth);
            }
        }
    }

    for (int k3 = 0; k3 < (int) midPoint_pair.size(); ++k3) {
        float midPoint_distance = centerDistance(midPoint_pair[k3][0], midPoint_pair[k3][1]);
        float midPoint_slope = fabs((midPoint_pair[k3][0].y - midPoint_pair[k3][1].y) / (midPoint_pair[k3][0].x - midPoint_pair[k3][1].x));
        if (midPoint_slope < CENTER_SLPOE_MAX) {
            if (midPoint_distance >= distance_max) {
                Point2f true_center = Point2f((midPoint_pair[k3][0].x + midPoint_pair[k3][1].x) / 2, (midPoint_pair[k3][0].y + midPoint_pair[k3][1].y) / 2);
                rectangle(dst_img,
                    Point(true_center.x - int(midPoint_distance * 0.40), true_center.y - int(midPoint_distance * 0.18)),
                    Point(true_center.x + int(midPoint_distance * 0.40), true_center.y + int(midPoint_distance * 0.18)),
                    Scalar(255, 0, 255), 2, 8, 0);
                circle(dst_img, true_center, 5, Scalar(255, 0, 255), 2, 8, 0);

                // int16_t gyroscope_data = 0;
                // bool is_receive_success = false;
                // SerialPort::RM_receiveData_armor(gyroscope_data, is_receive_success);
                // if (is_receive_success == true) {
                //     cout << "gyroscope_data  " << gyroscope_data << endl;
                // }
                // kf.update_feedback_Param(gyroscope_data);
                // Point2f new_center = kf.point_Predict(((1e-2) + 0.006666666f), true_center);
                // circle(dst_img, new_center, 5, Scalar(120, 120, 150), 2, 8, 0);
                // rectangle(dst_img,
                //     Point(new_center.x - int(midPoint_distance * 0.47), new_center.y - int(midPoint_distance * 0.2)),
                //     Point(new_center.x + int(midPoint_distance * 0.47), new_center.y + int(midPoint_distance * 0.2)),
                //     Scalar(255, 0, 255), 2, 8, 0);

                x_buff = true_center.x; //更新缓存帧
                y_buff = true_center.y; //更新缓存帧

                x_widht = true_center.x; //更新发送数据
                y_height = true_center.y; //更新发送数据

                test_fuzzy_distance = fuzzy_distance[k3];
                bool is_it_in_service_area = false;//iscentral_region(true_center, test_fuzzy_distance);
                if (is_it_in_service_area) {
                    Recoginition_success_flag = data_in_center; //重置标记　1 表示位于中心区
                } else {
                    Recoginition_success_flag = data_normal; //重置标记　2 表示正常发送数据
                    // x_widht = new_center.x; //更新发送数据
                    // y_height = new_center.y; //更新发送数据
                }
                buff_frame_data = true; //上一帧数据标记　1
                new_frame_none_count = 0; //重置空帧计数器
                break;
            }
        }
    }

    if (SERIAL_IS_OPEN == 1) {
        /*获取发送类型标记,选择串口发送类型*/
        switch (Recoginition_success_flag) {
            case data_none:
                {
                    /** 未识别到目标时 **/
                    /*　判断上一次是否有数据成功发送　有则使用缓存,无则发送停止符*/
                    if (buff_frame_data == true) {
                        /** 没有检测到装甲板，上一帧有数据**/
                        if (sendbuff_count < buff_time) {
                            /** 检测到装甲板**/
                            int buff_reduce_value_x = abs(src_img.cols / 2 - x_buff);
                            int buff_reduce_value_y = abs(src_img.rows / 2 - y_buff);
                            unsigned int send_buff_x = bufferReturn(x_buff, buff_reduce_value_x, 0);
                            unsigned int send_buff_y = bufferReturn(y_buff, buff_reduce_value_y, 1);
                            SerialPort::RMserialWrite(send_buff_x, send_buff_y - 85, test_fuzzy_distance);
                            sendbuff_count += 1;
                        } else {
                            SerialPort::RMserialWrite(x_widht, y_height, test_fuzzy_distance);
                        }
                    } else {
                        /** 没有检测到装甲板并且上一帧无数据 **/
                        SerialPort::RMserialWrite(x_widht, y_height, test_fuzzy_distance);
                    }
                }
                break;
            case data_in_center:
                {
                    sendbuff_count = 0; //成功发送数据时将缓存计数器置零
                    SerialPort::RMserialWrite(x_widht, y_height, test_fuzzy_distance);
                }
                break;
            case data_normal:
                {
                    sendbuff_count = 0;
                    SerialPort::RMserialWrite(x_widht, y_height - 85, test_fuzzy_distance);
                }
                break;
            default:
                break;
        }
    }

    imshow("bin_img", bin_img);
    imshow("dst_img", dst_img);
}