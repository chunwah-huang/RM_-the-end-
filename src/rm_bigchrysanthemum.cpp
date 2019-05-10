#include "rm_bigchrysanthemum.h"

using namespace std;

RM_BigChrysanthemum::RM_BigChrysanthemum() {
    cout << "bigChrysanthemum is ready" << endl;
}
/**
 * @brief 旋转矩形筛选函数
 * 
 * @param rect 
 * @return true 
 * @return false 
 */
bool RM_BigChrysanthemum::is_target_rect(RotatedRect & rect) {
    bool is_target = false;
    double rect_h, rect_w;

    if (rect.size.height > rect.size.width) {
        rect_h = rect.size.height;
        rect_w = rect.size.width;
    } else {
        rect_w = rect.size.height;
        rect_h = rect.size.width;
    }

    double area = rect_w * rect_h;
    double ratio = rect_w / rect_h;

    switch (find_condition) {
        case find_ellipse:

            if ((MIN_RECT_ELLIPSE < area && area < MAX_RECT_ELLIPSE) && (MIN_RECT_RATIO_ELLIPSE < ratio && ratio < MAX_RECT_RATIO_ELLIPSE)) {
                is_target = true;
                //cout << "ellipse_area:" << area << "   " << "ellipse_ratio:" << ratio << endl;
            }
            break;

        case find_circle:

            if ((MIN_RECT_CIRCLE < area && area < MAX_RECT_CIRCLE) && (MIN_RECT_RATIO_CIRCLE < ratio && ratio < MAX_RECT_RATIO_CIRCLE)) {
                is_target = true;
                //cout << "circle_area:" << area << "   " << "circle_ratio:" << ratio << endl;
            }
            break;

        case find_rect:

            if ((MIN_RECT_RECT < area && area < MAX_RECT_RECT) && (MIN_RECT_RATIO_RECT < ratio && ratio < MAX_RECT_RATIO_RECT)) {
                is_target = true;
                //cout << "rect_area:" << area << "   " << "rect_ratio:" << ratio << endl;
            }
            break;

        default:
            break;
    }
    return is_target;
}
/**
 * @brief 圆半径计算函数
 * 
 * @param p3 
 * @param p4 
 * @return double 
 */
double RM_BigChrysanthemum::dis_circle_rect(Point2f & p3, Point2f & p4) {
    double x_dis_2 = pow(p3.x - p4.x, 2);
    double y_dis_2 = pow(p3.y - p4.y, 2);
    double x_y_dis_2 = sqrt(x_dis_2 + y_dis_2);
    int average;

    if (dis_count.size() >= 4) {
        //cout << dis_count.size() << endl;
        vector < int > ::iterator iter = dis_count.begin();
        dis_count.erase(iter);
        average = (dis_count[1] + dis_count[2] + dis_count[3]) / dis_count.size();
        // cout << average << endl;
    }
    dis_count.push_back(x_y_dis_2);

    return average;
}
/**
 * @brief 判断流水灯与圆心距离函数
 * 
 * @param p1 
 * @param p2 
 * @return true 
 * @return false 
 */
bool RM_BigChrysanthemum::dis_circle_ellipse(Point2f & p1, Point2f & p2, double & radius) {

    bool is_circle = false;
    double x_dis = pow(p1.x - p2.x, 2);
    double y_dis = pow(p1.y - p2.y, 2);
    double x_y_dis = sqrt(x_dis + y_dis);
    double dis_circle_ellipse = radius * 0.8;

    if (x_y_dis < dis_circle_ellipse) {
        is_circle = true;
    }
    //cout << radius << "   " << dis_circle_ellipse << endl;
    return is_circle;
}
/**
 * @brief 预测函数
 * 
 * @param track_point 
 * @param last_track_point 
 * @param circle_point 
 * @param mode 
 * @return Point2f 
 */
Point2f RM_BigChrysanthemum::follow_Shoot(Point2f & track_point, Point2f & circle_point, int & mode) {

    Point2f predict_target;
    double total;
    double adjust = PREDICT_ADJUST;
    double theta = atan(double(track_point.y - circle_point.y) / (track_point.x - circle_point.x)) * CV_PI / 180;

    if (mode == 4) {
        total = theta + adjust;
    } else {
        total = theta - adjust;
    }

    predict_target.x = (track_point.x - circle_point.x) * cos(total) - (track_point.y - circle_point.y) * sin(total) + circle_point.x;
    predict_target.y = (track_point.x - circle_point.x) * sin(total) + (track_point.y - circle_point.y) * cos(total) + circle_point.y;

    return predict_target;
}

void RM_BigChrysanthemum::chrysanthemum_paramRefresh() {
    RotatedRect last_take_rect;
    dis_count.clear();

    ans_count = 0;
    good_count = 0;
    poor_count = 0;

    last_rect_circle = 0;
    cout << "in again!!!!!!!!!!" << endl;
}
/*-------------------------------------------------------------Main Code----------------------------------------------------------*/
/**
 * @brief 预处理函数
 * 
 * @param frame 
 */
void RM_BigChrysanthemum::imageProcessing(Mat frame) {

    resize(frame, frame, Size(640, 480));
    frame.copyTo(src_img);
    blur(frame, frame, Size(7, 7));
    //imshow("blur", frame);
    cvtColor(frame, hsv_img, COLOR_BGR2HSV);

    if (ARMOR_COLOR == 1) { //己方颜色
        inRange(hsv_img, Scalar(156, 50, 20), Scalar(180, 255, 255), mix_img_1); //red
        inRange(hsv_img, Scalar(0, 50, 20), Scalar(10, 255, 255), mix_img_2);
        bin_img_1 = mix_img_1 + mix_img_2;
    } else {
        //inRange(hsv_img, Scalar(80, 210, 130), Scalar(124, 255, 255), bin_img_1); //blue
        inRange(hsv_img, Scalar(80, 220, 60), Scalar(124, 255, 255), bin_img_1); //blue    rm_use
    }

    element_1 = getStructuringElement(MORPH_RECT, Size(3, 3));
    morphologyEx(bin_img_1, bin_img_1, MORPH_OPEN, element_1);
    //imshow("hsv_img", bin_img_1);

    Point2f seed_point(10, 10);
    bool is_black = false;
    for (int j = seed_point.y; j < frame.rows; ++j) {
        uchar * dst = bin_img_1.ptr < uchar > (j);
        for (int i = seed_point.x; i < frame.cols; ++i) {

            int value = dst[i];
            //cout << "value:" << value << endl;/*0:黑色像素点   255:白色像素点*/
            if (value == 0) {
                seed_point.x = i;
                seed_point.y = j;

                i = frame.cols;
                j = frame.rows;

                is_black = true;
            }
        }
    }

    if (is_black == true) {

        bin_img_1.copyTo(bin_img_2);

        floodFill(bin_img_2, seed_point, Scalar(255)); //漫水填充
        threshold(bin_img_2, bin_img_2, 254, 255, CV_THRESH_BINARY_INV); //漫水填充和阈值反操作后尽可能消除噪点影响
        //imshow("flood", bin_img_2);
        element_2 = getStructuringElement(MORPH_RECT, Size(3, 3));
        morphologyEx(bin_img_2, bin_img_2, MORPH_OPEN, element_2); //除孤立小白噪点
        //imshow("element_2", bin_img_2);
        element_3 = getStructuringElement(MORPH_RECT, Size(13, 13));
        dilate(bin_img_2, bin_img_2, element_3); //膨胀粘合叶片和装甲板白块
        //imshow("element_3", bin_img_2);
        element_4 = getStructuringElement(MORPH_RECT, Size(3, 3));
        morphologyEx(bin_img_2, bin_img_2, MORPH_CLOSE, element_4); //除孤立小黑噪点
        //imshow("element_4", bin_img_2);
    }
}
/**
 * @brief 主要逻辑函数
 * 
 */
void RM_BigChrysanthemum::find_energy_agency() {

    Point2f send_point = Point(320, 240);

    RotatedRect take_rect;
    take_rect.center.x = 320;
    take_rect.center.y = 240;

    RotatedRect take_ellipse;
    take_ellipse.center.x = 320;
    take_ellipse.center.y = 240;

    RotatedRect take_circle;
    take_circle.center.x = 320;
    take_circle.center.y = 240;

    bool is_rect = false; //识别判断
    bool is_ellipse = false; //流水灯判断
    bool is_circle = false; //圆心判断

    vector < vector < Point >> contours_1;
    vector < vector < Point >> contours_2;
    vector < Vec4i > hierarchy_1;
    vector < Vec4i > hierarchy_2;

    findContours(bin_img_2, contours_1, hierarchy_1, CV_RETR_EXTERNAL, CHAIN_APPROX_NONE, Point(0, 0));
    findContours(bin_img_1, contours_2, hierarchy_2, CV_RETR_EXTERNAL, CHAIN_APPROX_NONE, Point(0, 0));

    RotatedRect surround_rect;
    RotatedRect surround_ellipse_circle;
    Rect RECT_1;

    for (int i1 = 0; i1 < (int) contours_1.size(); ++i1) {

        surround_rect = minAreaRect(contours_1[i1]);
        find_condition = find_rect;

        if (is_target_rect(surround_rect)) {
            take_rect = surround_rect;
            is_rect = true;

            RECT_1 = surround_rect.boundingRect();
            circle(src_img, surround_rect.center, 4, Scalar(255, 0, 0), -1, 8); //识别点
            rectangle(src_img, RECT_1, Scalar(0, 0, 255), 2);
        }
    }

    double contour_area;
    double complexity;
    for (int i2 = 0; i2 < (int) contours_2.size(); ++i2) {

        contour_area = contourArea(contours_2[i2]); //轮廓面积
        complexity = pow(arcLength(contours_2[i2], true), 2) / contour_area; //复杂度
        //cout << "all_contour_area:" << contour_area << endl;
        if (contour_area < MAX_CONTOUR_ELLIPSE) { //风车流水灯面积上限
            //cout << "ellipse_contour_area:" << contour_area << endl;
            surround_ellipse_circle = minAreaRect(contours_2[i2]);

            if ((MIN_CONTOUR_ELLIPSE < contour_area) && (MIN_COMPLEXITY_ELLIPSE < complexity && complexity < MAX_COMPLEXITY_ELLIPSE)) { //风车流水灯轮廓面积复杂度筛选//TODO:
                //cout << "ellipse_contour_area:" <<contour_area << "    ellipse_complexity:" << complexity << endl;
                find_condition = find_ellipse;
                if (is_target_rect(surround_ellipse_circle)) { //风车流水灯比例面积筛选
                    take_ellipse = surround_ellipse_circle;
                    is_ellipse = true;
                    ellipse(src_img, surround_ellipse_circle, Scalar(0, 0, 255), 1, 8);
                    circle(src_img, surround_ellipse_circle.center, 3, Scalar(0, 255, 0), -1, 8);
                }
            }

            if ((MIN_CONTOUR_CIRCLE < contour_area && contour_area < MAX_CONTOUR_CIRCLE) && (MIN_COMPLEXITY_CIRCLE < complexity && complexity < MAX_COMPLEXITY_CIRCLE)) { //风车圆心轮廓面积复杂度筛选
                //cout << "circle_contour_area:" <<contour_area << "    circle_complexity:" << complexity << endl;
                find_condition = find_circle;
                if (is_target_rect(surround_ellipse_circle)) { //风车圆心比例面积筛选
                    take_circle = surround_ellipse_circle;
                    is_circle = true; //圆心通过筛选
                    ellipse(src_img, take_circle, Scalar(0, 255, 0), 2, 8);
                }
            }

        }
    }

    int rect_circlr = fastAtan2(take_rect.center.y - take_circle.center.y, take_rect.center.x - take_circle.center.x); //圆心和识别装甲板的向量
    int ellipse_circle = fastAtan2(take_ellipse.center.y - take_circle.center.y, take_ellipse.center.x - take_circle.center.x); //圆心和流水灯的向量
    int compare_angle = abs(rect_circlr - ellipse_circle); //前两者向量差计算

    if (last_take_rect.center.x > 0) { //第二帧才进入!!(帧差)
        //cout << "count:" << all_count << endl;
        bool is_success_1 = false;
        bool is_success_2 = false;
        bool is_return_center = false;

        if (is_circle && is_rect && is_ellipse) { //圆心和流水灯和识别点匹配一起才进入

            int ans = (take_circle.center.x - take_rect.center.x) * (last_take_rect.center.y - take_rect.center.y) - (take_circle.center.y - take_rect.center.y) * (last_take_rect.center.x - take_rect.center.x);
            int jump = rect_circlr - last_rect_circle; //跳动情况
            //cout << "jump:" << jump << endl;
            if ((abs(jump) < JUMP_VALUE) && (ans != 0) && (ans_count <= AIM_COUNT)) { //排除识别跳动状况并且识别转动情况(仅前固定帧数进入)
                if (ans_count < AIM_COUNT) { //大神符状态打开定旋转方向的帧数
                    if (ans > 0) { //顺时针数值叠加
                        good_count++;
                    } else { //逆时针数值叠加
                        poor_count++;
                    }
                    ans_count++;
                    //cout << "good_or_poor" << endl;
                } else if (ans_count == AIM_COUNT) { //叠加完成时，对比叠加量，固定往后的旋转模型状态
                    if (good_count > poor_count) {
                        find_direction = good; //顺时针
                    } else {
                        find_direction = poor; //逆时针
                    }
                    ans_count++;
                    //cout << good_count << "  " << poor_count << "   " << find_direction << endl;
                }
            }
            is_success_1 = true; //3个识别目标匹配成功
        } else if ((is_circle == 1) && (is_rect == 0)) { //装甲板识别丢失，但识别圆心
            if ((take_circle.center.x < 120) || (take_circle.center.x > 640 - 120)) {
                is_return_center = true;
            } else if ((take_circle.center.y < 120) || (take_circle.center.y > 480 - 120)) {
                is_return_center = true;
            }
        }

        if (is_success_1) {
            double dis_radius = dis_circle_rect(take_rect.center, take_circle.center);
            if ((dis_circle_ellipse(take_ellipse.center, take_circle.center, dis_radius)) && (compare_angle < COMPARE_VECTOR)) {
                is_success_2 = true; //3个识别目标匹配成功并且几何关系成立
            }
        }

        if (is_success_2) { //目标筛选完成
            int choose;
            switch (find_direction) {
                case good:
                    choose = good;
                    send_point = follow_Shoot(take_rect.center, take_circle.center, choose);
                    break;

                case poor:
                    choose = poor;
                    send_point = follow_Shoot(take_rect.center, take_circle.center, choose);
                    break;

                default:
                    break;
            }

            if ((SERIAL_IS_OPEN == 1) && (ans_count >= AIM_COUNT)) { //固定帧数跑完之后才进入
                if (send_point.x < 0 || send_point.x > 640) {
                    if (send_point.x < 0) {
                        send_point.x = 0;
                    } else {
                        send_point.x = 640;
                    }
                    cout << "fuck_x!!!!!!!!!!!!!!!!!!!!" << endl;
                }
                if (send_point.y < 0 || send_point.y > 480) {
                    if (send_point.y < 0) {
                        send_point.y = 0;
                    } else {
                        send_point.y = 480;
                    }
                    cout << "fuck_y!!!!!!!!!!!!!!!!!!!!" << endl;
                }

                SerialPort::RMserialWrite(send_point.x, send_point.y, 700); //识别目标1-1
                circle(src_img, Point(send_point.x, send_point.y), 3, Scalar(0, 0, 255), -1, 8);
                cout << "1-1，send_point:" << send_point << endl;
            } else if ((SERIAL_IS_OPEN == 1) && (ans_count < AIM_COUNT)) { //启动模式开始马上标准圆心1-2
                SerialPort::RMserialWrite(take_circle.center.x, take_circle.center.y, 700);
                circle(src_img, Point(take_circle.center.x, take_circle.center.y), 8, Scalar(255, 0, 255), -1, 8);
                cout << "1-2,aiming the center!!!:" << take_circle.center << endl;
            }
        } else if ((SERIAL_IS_OPEN == 1) && (is_return_center == 0)) { //识别丢失，发送坐标[320, 240]，2
            SerialPort::RMserialWrite(320, 240, 700);
            circle(src_img, Point(320, 240), 3, Scalar(0, 0, 255), -1, 8);
            cout << "2,lost!!!:[320, 240]" << endl;
        } else if ((SERIAL_IS_OPEN == 1) && (is_return_center == 1)) { //识别丢失，且识别圆心超出限定界面，发送圆心坐,3
            SerialPort::RMserialWrite(take_circle.center.x, take_circle.center.y, 700);
            circle(src_img, Point(take_circle.center.x, take_circle.center.y), 12, Scalar(0, 0, 255), -1, 8);
            cout << "3,return center point!!!:" << take_circle.center << endl;
        }
    } else {
        SerialPort::RMserialWrite(320, 240, 0);
        cout << "4,else!!!:[320, 240]" << endl;
    }
    rectangle(src_img, Point(120, 120), Point(640 - 120, 480 - 120), Scalar(0, 125, 255), 1, 8);
    circle(src_img, Point(320, 240), 30, Scalar(0, 125, 255), 2, 8);
    imshow("src_img", src_img);
    last_take_rect = take_rect;
    last_rect_circle = rect_circlr;
}