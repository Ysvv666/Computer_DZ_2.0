// ROS头文件
#include <ros/ros.h>
#include <image_transport/image_transport.h>       // 图像传输订阅器
#include <cv_bridge/cv_bridge.h>                   // ROS图像 -> OpenCV图像
#include <sensor_msgs/image_encodings.h>           // 图像编码
#include <geometry_msgs/PointStamped.h>            // 发布偏移坐标点

// OpenCV头文件
#include <opencv2/opencv.hpp>
#include <vector>
#include <iostream>

using namespace std;
using namespace cv;

// 发布器：用于发布相对于图像中心的偏移量
ros::Publisher offset_pub;


/// @brief 提取白色区域的轮廓和质心点
void out_w(const Mat& img, vector<vector<Point>>& w_list, vector<pair<Point, double>>& w_cent)
{
    int min_area = 10;

    // HSV下白色范围
    Scalar lower(0, 0, 230);
    Scalar upper(180, 75, 255);

    // 构建掩码
    Mat mask_w;
    inRange(img, lower, upper, mask_w);

    // 查找轮廓
    vector<vector<Point>> contours;
    findContours(mask_w, contours, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);

    // 遍历每个轮廓
    for (const auto& cnt : contours) {
        double area = contourArea(cnt);
        if (area > min_area) {
            // 最小外接矩形框
            RotatedRect rect = minAreaRect(cnt);
            Point2f box[4];
            rect.points(box);
            vector<Point> boxPts;
            for (int i = 0; i < 4; ++i)
                boxPts.push_back(Point(cvRound(box[i].x), cvRound(box[i].y)));
            w_list.push_back(boxPts);  // 存储白色区域框

            // 计算质心
            Moments m = moments(cnt);
            if (m.m00 != 0) {
                int cx = static_cast<int>(m.m10 / m.m00);
                int cy = static_cast<int>(m.m01 / m.m00);
                w_cent.push_back(make_pair(Point(cx, cy), area));  // 存储中心点
            }
        }
    }
}

/// @brief 提取蓝色区域（用于包围识别白色区域）
vector<RotatedRect> out_blue(Mat img) {
    vector<RotatedRect> blue_list;

    // 蓝色HSV阈值
    //Scalar lower(100, 80, 180);
    //Scalar upper(140, 150, 255);
    Scalar lower(100, 80, 180);
    Scalar upper(140, 150, 255);
    Mat mask_b;
    inRange(img, lower, upper, mask_b);

    // 查找蓝色区域轮廓
    vector<vector<Point>> contours;
    findContours(mask_b, contours, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);

    for (const auto& cnt : contours) {
        double area = contourArea(cnt);
        if (area > 20) {
            RotatedRect rect = minAreaRect(cnt);
            blue_list.emplace_back(rect);
        }
    }

    // 过滤反射/重复区域（保留靠上的目标）
    vector<RotatedRect> filtered_blue_list;
    const int threshold_x_diff = 60;

    while (!blue_list.empty()) {
        RotatedRect current_box = blue_list.front();
        blue_list.erase(blue_list.begin());

        Point2f current_box_center = current_box.center;
        vector<RotatedRect> similar_boxes = {current_box};

        for (const auto& box : blue_list) {
            Point2f box_center = box.center;
            if (abs(current_box_center.x - box_center.x) < threshold_x_diff)
                similar_boxes.push_back(box);
        }

        // 取Y最小的（最靠上的）
        RotatedRect max_y_box = *max_element(
            similar_boxes.begin(), similar_boxes.end(),
            [](const RotatedRect& a, const RotatedRect& b) {
                return a.center.y > b.center.y;
            });

        filtered_blue_list.push_back(max_y_box);

        // 从blue_list中删除已经处理的框
        blue_list.erase(
            remove_if(
                blue_list.begin(), blue_list.end(),
                [&](const RotatedRect& box) {
                    return any_of(
                        similar_boxes.begin(), similar_boxes.end(),
                        [&](const RotatedRect& sb) {
                            return sb.boundingRect() == box.boundingRect();
                        });
                }),
            blue_list.end());
    }

    return filtered_blue_list;
}

/// @brief 图像订阅回调函数：图像处理主逻辑
void imageCallback(const sensor_msgs::ImageConstPtr& msg) {
    cv_bridge::CvImagePtr cv_ptr;

    try {
        // 转换为OpenCV图像
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception& e) {
        ROS_ERROR("cv_bridge error: %s", e.what());
        return;
    }

    Mat frame = cv_ptr->image;
    resize(frame, frame, Size(640, 480));  // 缩放
    Mat hsv_img;
    cvtColor(frame, hsv_img, COLOR_BGR2HSV);  // 转HSV

    // 获取白色中心区域（外轮廓 + 质心）
    vector<vector<Point>> w_list;
    vector<pair<Point, double>> w_cent;
    out_w(hsv_img, w_list, w_cent);

    // 获取蓝色包围区域
    vector<RotatedRect> b_list = out_blue(hsv_img);

    // 绘制黄色白色轮廓
    for (const auto& box : w_list)
        drawContours(frame, vector<vector<Point>>{box}, 0, Scalar(0, 255, 255), 1);

    // 绘制蓝色包围区域
    for (const auto& box : b_list) {
        Point2f vertices[4];
        box.points(vertices);
        vector<Point> pts(4);
        for (int i = 0; i < 4; ++i)
            pts[i] = vertices[i];
        polylines(frame, pts, true, Scalar(255, 0, 0), 2);
    }

    // 绘制绿色中心点
    for (const auto& cent : w_cent) {
        circle(frame, cent.first, 2, Scalar(0, 255, 0), -1);  // 使用 cent.first 提取质心坐标
    }
    // 计算图像中心
    Point image_center(frame.cols / 2, frame.rows / 2);


    vector<pair<Point, double>> hit_points;

    // 判断是否中心点被蓝色框包围，若是，发布相对偏移（红色点 + 发布）
    for (const auto& point : w_cent) {
        Point centroid = point.first;
        for (const auto& box : b_list) {
            Point2f vertices[4];
            box.points(vertices);
            vector<Point> contour(4);
            for (int i = 0; i < 4; ++i)
                contour[i] = vertices[i];

            if (pointPolygonTest(contour, centroid, false) >= 0) {
                // 命中目标
                hit_points.push_back(point);
                break;
            }
        }
    }

    
    if (hit_points.empty()) {
    ROS_WARN("nothing");
    }
    else{
        auto max_it = max_element(hit_points.begin(), hit_points.end(),
                          [](const pair<Point, double>& a, const pair<Point, double>& b) {
                              return a.second < b.second;
                          });

    Point largest_centroid = max_it->first;  // 面积最大的质心坐标
    double largest_area = max_it->second;   // 最大面积值

    // 绘制红色圆点表示面积最大的质心点
    circle(frame, largest_centroid, 3, Scalar(0, 0, 255), -1);

    // 发布偏移坐标
    geometry_msgs::PointStamped offset_msg;
    offset_msg.header.stamp = ros::Time::now();
    offset_msg.header.frame_id = "camera";  // 可换成具体相机frame
    offset_msg.point.x = largest_centroid.x - image_center.x;
    offset_msg.point.y = largest_centroid.y - image_center.y;
    offset_msg.point.z = 0.0;
    offset_pub.publish(offset_msg);
                        }
    imshow("a", frame);
    imshow("b", hsv_img);
    waitKey(1);
    // 显示调试窗口

}

/// @brief 主函数：初始化ROS节点，订阅图像，启动循环
int main(int argc, char** argv) {
    ros::init(argc, argv, "target_node");
    ros::NodeHandle nh;

    // 初始化图像传输器
    image_transport::ImageTransport it(nh);

    // 订阅图像话题（摄像头）
    image_transport::Subscriber image_sub = it.subscribe("/usb_cam/image_raw", 1, imageCallback);

    // 广播目标偏移话题
    offset_pub = nh.advertise<geometry_msgs::PointStamped>("target_point", 10);

    // ROS循环
    ros::spin();
    return 0;
}
