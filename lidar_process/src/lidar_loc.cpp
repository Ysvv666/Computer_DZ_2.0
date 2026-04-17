#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/RegionOfInterest.h>
#include <sensor_msgs/LaserScan.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <std_msgs/String.h>
#include <std_srvs/Empty.h>
#include <vector>
#include <cmath>

nav_msgs::OccupancyGrid map_msg;
cv::Mat map_cropped;
cv::Mat map_temp;
sensor_msgs::RegionOfInterest map_roi_info;
std::vector<cv::Point2f> scan_points;
std::vector<cv::Point2f> best_transform;
// ros::ServiceClient clear_costmaps_client;
std::string base_frame;
std::string odom_frame;
std::string laser_frame;
std::string laser_topic;

float lidar_x = 250, lidar_y = 250, lidar_yaw = 0;
float deg_to_rad = M_PI / 180.0;
int cur_sum = 0;
int clear_countdown = -1;
int scan_count = 0;

ros::Publisher lidar_pose_pub;

void initialPoseCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg)
{
    double map_x = msg->pose.pose.position.x;
    double map_y = msg->pose.pose.position.y;
    tf2::Quaternion q;
    tf2::fromMsg(msg->pose.pose.orientation, q);
    tf2::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);
    if (map_msg.info.resolution <= 0) {
        ROS_ERROR("Map information is invalid or has not been received");
        return;
    }
    lidar_x = (map_x - map_msg.info.origin.position.x) / map_msg.info.resolution - map_roi_info.x_offset;
    lidar_y = (map_y - map_msg.info.origin.position.y) / map_msg.info.resolution - map_roi_info.y_offset;
    lidar_yaw = -yaw;
    clear_countdown = 30;
}

void crop_map();
void processMap();

void mapCallback(const nav_msgs::OccupancyGrid::ConstPtr& msg)
{
    map_msg = *msg;
    crop_map();
    processMap();        
}

void crop_map()
{
    std_msgs::Header header = map_msg.header;
    nav_msgs::MapMetaData info = map_msg.info;
    int xMax,xMin,yMax,yMin ;
    xMax=xMin= info.width/2;
    yMax=yMin= info.height/2;
    bool bFirstPoint = true;
    cv::Mat map_raw(info.height, info.width, CV_8UC1, cv::Scalar(128)); 
    for(int y = 0; y < info.height; y++)
    {
        for(int x = 0; x < info.width; x++)
        {
            int index = y * info.width + x;
            map_raw.at<uchar>(y, x) = static_cast<uchar>(map_msg.data[index]);
            if(map_msg.data[index] == 100)
            {
                if(bFirstPoint)
                {
                    xMax = xMin = x;
                    yMax = yMin = y;
                    bFirstPoint = false;
                    continue;
                }
                xMin = std::min(xMin, x);
                xMax = std::max(xMax, x);
                yMin = std::min(yMin, y);
                yMax = std::max(yMax, y);
            }
        }
    }
    int cen_x = (xMin + xMax)/2;
    int cen_y = (yMin + yMax)/2;
    int new_half_width = abs(xMax - xMin)/2 + 50;
    int new_half_height = abs(yMax - yMin)/2 + 50;
    int new_origin_x = cen_x - new_half_width;
    int new_origin_y = cen_y - new_half_height;
    int new_width = new_half_width*2;
    int new_height = new_half_height*2;
    if(new_origin_x < 0) new_origin_x = 0;
    if((new_origin_x + new_width) > info.width) new_width = info.width - new_origin_x;
    if(new_origin_y < 0) new_origin_y = 0;
    if((new_origin_y + new_height) > info.height) new_height = info.height - new_origin_y;
    cv::Rect roi(new_origin_x, new_origin_y, new_width, new_height);
    cv::Mat roi_map = map_raw(roi).clone();
    map_cropped = roi_map;
    map_roi_info.x_offset = new_origin_x;
    map_roi_info.y_offset = new_origin_y;
    map_roi_info.width = new_width;
    map_roi_info.height = new_height;
    geometry_msgs::PoseWithCovarianceStamped init_pose;
    init_pose.pose.pose.position.x = 0.0;
    init_pose.pose.pose.position.y = 0.0;
    init_pose.pose.pose.position.y = 0.0;
    init_pose.pose.pose.orientation.x = 0.0;
    init_pose.pose.pose.orientation.y = 0.0;
    init_pose.pose.pose.orientation.z = 0.0;
    init_pose.pose.pose.orientation.w = 1.0;
    geometry_msgs::PoseWithCovarianceStamped::ConstPtr init_pose_ptr(new geometry_msgs::PoseWithCovarianceStamped(init_pose));
    initialPoseCallback(init_pose_ptr);
}

static bool lidar_is_inverted = false;
bool check(float x, float y, float yaw);
void scanCallback(const sensor_msgs::LaserScan::ConstPtr& msg)
{
    scan_points.clear();
    double angle = msg->angle_min;
    static tf2_ros::Buffer tfBuffer;
    static tf2_ros::TransformListener tfListener(tfBuffer);
    static bool tf_available = false;
    static bool first_scan_received = false;
    if (!first_scan_received) {
        first_scan_received = true;
        ROS_INFO("First laser scan data received");
    }
    if (!tf_available) {
        try {
            tfBuffer.lookupTransform(base_frame, laser_frame, ros::Time(0), ros::Duration(0.1));
            tf_available = true;
            ROS_INFO("TF available: %s -> %s", laser_frame.c_str(), base_frame.c_str());
        }
        catch (tf2::TransformException &ex) {
            static bool first_warning = true;
            if (first_warning) {
                ROS_WARN("Waiting for TF: %s -> %s", laser_frame.c_str(), base_frame.c_str());
                first_warning = false;
            }
            return;
        }
    }
    geometry_msgs::TransformStamped transformStamped;
    try {
        transformStamped = tfBuffer.lookupTransform(base_frame, laser_frame, ros::Time(0));
    }
    catch (tf2::TransformException &ex) {
        ROS_WARN("Failed to get TF transform: %s", ex.what());
        tf_available = false;  
        return;
    }
    tf2::Quaternion q_lidar;
    tf2::fromMsg(transformStamped.transform.rotation, q_lidar);
    double roll, pitch, yaw;
    tf2::Matrix3x3(q_lidar).getRPY(roll, pitch, yaw);
    const double tolerance = 0.1;
    bool lidar_is_inverted = std::abs(std::abs(roll) - M_PI) < tolerance;
    lidar_is_inverted *= !(std::abs(std::abs(pitch) - M_PI) < tolerance);
    for (size_t i = 0; i < msg->ranges.size(); ++i)
    {
        if (msg->ranges[i] >= msg->range_min && msg->ranges[i] <= msg->range_max)
        {
            float x_laser = msg->ranges[i] * cos(angle);
            float y_laser = -msg->ranges[i] * sin(angle);
            geometry_msgs::PointStamped point_laser;
            point_laser.header.frame_id = laser_frame;
            point_laser.header.stamp = msg->header.stamp;
            point_laser.point.x = x_laser;
            point_laser.point.y = y_laser;
            point_laser.point.z = 0.0;
            geometry_msgs::PointStamped point_base;
            tf2::doTransform(point_laser, point_base, transformStamped);
            float x = point_base.point.x / map_msg.info.resolution;
            float y = point_base.point.y / map_msg.info.resolution;
            if (lidar_is_inverted)
            {
                x = -x;
                y = -y;
            }
            scan_points.push_back(cv::Point2f(x, y));
        }
        angle += msg->angle_increment;
    }
    if(scan_count == 0)
        scan_count ++;
    while (ros::ok())
    {
        if (!map_cropped.empty())
        {
            std::vector<cv::Point2f> transform_points, clockwise_points, counter_points;
            int max_sum = 0;
            float best_dx = 0, best_dy = 0, best_dyaw = 0;
            for (const auto& point : scan_points)
            {
                float rotated_x = point.x * cos(lidar_yaw) - point.y * sin(lidar_yaw);
                float rotated_y = point.x * sin(lidar_yaw) + point.y * cos(lidar_yaw);
                transform_points.push_back(cv::Point2f(rotated_x + lidar_x, lidar_y - rotated_y));
                float clockwise_yaw = lidar_yaw + deg_to_rad;
                rotated_x = point.x * cos(clockwise_yaw) - point.y * sin(clockwise_yaw);
                rotated_y = point.x * sin(clockwise_yaw) + point.y * cos(clockwise_yaw);
                clockwise_points.push_back(cv::Point2f(rotated_x + lidar_x, lidar_y - rotated_y));
                float counter_yaw = lidar_yaw - deg_to_rad;
                rotated_x = point.x * cos(counter_yaw) - point.y * sin(counter_yaw);
                rotated_y = point.x * sin(counter_yaw) + point.y * cos(counter_yaw);
                counter_points.push_back(cv::Point2f(rotated_x + lidar_x, lidar_y - rotated_y));
            }
            std::vector<cv::Point2f> offsets = {{0,0}, {1,0}, {-1,0}, {0,1}, {0,-1}};
            std::vector<std::vector<cv::Point2f>> point_sets = {transform_points, clockwise_points, counter_points};
            std::vector<float> yaw_offsets = {0, deg_to_rad, -deg_to_rad};
            for (int i = 0; i < offsets.size(); ++i)
            {
                for (int j = 0; j < point_sets.size(); ++j)
                {
                    int sum = 0;
                    for (const auto& point : point_sets[j])
                    {
                        float px = point.x + offsets[i].x;
                        float py = point.y + offsets[i].y;
                        if (px >= 0 && px < map_temp.cols && py >= 0 && py < map_temp.rows)
                        {
                            sum += map_temp.at<uchar>(py, px);
                        }
                    }
                    if (sum > max_sum)
                    {
                        max_sum = sum;
                        best_dx = offsets[i].x;
                        best_dy = offsets[i].y;
                        best_dyaw = yaw_offsets[j];
                    }
                }
            }
            lidar_x += best_dx;
            lidar_y += best_dy;
            lidar_yaw += best_dyaw;
            if(check(lidar_x , lidar_y , lidar_yaw))
            {
                break;
            }
        }
        else
        {
            break;
        }
    }
    // if(clear_countdown > -1)
    //     clear_countdown --;
    // if(clear_countdown == 0)
    // {
    //     std_srvs::Empty srv;
    //     clear_costmaps_client.call(srv);
    // }
}

std::deque<std::tuple<float, float, float>> data_queue;
const size_t max_size = 10;
bool check(float x, float y, float yaw)
{
    if(x == 0 && y == 0 && yaw == 0)
    {
        data_queue.clear();
        return true;
    }
    data_queue.push_back(std::make_tuple(x, y, yaw));
    if (data_queue.size() > max_size) 
    {
        data_queue.pop_front();
    }
    if (data_queue.size() == max_size) 
    {
        auto& first = data_queue.front();
        auto& last = data_queue.back();
        float dx = std::abs(std::get<0>(last) - std::get<0>(first));
        float dy = std::abs(std::get<1>(last) - std::get<1>(first));
        float dyaw = std::abs(std::get<2>(last) - std::get<2>(first));
        if (dx < 5 && dy < 5 && dyaw < 5*deg_to_rad)
        {
            data_queue.clear();
            return true;
        }
    }
    return false;
}

cv::Mat createGradientMask(int size)
{
    cv::Mat mask(size, size, CV_8UC1);
    int center = size / 2;
    for (int y = 0; y < size; y++)
    {
        for (int x = 0; x < size; x++)
        {
            double distance = std::hypot(x - center, y - center);
            int value = cv::saturate_cast<uchar>(255 * std::max(0.0, 1.0 - distance / center));
            mask.at<uchar>(y, x) = value;
        }
    }
    return mask;
}

void processMap()
{
    if (map_cropped.empty()) return;
    map_temp = cv::Mat::zeros(map_cropped.size(), CV_8UC1);
    cv::Mat gradient_mask = createGradientMask(101);  
    for (int y = 0; y < map_cropped.rows; y++)
    {
        for (int x = 0; x < map_cropped.cols; x++)
        {
            if (map_cropped.at<uchar>(y, x) == 100)  
            {
                int left = std::max(0, x - 50);
                int top = std::max(0, y - 50);
                int right = std::min(map_cropped.cols - 1, x + 50);
                int bottom = std::min(map_cropped.rows - 1, y + 50);
                cv::Rect roi(left, top, right - left + 1, bottom - top + 1);
                cv::Mat region = map_temp(roi);
                int mask_left = 50 - (x - left);
                int mask_top = 50 - (y - top);
                cv::Rect mask_roi(mask_left, mask_top, roi.width, roi.height);
                cv::Mat mask = gradient_mask(mask_roi);
                cv::max(region, mask, region);
            }
        }
    }
}

void pose_tf()
{
    if (scan_count == 0) return;
    if (map_cropped.empty() || map_msg.data.empty() || map_msg.info.resolution <= 0) return;
    static tf2_ros::Buffer tfBuffer;
    static tf2_ros::TransformListener tfListener(tfBuffer);
    static bool tf_available = false;
    static bool first_tf_check = true;
    double full_map_pixel_x = lidar_x + map_roi_info.x_offset;
    double full_map_pixel_y = lidar_y + map_roi_info.y_offset;
    double x_in_map_frame = full_map_pixel_x * map_msg.info.resolution + map_msg.info.origin.position.x;
    double y_in_map_frame = full_map_pixel_y * map_msg.info.resolution + map_msg.info.origin.position.y;
    double yaw_in_map_frame = -lidar_yaw;
    tf2::Transform map_to_base;
    map_to_base.setOrigin(tf2::Vector3(x_in_map_frame, y_in_map_frame, 0.0));
    tf2::Quaternion q;
    q.setRPY(0, 0, yaw_in_map_frame);
    map_to_base.setRotation(q);
    if (!tf_available) {
        try {
            tfBuffer.lookupTransform(odom_frame, base_frame, ros::Time(0), ros::Duration(0.1));
            tf_available = true;
            if (first_tf_check) {
                ROS_INFO("TF available: %s -> %s", odom_frame.c_str(), base_frame.c_str());
                first_tf_check = false;
            }
        }
        catch (tf2::TransformException &ex) {
            if (first_tf_check) {
                ROS_WARN("Waiting for TF: %s -> %s", odom_frame.c_str(), base_frame.c_str());
                first_tf_check = false;
            }
            return;
        }
    }
    geometry_msgs::TransformStamped odom_to_base_msg;
    try {
        odom_to_base_msg = tfBuffer.lookupTransform(odom_frame, base_frame, ros::Time(0));
    }
    catch (tf2::TransformException &ex) {
        ROS_WARN("Failed to get TF: %s -> %s: %s", odom_frame.c_str(), base_frame.c_str(), ex.what());
        tf_available = false;  
        return;
    }
    tf2::Transform odom_to_base_tf2;
    tf2::fromMsg(odom_to_base_msg.transform, odom_to_base_tf2);
    tf2::Transform map_to_odom = map_to_base * odom_to_base_tf2.inverse();
    static tf2_ros::TransformBroadcaster br;
    geometry_msgs::TransformStamped map_to_odom_msg;
    map_to_odom_msg.header.stamp = ros::Time::now(); 
    map_to_odom_msg.header.frame_id = "map";
    map_to_odom_msg.child_frame_id = odom_frame;
    map_to_odom_msg.transform = tf2::toMsg(map_to_odom);
    br.sendTransform(map_to_odom_msg);
    geometry_msgs::PoseWithCovarianceStamped lidar_pose_msg;
    lidar_pose_msg.header.stamp = ros::Time::now();
    lidar_pose_msg.header.frame_id = "map";
    lidar_pose_msg.pose.pose.position.x = x_in_map_frame;
    lidar_pose_msg.pose.pose.position.y = y_in_map_frame;
    lidar_pose_msg.pose.pose.position.z = 0.0;
    tf2::Quaternion q_pose;
    q_pose.setRPY(0, 0, yaw_in_map_frame);
    lidar_pose_msg.pose.pose.orientation = tf2::toMsg(q_pose);
    for (int i = 0; i < 36; ++i) lidar_pose_msg.pose.covariance[i] = 0;
    lidar_pose_pub.publish(lidar_pose_msg);
}

int main(int argc, char** argv)
{
    setlocale(LC_ALL,"");
    ros::init(argc, argv, "lidar_loc");
    ros::NodeHandle private_nh("~");
    private_nh.param<std::string>("base_frame", base_frame, "base_link");
    private_nh.param<std::string>("odom_frame", odom_frame, "odom");
    private_nh.param<std::string>("laser_frame", laser_frame, "laser");
    private_nh.param<std::string>("laser_topic", laser_topic, "scan");
    ros::NodeHandle nh;
    ros::Subscriber map_sub = nh.subscribe("map", 1, mapCallback);
    ros::Subscriber scan_sub = nh.subscribe(laser_topic, 1, scanCallback);
    ros::Subscriber initial_pose_sub = nh.subscribe("initialpose", 1, initialPoseCallback);
    // clear_costmaps_client = nh.serviceClient<std_srvs::Empty>("move_base/clear_costmaps");
    lidar_pose_pub = nh.advertise<geometry_msgs::PoseWithCovarianceStamped>("lidar_pose", 10);
    ros::Rate rate(50);  
    while (ros::ok())
    {
        pose_tf();
        ros::spinOnce();
        rate.sleep();
    }
    return 0;
}