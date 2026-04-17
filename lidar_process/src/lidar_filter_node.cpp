#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <limits> 
#include <cmath> 

class CLidarFilter
{
public:
    CLidarFilter(ros::NodeHandle& nh, ros::NodeHandle& pnh); 
private:
     ros::NodeHandle n_; 
     ros::Publisher scan_pub_;
     ros::Subscriber scan_sub_;
     std::string source_topic_name_;
     std::string pub_topic_name_;
     double outlier_threshold_;
     double angle_min_;       
     double angle_max_;       
     bool enable_angle_filter_; 

     void lidarCallback(const sensor_msgs::LaserScan::ConstPtr& scan);
};

CLidarFilter::CLidarFilter(ros::NodeHandle& nh, ros::NodeHandle& pnh) : n_(nh)
{
    pnh.param<std::string>("source_topic", source_topic_name_, "/scan");  //定义source_topic默认值为/scan     
    pnh.param<std::string>("pub_topic", pub_topic_name_, "/scan_filtered");  //定义pub_topic默认值为/scan_filtered   
    pnh.param<double>("outlier_threshold", outlier_threshold_, 0.1);  //outlier是离群点的意思，定义一个参数outlier_threshold，默认值为0.1米，表示如果一个点和它左右邻居的距离差超过这个值，就认为它是离群点。          
    
    pnh.param<bool>("enable_angle_filter", enable_angle_filter_, false);         
    pnh.param<double>("angle_min", angle_min_, 0.0);                            
    pnh.param<double>("angle_max", angle_max_, 0.0);                             
    
    ROS_INFO("Lidar filter initialized with angle filter: %s, min: %.2f rad, max: %.2f rad", 
             enable_angle_filter_ ? "enabled" : "disabled", angle_min_, angle_max_);

    scan_pub_ = n_.advertise<sensor_msgs::LaserScan>(pub_topic_name_, 10);//发布到话题/scan_filtered中
    scan_sub_ = n_.subscribe<sensor_msgs::LaserScan>(source_topic_name_, 10, &CLidarFilter::lidarCallback, this);//订阅话题/scan中的激光雷达数据，回调函数是lidarCallback
}

void CLidarFilter::lidarCallback(const sensor_msgs::LaserScan::ConstPtr& scan)
{
    int nRanges = scan->ranges.size();

    if (nRanges < 3) 
    {
        scan_pub_.publish(scan);
        return;
    }

    sensor_msgs::LaserScan new_scan;

    new_scan.header = scan->header; 
    new_scan.angle_min = scan->angle_min;
    new_scan.angle_max = scan->angle_max;
    new_scan.angle_increment = scan->angle_increment;
    new_scan.time_increment = scan->time_increment;
    new_scan.scan_time = scan->scan_time; 
    new_scan.range_min = scan->range_min; 
    new_scan.range_max = scan->range_max;
    new_scan.ranges = scan->ranges; 
    if (!scan->intensities.empty()) 
    {
        new_scan.intensities = scan->intensities; 
    }
    
    if (enable_angle_filter_)
    {
        for (int i = 0; i < nRanges; ++i)
        {
            float current_angle = scan->angle_min + i * scan->angle_increment;
            
            while (current_angle > M_PI) current_angle -= 2.0 * M_PI;
            while (current_angle < -M_PI) current_angle += 2.0 * M_PI;
            
            if (current_angle >= angle_min_ && current_angle <= angle_max_)
            {
                new_scan.ranges[i] = std::numeric_limits<float>::infinity();
                if (!new_scan.intensities.empty() && i < new_scan.intensities.size()) 
                {
                    new_scan.intensities[i] = 0.0f; 
                }
            }
        }
    }
	//离群点滤波（去噪点）：一个点如果和左右邻居都差很多 → 判定为噪点 → 删掉
    for (int i = 1; i < nRanges - 1; ++i)
    {
        float prev_range = new_scan.ranges[i-1];
        float current_range = new_scan.ranges[i];
        float next_range = new_scan.ranges[i+1];

        // 先判断当前点是否有效，如果无效直接跳过，不参与离群点判断，避免误删有效点
        bool current_valid = std::isfinite(current_range) && 
                             current_range >= new_scan.range_min && 
                             current_range <= new_scan.range_max;

        // 如果无效跳过
        if (!current_valid) 
        {
            continue;
        }

        
        bool prev_valid = std::isfinite(prev_range) &&
                          prev_range >= new_scan.range_min &&
                          prev_range <= new_scan.range_max;
        
        bool next_valid = std::isfinite(next_range) &&
                          next_range >= new_scan.range_min &&
                          next_range <= new_scan.range_max;
        // 如果左右都有效
        if (prev_valid && next_valid) {
            // 判定为离群点 → 删掉！
            if (std::abs(current_range - prev_range) > outlier_threshold_ &&
                std::abs(current_range - next_range) > outlier_threshold_)
            {
                new_scan.ranges[i] = std::numeric_limits<float>::infinity();
                if (!new_scan.intensities.empty() && i < new_scan.intensities.size()) 
                {
                    new_scan.intensities[i] = 0.0f; 
                }
            }
        }
    }
//去掉那种突然很远 / 突然很近的孤立噪点。

    scan_pub_.publish(new_scan);//发布到话题/scan_filtered中
}

int main(int argc, char** argv)
{
    ros::init(argc,argv,"lidar_filter_node");

    ros::NodeHandle nh; 
    ros::NodeHandle pnh("~"); 

    CLidarFilter lidar_filter(nh, pnh);
    
    ros::spin();
    return 0; 
}