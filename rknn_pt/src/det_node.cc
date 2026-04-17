#include <stdio.h>
#include <memory>
#include <sys/time.h>
#include <ros/ros.h>
#include <std_msgs/Int32MultiArray.h>
#include <opencv2/opencv.hpp> 
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include "rkpt.hpp" 
#include "rknnPool.hpp" 
#include <unordered_map> 
#include <Eigen/Dense> 

cv::Mat ros_frame;

void imageCallback(const sensor_msgs::ImageConstPtr &msg)
{
  try
  {
    ros_frame = cv_bridge::toCvShare(msg, "bgr8")->image.clone();
  }
  catch (cv_bridge::Exception &e)
  {
    ROS_ERROR("No image convert to : %s", e.what());
    return;
  }
}
 
// 类别名称到数字的映射表（修正版）
std::unordered_map<std::string, int> class_name_to_id = {
    {"0", 0}, {"1", 1}, {"2", 2}, {"3", 3}, {"4", 4}, 
    {"5", 5}, {"6", 6}, {"7", 7}, {"8", 8}, {"9", 9},
    {"wrench", 101}, {"soldering_iron", 102}, {"electrodrill", 103},
    {"tape_measure", 104}, {"screwdriver", 105}, {"pliers", 106},
    {"oscillograph", 107}, {"multimeter", 108}, {"printer", 109},
    {"keyboard", 110}, {"mobile_phone", 111}, {"mouse", 112},
    {"headphones", 113}, {"monitor", 114}, {"speaker", 115},
    {"goal", 116}, {"redgoal", 117},{"unhitgoal", 118}
};

int main(int argc, char **argv) {
    setlocale(LC_ALL,"");
    // ROS节点初始化 
    ros::init(argc, argv, "det_node");
    ros::NodeHandle nh;
    ros::NodeHandle private_nh_("~");
    // 参数配置 
    std::string model_path;
    bool draw_result = true;
    int thread_num = 4;
    
    // 从参数服务器获取配置 
    private_nh_.param<std::string>("model_path",  model_path, "/home/duzhong/dzacs/src/rknn_pt/model/duzhong0731.rknn"); 
    private_nh_.param<bool>("draw_result",  draw_result, true);
    private_nh_.param<int>("thread_num",  thread_num, 4);

    image_transport::ImageTransport it(nh);
    image_transport::Subscriber image_sub = it.subscribe("/usb_cam/image_raw", 1, imageCallback);
    
    // 创建发布器和订阅器 
    ros::Publisher det_pub = nh.advertise<std_msgs::Int32MultiArray>("/pt_det_topic",  10);
    // 初始化RKNN推理池 
    rknnPool<RkPt, cv::Mat, DetectResultsGroup> detectPool(model_path, thread_num);
    if (detectPool.init()  != 0) {
        ROS_ERROR("RKNN推理池初始化失败!");
        return -1;
    }
    
    // 计时变量 
    struct timeval time;
    gettimeofday(&time, nullptr);
    auto start_time = time.tv_sec  * 1000 + time.tv_usec  / 1000;
    int frames = 0;

    // cv::VideoCapture cap("/dev/video0", cv::CAP_V4L2);

    // cap.set(cv::CAP_PROP_FOURCC,
    //         cv::VideoWriter::fourcc('M','J','P','G'));
            
    // cap.set(cv::CAP_PROP_FRAME_WIDTH, 640);
    // cap.set(cv::CAP_PROP_FRAME_HEIGHT, 480);
    // cap.set(cv::CAP_PROP_FPS, 120); 

    
    // ROS主循环 
    ros::Rate loop_rate(60);
     ros::Duration(3).sleep();//ysvv,防止一开启所有节点就开始识别输入数据
    while (ros::ok()) {

        // cap >> ros_frame;
        // 准备发布消息 
        
        std_msgs::Int32MultiArray msg;
        
        // 设置消息布局（每个检测结果包含4个数据: x偏移, y偏移, 类别ID, 置信度）
        msg.layout.dim.push_back(std_msgs::MultiArrayDimension()); 
        msg.layout.dim[0].label  = "detections";  // 检测结果标签 
        msg.layout.dim[0].size  = 0;              // 初始化为0，后面会更新 
        msg.layout.dim[0].stride  = 4;            // 每个检测结果包含4个数据 
        
        if (!ros_frame.empty())  {
            // 将图像放入推理池 
            if (detectPool.put(ros_frame,  frames) != 0) {
                ROS_WARN("rknnPool init fail!");
                continue;
            }
            
            // 获取推理结果(当累积帧数≥线程数时才开始获取)
            DetectResultsGroup results_group;
            if (frames >= thread_num && detectPool.get(results_group)  == 0) {
                // 处理检测结果 
                for (const auto &res : results_group.dets)  {
                    // 计算目标中心相对于图像中心的偏移 
                    int center_x = res.box.x  + res.box.width  / 2;//目标中心x坐标
                    int center_y = res.box.y  + res.box.height  / 2;//目标中心y坐标

                    int offset_x = center_x - results_group.cur_img.cols  / 2;//目标中心相对于图像中心的x偏移
                    int offset_y = center_y - results_group.cur_img.rows  / 2;//目标中心相对于图像中心的y偏移
                    
                    // 查找类别ID 
                    int class_id = class_name_to_id.count(res.det_name)  ? 
                                  class_name_to_id[res.det_name] : -1;
                    
                    // 将分数转换为整数百分比(0-100)
                    int score_percent = static_cast<int>(res.score  * 100);

                    
                    // 填充消息数据: [x中心, y中心, 类别ID, 置信度(0-100)]
                    msg.data.push_back(offset_x);
                    msg.data.push_back(offset_y); 
                    msg.data.push_back(class_id); 
                    msg.data.push_back(score_percent); 
                    
                }
                 if (msg.data.empty()) {//什么也没识别到的时候 发送-1
                msg.data.push_back(-1);
                msg.data.push_back(-1);
                msg.data.push_back(-1);
                msg.data.push_back(-1);
                }
                
                // 更新消息布局中的尺寸信息 
                msg.layout.dim[0].size  = msg.data.size(); 
                
                // 发布检测结果 
                det_pub.publish(msg); 
                
                // 可视化结果 
                if (draw_result) {
                    show_draw_results(results_group);
                    cv::imshow("Detection Results", results_group.cur_img); 
                    if (cv::waitKey(1) == 'q') break;  // 按q键退出 
                }
            }
            
            frames++;
        }
        
        ros::spinOnce();
        loop_rate.sleep(); 
    }

    while (true)
    {
        DetectResultsGroup results_group;
        if (detectPool.get(results_group) != 0)
            break;
        frames++;
    }

    // cap.release();
    cv::destroyAllWindows();

    gettimeofday(&time, nullptr);
    auto endTime = time.tv_sec * 1000 + time.tv_usec / 1000;

    printf("Average:\t %f fps/s\n", float(frames) / float(endTime - start_time) * 1000.0);

    
    return 0;
}