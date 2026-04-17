#include "savepath.h"

savepath::savepath(ros::NodeHandle nh)
{

  receivePose_flag = false;
  nh.param("/dzsavepath/mappinmgsavePath", mappinmgsavePath, std::string("/home/duzhong/dzacs/src/resource/mapping.txt"));

  // 检查文件是否存在，并删除
  if (std::filesystem::exists(mappinmgsavePath))
  {
      std::filesystem::remove(mappinmgsavePath); // 删除文件
      std::cout << FRED("File exists and has been removed: ") << mappinmgsavePath << std::endl;
  }
  else
  {
      std::cout << FGRN("File does not exist: ") << mappinmgsavePath << std::endl;
  }

  std::cout << FRED("Copyright©2016-2020 duzhong robot. All rights reserved ") << std::endl;
  std::cout << FYEL("*****dzsavepath:parameters*******************") << std::endl;
  std::cout << FGRN("mappinmgsavePath: ") << mappinmgsavePath << std::endl;
  std::cout << FYEL("*****dzsavepath:parameters end***************") << std::endl;

  amcl_pose_sub_ = nh.subscribe("lidar_pose", 50, &savepath::amcl_pose_callback, this);
}

savepath::~savepath()
{
}

void savepath::run()
{
  ros::Rate rate(20);
  // ros::Time current_time, last_time;
  while (ros::ok())
  {
    ros::spinOnce();
    if (receivePose_flag)
    {
      if (distance(current_position, last_position) >= 0.02)
      {
        fp = fopen(mappinmgsavePath.c_str(), "a");
        fprintf(fp, "%.4lf,%.4lf,%.4lf,%.4lf,%.4lf,%.4lf,%.4lf,\n",
                current_position.pose.pose.position.x,
                current_position.pose.pose.position.y,
                current_position.pose.pose.position.z,
                current_position.pose.pose.orientation.x,
                current_position.pose.pose.orientation.y,
                current_position.pose.pose.orientation.z,
                current_position.pose.pose.orientation.w);
        fclose(fp);
        last_position = current_position;
      }
    }

    rate.sleep();
  }
}

void savepath::amcl_pose_callback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &msg)
{
  current_position = *msg;
  receivePose_flag = true;
}

double savepath::distance(const geometry_msgs::PoseWithCovarianceStamped &p1, const geometry_msgs::PoseWithCovarianceStamped &p2)
{
  return hypot(p1.pose.pose.position.x - p2.pose.pose.position.x, p1.pose.pose.position.y - p2.pose.pose.position.y);
}
