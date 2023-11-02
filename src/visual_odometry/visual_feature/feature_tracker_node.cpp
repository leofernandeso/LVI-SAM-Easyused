#include <ros/ros.h>

#include "feature_tracker.h"
#include "ros/console_backend.h"
#include "sensor_msgs/Image.h"
#include "sensor_msgs/PointCloud2.h"

#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#define SHOW_UNDISTORTION 0

// mtx lock for two threads
std::mutex mtx_lidar;

// global variable for saving the depthCloud shared between two threads
pcl::PointCloud<PointType>::Ptr depthCloud(new pcl::PointCloud<PointType>());

// global variables saving the lidar point cloud
deque<pcl::PointCloud<PointType>> cloudQueue;
deque<double> timeQueue;

// global depth register for obtaining depth of a feature
DepthRegister *depthRegister;

// feature publisher for VINS estimator
ros::Publisher pub_feature;
ros::Publisher pub_match;
ros::Publisher pub_restart;

void callback(const sensor_msgs::ImageConstPtr& left_img_msg, const sensor_msgs::ImageConstPtr& right_img_msg, const sensor_msgs::PointCloud2ConstPtr& laser_msg)
{
  ROS_INFO("Processing image and lidar point cloud...");
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "vins");
  ros::NodeHandle n;
  ROS_INFO("\033[1;32m----> Visual Feature Tracker Started.\033[0m");
  ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME,
                                 ros::console::levels::Info);
  readParameters(n);

  // subscribers. use a message filter to software-synchronize the images and lidar
  message_filters::Subscriber<sensor_msgs::Image> left_img_sub(n, LEFT_IMAGE_TOPIC, 5);
  message_filters::Subscriber<sensor_msgs::Image> right_img_sub(n, RIGHT_IMAGE_TOPIC, 5);
  message_filters::Subscriber<sensor_msgs::PointCloud2> lidar_sub(n, POINT_CLOUD_TOPIC, 5);

  typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image, sensor_msgs::PointCloud2> MySyncPolicy;
  message_filters::Synchronizer<MySyncPolicy> sync(MySyncPolicy(10), left_img_sub, right_img_sub, lidar_sub);
  sync.registerCallback(boost::bind(&callback, _1, _2, _3));

  // messages to vins estimator
  pub_feature = n.advertise<sensor_msgs::PointCloud>(
      PROJECT_NAME + "/vins/feature/feature", 5);
  pub_match = n.advertise<sensor_msgs::Image>(
      PROJECT_NAME + "/vins/feature/feature_img", 5);
  pub_restart =
      n.advertise<std_msgs::Bool>(PROJECT_NAME + "/vins/feature/restart", 5);

  ros::spin();

  return 0;
}
