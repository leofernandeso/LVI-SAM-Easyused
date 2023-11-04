#include <ros/ros.h>

#include <future>

#include "feature_tracker.h"
#include "ros/console_backend.h"
#include "ros/node_handle.h"
#include "sensor_msgs/Image.h"
#include "sensor_msgs/PointCloud2.h"

#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/time_synchronizer.h>

#define SHOW_UNDISTORTION 0

// mtx lock for two threads
std::mutex mtx_lidar;

// global variable for saving the depthCloud shared between two threads
pcl::PointCloud<PointType>::Ptr depthCloud(new pcl::PointCloud<PointType>());

// global variables saving the lidar point cloud
deque<pcl::PointCloud<PointType>> cloudQueue;
deque<double> timeQueue;

std::array<std::string, 2> CAMS{
    "cam0", "cam1"}; // TODO: parse this directly from the calibration file

cv::Mat imageMsgToMat(const sensor_msgs::ImageConstPtr &img_msg) {
  cv_bridge::CvImageConstPtr ptr;
  if (img_msg->encoding == "8UC1") {
    sensor_msgs::Image img;
    img.header = img_msg->header;
    img.height = img_msg->height;
    img.width = img_msg->width;
    img.is_bigendian = img_msg->is_bigendian;
    img.step = img_msg->step;
    img.data = img_msg->data;
    img.encoding = "mono8";
    ptr = cv_bridge::toCvCopy(img, sensor_msgs::image_encodings::MONO8);
  } else {
    ptr = cv_bridge::toCvCopy(img_msg, sensor_msgs::image_encodings::MONO8);
  }
  return ptr->image;
}

class FeatureTrackerWrapper {
public:
  FeatureTrackerWrapper(const std::string &calibration_file_path) {
    setupPublishers();
    initializeTrackers(calibration_file_path);
  }

  void initializeTrackers(const std::string &calibration_file_path) {
    for (const std::string &cam_name : CAMS) {
      camera_trackers_.insert(std::make_pair(cam_name, FeatureTracker()));
      camera_trackers_[cam_name].readIntrinsicParameter(calibration_file_path,
                                                        cam_name);
    }
  }

  void setupPublishers() {

    for (const std::string &cam_name : CAMS) {
      camera_pubs_.insert(std::make_pair(
          cam_name,
          nh_.advertise<sensor_msgs::PointCloud>(
              PROJECT_NAME + "/vins/feature/feature/" + cam_name, 5)));
    }
  }

  void processFrame(const sensor_msgs::ImageConstPtr &left_img_msg,
                    const sensor_msgs::ImageConstPtr &right_img_msg,
                    const sensor_msgs::PointCloud2ConstPtr &laser_msg) {
    ROS_INFO("Processing data frame...");


    std::vector<std::future<void>> futures;
    futures.emplace_back(std::async(std::launch::async, &FeatureTrackerWrapper::processLidar, this, laser_msg));
    for (size_t i = 0; i < CAMS.size(); ++i) {
      futures.emplace_back(std::async(std::launch::async, &FeatureTrackerWrapper::processImage, this,
                                      left_img_msg,
                                      std::ref(camera_trackers_.at(CAMS[i])), i));
    }

    // wait for all threads to finish
    while (!futures.empty()) {
      for (auto it = futures.begin(); it != futures.end();) {
        if (it->wait_for(std::chrono::milliseconds(0)) == std::future_status::ready) {
          it = futures.erase(it);
        } else {
          ++it;
        }
      }
    }

    /* std::thread lidar_thread(&FeatureTrackerWrapper::processLidar, this, */
    /*                          laser_msg); */
    /* std::vector<std::thread> img_threads; */
    /* for (size_t i = 0; i < CAMS.size(); ++i) { */
    /*   img_threads.emplace_back(&FeatureTrackerWrapper::processImage, this, */
    /*                            left_img_msg, */
    /*                            std::ref(camera_trackers_.at(CAMS[i])), i); */
    /* } */

    /* lidar_thread.join(); */
    /* for (auto &img_thread : img_threads) { */
    /*   img_thread.join(); */
    /* } */
  }

  void processImage(const sensor_msgs::ImageConstPtr &img_msg,
                    FeatureTracker &tracker, size_t cam_id) {
    ROS_INFO("Processing image from cam %zu...", cam_id);
    double cur_img_time = img_msg->header.stamp.toSec();
    cv::Mat image = imageMsgToMat(img_msg);
    tracker.readImage(image, cur_img_time);

    // not sure what this does for now
    for (unsigned int i = 0;; i++) {
      bool completed = false;
      for (int j = 0; j < NUM_OF_CAM; j++)
        if (j != 1 || !STEREO_TRACK)
          completed |= tracker.updateID(i);
      if (!completed)
        break;
    }

    sensor_msgs::PointCloudPtr feature_points(new sensor_msgs::PointCloud);
    sensor_msgs::ChannelFloat32 id_of_point;
    sensor_msgs::ChannelFloat32 u_of_point;
    sensor_msgs::ChannelFloat32 v_of_point;
    sensor_msgs::ChannelFloat32 velocity_x_of_point;
    sensor_msgs::ChannelFloat32 velocity_y_of_point;

    feature_points->header.stamp = img_msg->header.stamp;
    feature_points->header.frame_id = "vins_body";

    auto &un_pts = tracker.cur_un_pts;
    auto &cur_pts = tracker.cur_pts;
    auto &ids = tracker.ids;
    auto &pts_velocity = tracker.pts_velocity;
    for (unsigned int j = 0; j < ids.size(); j++) {
      if (tracker.track_cnt[j] > 1) {
        int p_id = ids[j];
        geometry_msgs::Point32 p;
        p.x = un_pts[j].x;
        p.y = un_pts[j].y;
        p.z = 1;

        feature_points->points.push_back(p);
        id_of_point.values.push_back(p_id * NUM_OF_CAM + cam_id);
        u_of_point.values.push_back(cur_pts[j].x);
        v_of_point.values.push_back(cur_pts[j].y);
        velocity_x_of_point.values.push_back(pts_velocity[j].x);
        velocity_y_of_point.values.push_back(pts_velocity[j].y);
        std::cout << "Adding point " << p_id << " to feature cloud.";
      }
    }

    feature_points->channels.push_back(id_of_point);
    feature_points->channels.push_back(u_of_point);
    feature_points->channels.push_back(v_of_point);
    feature_points->channels.push_back(velocity_x_of_point);
    feature_points->channels.push_back(velocity_y_of_point);

    // get feature depth from lidar point cloud
    pcl::PointCloud<PointType>::Ptr depth_cloud_temp(
        new pcl::PointCloud<PointType>());
    mtx_lidar.lock();
    *depth_cloud_temp = *depthCloud;
    mtx_lidar.unlock();

    sensor_msgs::ChannelFloat32 depth_of_points = depth_register_.get_depth(
        img_msg->header.stamp, image, depth_cloud_temp, tracker.m_camera,
        feature_points->points);

    feature_points->channels.push_back(depth_of_points);

    if (tracker.isFlowAvailable()) {
      camera_pubs_.at(CAMS[cam_id]).publish(feature_points);
    } else {
      tracker.markFlowAsAvailable();
    }

  }

  void processLidar(const sensor_msgs::PointCloud2ConstPtr &laser_msg) {
    ROS_INFO("Processing lidar data...");
    static int lidar_count = -1;
    if (++lidar_count % (LIDAR_SKIP + 1) != 0) {
      std::cout << "skip lidar frame" << std::endl;
      return;
    }

    // 0. listen to transform
    static tf::TransformListener listener;
    static tf::StampedTransform transform_world_cFLU;
    static tf::StampedTransform transform_cFLU_imu;
    std::cout << "Waiting for transform..." << std::endl;
    try {
      listener.waitForTransform("vins_world", "vins_cameraFLU",
                                laser_msg->header.stamp, ros::Duration(0.01));
      listener.lookupTransform("vins_world", "vins_cameraFLU",
                               laser_msg->header.stamp, transform_world_cFLU);
      listener.waitForTransform("vins_cameraFLU", "vins_body_imuhz",
                                laser_msg->header.stamp, ros::Duration(0.01));
      listener.lookupTransform("vins_cameraFLU", "vins_body_imuhz",
                               laser_msg->header.stamp, transform_cFLU_imu);
    } catch (tf::TransformException ex) {
      ROS_ERROR("%s", ex.what());
      return;
    }

    double xCur, yCur, zCur, rollCur, pitchCur, yawCur;
    xCur = transform_world_cFLU.getOrigin().x();
    yCur = transform_world_cFLU.getOrigin().y();
    zCur = transform_world_cFLU.getOrigin().z();
    tf::Matrix3x3 m(transform_world_cFLU.getRotation());
    m.getRPY(rollCur, pitchCur, yawCur);
    Eigen::Affine3f transNow =
        pcl::getTransformation(xCur, yCur, zCur, rollCur, pitchCur, yawCur);

    std::cout << "transNow: " << transNow.matrix() << std::endl;

    // 1. convert laser cloud message to pcl
    pcl::PointCloud<PointType>::Ptr laser_cloud_in(
        new pcl::PointCloud<PointType>());
    pcl::fromROSMsg(*laser_msg, *laser_cloud_in);

    // 2. downsample new cloud (save memory)
    pcl::PointCloud<PointType>::Ptr laser_cloud_in_ds(
        new pcl::PointCloud<PointType>());
    static pcl::VoxelGrid<PointType> downSizeFilter;
    downSizeFilter.setLeafSize(0.2, 0.2, 0.2);
    downSizeFilter.setInputCloud(laser_cloud_in);
    downSizeFilter.filter(*laser_cloud_in_ds);
    *laser_cloud_in = *laser_cloud_in_ds;

    // 3.
    // 把lidar坐标系下的点云转到相机的FLU坐标系下表示，因为下一步需要使用相机FLU坐标系下的点云进行初步过滤
    pcl::PointCloud<PointType>::Ptr laser_cloud_offset(
        new pcl::PointCloud<PointType>());
    //; T_cFLU_lidar
    tf::Transform transform_cFLU_lidar =
        transform_cFLU_imu * Transform_imu_lidar;
    double roll, pitch, yaw, x, y, z;
    x = transform_cFLU_lidar.getOrigin().getX();
    y = transform_cFLU_lidar.getOrigin().getY();
    z = transform_cFLU_lidar.getOrigin().getZ();
    tf::Matrix3x3(transform_cFLU_lidar.getRotation()).getRPY(roll, pitch, yaw);
    Eigen::Affine3f transOffset =
        pcl::getTransformation(x, y, z, roll, pitch, yaw);
    //; lidar本体坐标系下的点云，转到相机FLU坐标系下表示
    pcl::transformPointCloud(*laser_cloud_in, *laser_cloud_offset, transOffset);
    *laser_cloud_in = *laser_cloud_offset;

    // 4. filter lidar points (only keep points in camera view)
    //; 根据已经转到相机FLU坐标系下的点云，先排除不在相机FoV内的点云
    pcl::PointCloud<PointType>::Ptr laser_cloud_in_filter(
        new pcl::PointCloud<PointType>());
    for (int i = 0; i < (int)laser_cloud_in->size(); ++i) {
      PointType p = laser_cloud_in->points[i];
      if (p.x >= 0 && abs(p.y / p.x) <= 10 && abs(p.z / p.x) <= 10)
        laser_cloud_in_filter->push_back(p);
    }
    *laser_cloud_in = *laser_cloud_in_filter;

    // 5. transform new cloud into global odom frame
    pcl::PointCloud<PointType>::Ptr laser_cloud_global(
        new pcl::PointCloud<PointType>());
    //; cameraFLU坐标系下的点云，转到vinsworld系下表示
    pcl::transformPointCloud(*laser_cloud_in, *laser_cloud_global, transNow);

    // 6. save new cloud
    double timeScanCur = laser_msg->header.stamp.toSec();
    cloudQueue.push_back(*laser_cloud_global);
    timeQueue.push_back(timeScanCur);

    // 7. pop old cloud
    while (!timeQueue.empty()) {
      if (timeScanCur - timeQueue.front() > 5.0) {
        cloudQueue.pop_front();
        timeQueue.pop_front();
      } else {
        break;
      }
    }

    std::lock_guard<std::mutex> lock(mtx_lidar);
    // 8. fuse global cloud
    depthCloud->clear();
    for (int i = 0; i < (int)cloudQueue.size(); ++i)
      *depthCloud += cloudQueue[i];

    // 9. downsample global cloud
    pcl::PointCloud<PointType>::Ptr depthCloudDS(
        new pcl::PointCloud<PointType>());
    downSizeFilter.setLeafSize(0.2, 0.2, 0.2);
    downSizeFilter.setInputCloud(depthCloud);
    downSizeFilter.filter(*depthCloudDS);
    *depthCloud = *depthCloudDS;
  }

private:
  std::unordered_map<std::string, FeatureTracker>
      camera_trackers_; // TODO: change to vector of trackers in order to avoid
                        // lookups
  DepthRegister depth_register_;
  std::unordered_map<std::string, ros::Publisher>
      camera_pubs_; // TODO: change to vector of publishers in order to avoid
                    // lookups
  ros::NodeHandle nh_;
  std::mutex mtx_lidar;
};

int main(int argc, char **argv) {
  ros::init(argc, argv, "vins");
  ros::NodeHandle n;
  ROS_INFO("\033[1;32m----> Visual Feature Tracker Started.\033[0m");
  ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME,
                                 ros::console::levels::Info);
  std::string calibration_file_path;
  n.getParam("vins_config_file", calibration_file_path);
  readParameters(n);

  // subscribers. use a message filter to software-synchronize the images and
  // lidar
  message_filters::Subscriber<sensor_msgs::Image> left_img_sub(
      n, LEFT_IMAGE_TOPIC, 5);
  message_filters::Subscriber<sensor_msgs::Image> right_img_sub(
      n, RIGHT_IMAGE_TOPIC, 5);
  message_filters::Subscriber<sensor_msgs::PointCloud2> lidar_sub(
      n, POINT_CLOUD_TOPIC, 5);

  // initializing the feature tracker
  FeatureTrackerWrapper feature_tracker_wrapper{calibration_file_path};

  // initializing message filter so we can get synchronized messages
  typedef message_filters::sync_policies::ApproximateTime<
      sensor_msgs::Image, sensor_msgs::Image, sensor_msgs::PointCloud2>
      MySyncPolicy;
  message_filters::Synchronizer<MySyncPolicy> sync(
      MySyncPolicy(10), left_img_sub, right_img_sub, lidar_sub);
  sync.registerCallback(boost::bind(&FeatureTrackerWrapper::processFrame,
                                    &feature_tracker_wrapper, _1, _2, _3));

  ros::spin();

  return 0;
}
