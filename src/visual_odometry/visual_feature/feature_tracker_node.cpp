#include <future>
#include <ros/ros.h>

#include "datasets/kitti.h"
#include "feature_tracker.h"
#include "parameters.h"
#include "sensor_msgs/Image.h"

using StampedImage = std::pair<double, cv::Mat>;
using StampedCloud = std::pair<double, pcl::PointCloud<pcl::PointXYZI>::Ptr>;

class DataFrame {
public:
  typedef std::shared_ptr<DataFrame> Ptr;

  DataFrame(const StampedImage &left_image, const StampedImage &right_image,
            const StampedCloud &lidar_cloud)
      : left_image_(left_image), right_image_(right_image),
        lidar_cloud_(lidar_cloud) {}

  StampedImage left_image() const { return left_image_; }
  StampedImage right_image() const { return right_image_; }
  StampedCloud lidar_cloud() const { return lidar_cloud_; }

  static DataFrame::Ptr create(const StampedImage &left_image,
                               const StampedImage &right_image,
                               const StampedCloud &lidar_cloud) {
    return std::make_shared<DataFrame>(left_image, right_image, lidar_cloud);
  }

private:
  StampedImage left_image_;
  StampedImage right_image_;
  StampedCloud lidar_cloud_;
};

class FeatureTrackerWrapper {
public:
  FeatureTrackerWrapper(const std::string &calib_file_path) {
    feature_tracker_.readIntrinsicParameter(calib_file_path);
  }

  void processFrame(const DataFrame::Ptr &frame) {

    // using only left image + lidar for now
    auto left_img_future =
        std::async(std::launch::async, &FeatureTrackerWrapper::processImage,
                   this, frame->left_image(), 0);
    auto lidar_future =
        std::async(std::launch::async, &FeatureTrackerWrapper::processLidarScan,
                   this, frame->lidar_cloud());

    // blocks until processing is done
    left_img_future.wait();
    lidar_future.wait();
  }

  void processImage(const StampedImage &stamped_image, size_t cam_id) {

    ROS_INFO("Processing image from cam %zu", cam_id);

    // Performing tracking
    const auto [timestamp, image] = stamped_image;
    /* auto [timestamp, image] = frame->left_image(); */
    feature_tracker_.readImage(image, timestamp);

    // Not sure yet what this is doing
    for (unsigned int i = 0;; i++) {
      bool completed = false;
      for (int j = 0; j < NUM_OF_CAM; j++)
        if (j != 1 || !STEREO_TRACK)
          completed |= feature_tracker_.updateID(i);
      if (!completed)
        break;
    }

    sensor_msgs::PointCloudPtr feature_points(new sensor_msgs::PointCloud);
    sensor_msgs::ChannelFloat32 id_of_point;
    sensor_msgs::ChannelFloat32 u_of_point;
    sensor_msgs::ChannelFloat32 v_of_point;
    sensor_msgs::ChannelFloat32 velocity_x_of_point;
    sensor_msgs::ChannelFloat32 velocity_y_of_point;

    feature_points->header.stamp = ros::Time(timestamp);
    feature_points->header.frame_id = "vins_body";

    auto &un_pts = feature_tracker_.cur_un_pts;
    auto &cur_pts = feature_tracker_.cur_pts;
    auto &ids = feature_tracker_.ids;
    auto &pts_velocity = feature_tracker_.pts_velocity;
    for (unsigned int j = 0; j < ids.size(); j++) {
      if (feature_tracker_.track_cnt[j] > 1) {
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
      }
    }

    feature_points->channels.push_back(id_of_point);
    feature_points->channels.push_back(u_of_point);
    feature_points->channels.push_back(v_of_point);
    feature_points->channels.push_back(velocity_x_of_point);
    feature_points->channels.push_back(velocity_y_of_point);

    /* // get feature depth from lidar point cloud */
    pcl::PointCloud<PointType>::Ptr depth_cloud_temp(
        new pcl::PointCloud<PointType>());
    /* mtx_lidar_.lock(); */
    /* *depth_cloud_temp = *depthCloud; */
    /* mtx_lidar_.unlock(); */

    /* sensor_msgs::ChannelFloat32 depth_of_points = depthRegister->get_depth(
     */
    /*     img_msg->header.stamp, show_img, depth_cloud_temp, */
    /*     trackerData[0].m_camera, feature_points->points); */
    /* feature_points->channels.push_back(depth_of_points); */
  }

  void processLidarScan(const StampedCloud &stamped_cloud) {

    static int lidar_count = 0;
    if (++lidar_count % (LIDAR_SKIP + 1) != 0)
      return;

    ROS_INFO("Processing lidar scan");
    auto [timestamp, cloud] = stamped_cloud;

    // 0. listen to transform
    static tf::TransformListener listener;
    static tf::StampedTransform transform_world_cFLU; //; T_vinsworld_camera_FLU
    static tf::StampedTransform transform_cFLU_imu;   //; T_cameraFLU_imu
    ros::Time time_laser{timestamp};
    try {
      listener.waitForTransform("vins_world", "vins_cameraFLU",
                                time_laser, ros::Duration(0.01));
      listener.lookupTransform("vins_world", "vins_cameraFLU",
                               time_laser, transform_world_cFLU);
      listener.waitForTransform("vins_cameraFLU", "vins_body_imuhz",
                                time_laser, ros::Duration(0.01));
      listener.lookupTransform("vins_cameraFLU", "vins_body_imuhz",
                               time_laser, transform_cFLU_imu);
    } catch (tf::TransformException ex) {
      ROS_ERROR("%s", ex.what());
      return;
    }

  }

private:
  FeatureTracker feature_tracker_;
  std::mutex mtx_lidar_;
  size_t cam_id_;
};

void showFrame(const cv::Mat &left, const cv::Mat &right) {
  cv::imshow("left", left);
  cv::imshow("right", right);
  cv::waitKey(0);
}

int main(int argc, char **argv) {

  ros::init(argc, argv, "feature_tracker");
  ros::NodeHandle nh;

  // Fetching config file from parameter server
  std::string config_file_path;
  nh.getParam("vins_config_file", config_file_path);
  readParameters(nh);

  KittiDatasetManager dataset_manager{RAW_SEQUENCE_PATH};

  FeatureTrackerWrapper feature_tracker_wrapper{config_file_path};

  DataFrame::Ptr frame = DataFrame::create(dataset_manager.readImage(0, 0),
                                           dataset_manager.readImage(0, 1),
                                           dataset_manager.readPointCloud(0));

  for (size_t i = 1; i < dataset_manager.getDatasetSize(); ++i) {
    frame = DataFrame::create(dataset_manager.readImage(i, 0),
                              dataset_manager.readImage(i, 1),
                              dataset_manager.readPointCloud(i));
    feature_tracker_wrapper.processFrame(frame);
  }

  ros::spin();

  return 0;
}
