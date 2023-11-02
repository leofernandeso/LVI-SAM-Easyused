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

    // Performing tracking
    auto [timestamp, image] = frame->left_image();
    feature_tracker_.readImage(image, timestamp);

  }

private:
  FeatureTracker feature_tracker_;
};

/* void img_callback(const sensor_msgs::ImageConstPtr &img_msg) { */
/*   double cur_img_time = img_msg->header.stamp.toSec(); */

/*   cv::Mat image = ptr->image; */
/*   TicToc t_r; */
/*   std::vector<size_t> ids; */
/*   left_camera_tracker.readImage(image, cur_img_time); */
/*   auto current_left_pts = left_camera_tracker.cur_pts; */

/*   for (unsigned int i = 0;; i++) { */
/*     bool completed = false; */
/*     for (int j = 0; j < NUM_OF_CAM; j++) */
/*       if (j != 1 || !STEREO_TRACK) */
/*         completed |= left_camera_tracker.updateID(i); */
/*     if (!completed) */
/*       break; */
/*   } */

/*   if (PUB_THIS_FRAME) { */
/*     pub_count++; */
/*     sensor_msgs::PointCloudPtr feature_points(new sensor_msgs::PointCloud);
 */
/*     sensor_msgs::ChannelFloat32 id_of_point; */
/*     sensor_msgs::ChannelFloat32 u_of_point; */
/*     sensor_msgs::ChannelFloat32 v_of_point; */
/*     sensor_msgs::ChannelFloat32 velocity_x_of_point; */
/*     sensor_msgs::ChannelFloat32 velocity_y_of_point; */

/*     feature_points->header.stamp = img_msg->header.stamp; */
/*     feature_points->header.frame_id = "vins_body"; */

/*     vector<set<int>> hash_ids(NUM_OF_CAM); */
/*     for (int i = 0; i < NUM_OF_CAM; i++) { */
/*       auto &un_pts = left_camera_tracker.cur_un_pts; */
/*       auto &cur_pts = left_camera_tracker.cur_pts; */
/*       auto &ids = left_camera_tracker.ids; */
/*       ROS_WARN("ids size: %d", ids.size()); */
/*       auto &pts_velocity = left_camera_tracker.pts_velocity; */
/*       for (unsigned int j = 0; j < ids.size(); j++) { */
/*         if (left_camera_tracker.track_cnt[j] > 1) { */
/*           int p_id = ids[j]; */
/*           hash_ids[i].insert(p_id); */
/*           geometry_msgs::Point32 p; */
/*           p.x = un_pts[j].x; */
/*           p.y = un_pts[j].y; */
/*           p.z = 1; */

/*           feature_points->points.push_back(p); */
/*           id_of_point.values.push_back(p_id * NUM_OF_CAM + i); */
/*           u_of_point.values.push_back(cur_pts[j].x); */
/*           v_of_point.values.push_back(cur_pts[j].y); */
/*           velocity_x_of_point.values.push_back(pts_velocity[j].x); */
/*           velocity_y_of_point.values.push_back(pts_velocity[j].y); */
/*         } */
/*       } */
/*     } */

/*     feature_points->channels.push_back(id_of_point); */
/*     feature_points->channels.push_back(u_of_point); */
/*     feature_points->channels.push_back(v_of_point); */
/*     feature_points->channels.push_back(velocity_x_of_point); */
/*     feature_points->channels.push_back(velocity_y_of_point); */

/*     // get feature depth from lidar point cloud */
/*     pcl::PointCloud<PointType>::Ptr depth_cloud_temp( */
/*         new pcl::PointCloud<PointType>()); */
/*     mtx_lidar.lock(); */
/*     *depth_cloud_temp = *depthCloud; */
/*     mtx_lidar.unlock(); */

/*     std::cout << "Estimating depth for " << feature_points->points.size() */
/*               << " points" << std::endl; */
/*     sensor_msgs::ChannelFloat32 depth_of_points = depthRegister->get_depth(
 */
/*         img_msg->header.stamp, image, depth_cloud_temp, */
/*         left_camera_tracker.m_camera, feature_points->points); */
/*     feature_points->channels.push_back(depth_of_points); */

/*     // skip the first image; since no optical speed on frist image */
/*     if (!init_pub) { */
/*       init_pub = 1; */
/*     } else */
/*       pub_feature.publish(feature_points); */

/*     // publish features in image */
/*     if (pub_match.getNumSubscribers() != 0) { */
/*       ptr = cv_bridge::cvtColor(ptr, sensor_msgs::image_encodings::RGB8); */
/*       // cv::Mat stereo_img(ROW * NUM_OF_CAM, COL, CV_8UC3); */
/*       cv::Mat stereo_img = ptr->image; */

/*       for (int i = 0; i < NUM_OF_CAM; i++) { */
/*         cv::Mat tmp_img = stereo_img.rowRange(i * ROW, (i + 1) * ROW); */
/*         cv::cvtColor(image, tmp_img, CV_GRAY2RGB); */

/*         for (unsigned int j = 0; j < ids.size(); j++) { */
/*           if (SHOW_TRACK) { */
/*             // track count */
/*             double len = std::min(1.0, 1.0 * left_camera_tracker.track_cnt[j]
 * / */
/*                                            WINDOW_SIZE); */
/*             cv::circle(tmp_img, left_camera_tracker.cur_pts[j], 4, */
/*                        cv::Scalar(255 * (1 - len), 255 * len, 0), 4); */
/*           } else { */
/*             // depth */
/*             if (j < depth_of_points.values.size()) { */
/*               if (depth_of_points.values[j] > 0) */
/*                 cv::circle(tmp_img, left_camera_tracker.cur_pts[j], 4, */
/*                            cv::Scalar(0, 255, 0), 4); */
/*               else */
/*                 cv::circle(tmp_img, left_camera_tracker.cur_pts[j], 4, */
/*                            cv::Scalar(0, 0, 255), 4); */
/*             } */
/*           } */
/*         } */
/*       } */

/*       pub_match.publish(ptr->toImageMsg()); */
/*     } */
/*   } */
/* } */

/* void lidar_callback(const sensor_msgs::PointCloud2ConstPtr &laser_msg) { */
/*   static int lidar_count = -1; */
/*   if (++lidar_count % (LIDAR_SKIP + 1) != 0) */
/*     return; */

/*   // 0. listen to transform */
/*   static tf::TransformListener listener; */
/* #if IF_OFFICIAL */
/*   static tf::StampedTransform transform; //; T_vinsworld_camera_FLU */
/* #else */
/*   static tf::StampedTransform transform_world_cFLU; //;
 * T_vinsworld_camera_FLU */
/*   static tf::StampedTransform transform_cFLU_imu;   //; T_cameraFLU_imu */
/* #endif */
/*   try { */
/* #if IF_OFFICIAL */
/*     listener.waitForTransform("vins_world", "vins_body_ros", */
/*                               laser_msg->header.stamp, ros::Duration(0.01));
 */
/*     listener.lookupTransform("vins_world", "vins_body_ros", */
/*                              laser_msg->header.stamp, transform); */
/* #else */
/*     //? mod: 监听T_vinsworld_cameraFLU 和 T_cameraFLU_imu */
/*     listener.waitForTransform("vins_world", "vins_cameraFLU", */
/*                               laser_msg->header.stamp, ros::Duration(0.01));
 */
/*     listener.lookupTransform("vins_world", "vins_cameraFLU", */
/*                              laser_msg->header.stamp, transform_world_cFLU);
 */
/*     listener.waitForTransform("vins_cameraFLU", "vins_body_imuhz", */
/*                               laser_msg->header.stamp, ros::Duration(0.01));
 */
/*     listener.lookupTransform("vins_cameraFLU", "vins_body_imuhz", */
/*                              laser_msg->header.stamp, transform_cFLU_imu); */
/* #endif */
/*   } catch (tf::TransformException ex) { */
/*     // ROS_ERROR("lidar no tf"); */
/*     return; */
/*   } */

/*   double xCur, yCur, zCur, rollCur, pitchCur, yawCur; */
/* #if IF_OFFICIAL */
/*   xCur = transform.getOrigin().x(); */
/*   yCur = transform.getOrigin().y(); */
/*   zCur = transform.getOrigin().z(); */
/*   tf::Matrix3x3 m(transform.getRotation()); */
/* #else */
/*   xCur = transform_world_cFLU.getOrigin().x(); */
/*   yCur = transform_world_cFLU.getOrigin().y(); */
/*   zCur = transform_world_cFLU.getOrigin().z(); */
/*   tf::Matrix3x3 m(transform_world_cFLU.getRotation()); */
/* #endif */
/*   m.getRPY(rollCur, pitchCur, yawCur); */
/*   //; T_vinswolrd_cameraFLU */
/*   Eigen::Affine3f transNow = */
/*       pcl::getTransformation(xCur, yCur, zCur, rollCur, pitchCur, yawCur); */

/*   // 1. convert laser cloud message to pcl */
/*   pcl::PointCloud<PointType>::Ptr laser_cloud_in( */
/*       new pcl::PointCloud<PointType>()); */
/*   pcl::fromROSMsg(*laser_msg, *laser_cloud_in); */

/*   // 2. downsample new cloud (save memory) */
/*   pcl::PointCloud<PointType>::Ptr laser_cloud_in_ds( */
/*       new pcl::PointCloud<PointType>()); */
/*   static pcl::VoxelGrid<PointType> downSizeFilter; */
/*   downSizeFilter.setLeafSize(0.2, 0.2, 0.2); */
/*   downSizeFilter.setInputCloud(laser_cloud_in); */
/*   downSizeFilter.filter(*laser_cloud_in_ds); */
/*   *laser_cloud_in = *laser_cloud_in_ds; */

/*   // 3. */
/*   //
 * 把lidar坐标系下的点云转到相机的FLU坐标系下表示，因为下一步需要使用相机FLU坐标系下的点云进行初步过滤
 */
/* #if IF_OFFICIAL */
/*   pcl::PointCloud<PointType>::Ptr laser_cloud_offset( */
/*       new pcl::PointCloud<PointType>()); */
/*   Eigen::Affine3f transOffset = */
/*       pcl::getTransformation(L_C_TX, L_C_TY, L_C_TZ, L_C_RX, L_C_RY, L_C_RZ);
 */
/*   pcl::transformPointCloud(*laser_cloud_in, *laser_cloud_offset,
 * transOffset); */
/*   *laser_cloud_in = *laser_cloud_offset; */
/* #else */
/*   pcl::PointCloud<PointType>::Ptr laser_cloud_offset( */
/*       new pcl::PointCloud<PointType>()); */
/*   //; T_cFLU_lidar */
/*   tf::Transform transform_cFLU_lidar = transform_cFLU_imu *
 * Transform_imu_lidar; */
/*   double roll, pitch, yaw, x, y, z; */
/*   x = transform_cFLU_lidar.getOrigin().getX(); */
/*   y = transform_cFLU_lidar.getOrigin().getY(); */
/*   z = transform_cFLU_lidar.getOrigin().getZ(); */
/*   tf::Matrix3x3(transform_cFLU_lidar.getRotation()).getRPY(roll, pitch, yaw);
 */
/*   Eigen::Affine3f transOffset = */
/*       pcl::getTransformation(x, y, z, roll, pitch, yaw); */
/*   //; lidar本体坐标系下的点云，转到相机FLU坐标系下表示 */
/*   pcl::transformPointCloud(*laser_cloud_in, *laser_cloud_offset,
 * transOffset); */
/*   *laser_cloud_in = *laser_cloud_offset; */
/* #endif */

/*   // 4. filter lidar points (only keep points in camera view) */
/*   //; 根据已经转到相机FLU坐标系下的点云，先排除不在相机FoV内的点云 */
/*   pcl::PointCloud<PointType>::Ptr laser_cloud_in_filter( */
/*       new pcl::PointCloud<PointType>()); */
/*   for (int i = 0; i < (int)laser_cloud_in->size(); ++i) { */
/*     PointType p = laser_cloud_in->points[i]; */
/*     if (p.x >= 0 && abs(p.y / p.x) <= 10 && abs(p.z / p.x) <= 10) */
/*       laser_cloud_in_filter->push_back(p); */
/*   } */
/*   *laser_cloud_in = *laser_cloud_in_filter; */

/*   // 5. transform new cloud into global odom frame */
/*   pcl::PointCloud<PointType>::Ptr laser_cloud_global( */
/*       new pcl::PointCloud<PointType>()); */
/*   //; cameraFLU坐标系下的点云，转到vinsworld系下表示 */
/*   pcl::transformPointCloud(*laser_cloud_in, *laser_cloud_global, transNow);
 */

/*   // 6. save new cloud */
/*   double timeScanCur = laser_msg->header.stamp.toSec(); */
/*   cloudQueue.push_back(*laser_cloud_global); */
/*   timeQueue.push_back(timeScanCur); */

/*   // 7. pop old cloud */
/*   while (!timeQueue.empty()) { */
/*     if (timeScanCur - timeQueue.front() > 5.0) { */
/*       cloudQueue.pop_front(); */
/*       timeQueue.pop_front(); */
/*     } else { */
/*       break; */
/*     } */
/*   } */

/*   std::lock_guard<std::mutex> lock(mtx_lidar); */
/*   // 8. fuse global cloud */
/*   depthCloud->clear(); */
/*   for (int i = 0; i < (int)cloudQueue.size(); ++i) */
/*     *depthCloud += cloudQueue[i]; */

/*   // 9. downsample global cloud */
/*   pcl::PointCloud<PointType>::Ptr depthCloudDS( */
/*       new pcl::PointCloud<PointType>()); */
/*   downSizeFilter.setLeafSize(0.2, 0.2, 0.2); */
/*   downSizeFilter.setInputCloud(depthCloud); */
/*   downSizeFilter.filter(*depthCloudDS); */
/*   *depthCloud = *depthCloudDS; */
/* } */

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
