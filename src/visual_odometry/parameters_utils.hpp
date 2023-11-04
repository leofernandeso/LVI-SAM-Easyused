#include <Eigen/Dense>
#include <iostream>
#include <opencv2/core/eigen.hpp>
#include <opencv2/core/persistence.hpp>
#include <ros/package.h>
#include <ros/ros.h>
#include <tf/transform_datatypes.h>
#include <vector>

/* std::string PROJECT_NAME; */

/* double INIT_DEPTH; */

/* // store camera extrinsic parameters and image sizes */
/* std::vector<Eigen::Matrix3d> RIC; */
/* std::vector<Eigen::Vector3d> TIC; */
/* std::vector<cv::Size> IMAGE_SIZES; */

/* double BIAS_ACC_THRESHOLD; */
/* double BIAS_GYR_THRESHOLD; */
/* int ROLLING_SHUTTER; */
/* double TD, TR; */

/* int ALIGN_CAMERA_LIDAR_COORDINATE; */

/* Eigen::Matrix3d R_imu_lidar = Eigen::Matrix3d::Identity(); */
/* Eigen::Vector3d t_imu_lidar = Eigen::Vector3d::Zero(); */

/* std::string FISHEYE_MASK; */
/* int FREQ; */
/* int SHOW_TRACK; */
/* bool PUB_THIS_FRAME; */

/* tf::Transform Transform_imu_lidar; */

struct CameraOptions {
  std::string cam_name;
  std::string image_topic;
  std::string fisheye_mask_path;
  int image_width;
  int image_height;
  bool is_fisheye;
  bool is_rolling_shutter;

  void print() {
    std::cout << "CameraOptions: " << std::endl;
    std::cout << "  cam_name: " << cam_name << std::endl;
    std::cout << "  image_topic: " << image_topic << std::endl;
    std::cout << "  image_width: " << image_width << std::endl;
    std::cout << "  image_height: " << image_height << std::endl;
    std::cout << "  is_fisheye: " << is_fisheye << std::endl;
    std::cout << "  is_rolling_shutter: " << is_rolling_shutter << std::endl;
  }
};

struct LidarOptions {
  std::string lidar_topic;
  int skip;
  bool use;

  void print() {
    std::cout << "LidarOptions: " << std::endl;
    std::cout << "  lidar_topic: " << lidar_topic << std::endl;
    std::cout << "  skip: " << skip << std::endl;
    std::cout << "  use: " << use << std::endl;
  }
};

struct FeatureTrackingOptions {
  double f_threshold;
  int max_cnt;
  int min_dist;
  bool equalize;

  void print() {
    std::cout << "FeatureTrackingOptions: " << std::endl;
    std::cout << "  f_threshold: " << f_threshold << std::endl;
    std::cout << "  max_cnt: " << max_cnt << std::endl;
    std::cout << "  min_dist: " << min_dist << std::endl;
    std::cout << "  equalize: " << equalize << std::endl;
  }
};

struct OptimizationOptions {
  double max_solver_time;
  double keyframe_parallax; // ??
  int window_size;
  int max_num_iterations;
  int focal_length; // ??
  bool estimate_td;

  void print() {
    std::cout << "OptimizationOptions: " << std::endl;
    std::cout << "  max_solver_time: " << max_solver_time << std::endl;
    std::cout << "  keyframe_parallax: " << keyframe_parallax << std::endl;
    std::cout << "  window_size: " << window_size << std::endl;
    std::cout << "  max_num_iterations: " << max_num_iterations << std::endl;
    std::cout << "  focal_length: " << focal_length << std::endl;
    std::cout << "  estimate_td: " << estimate_td << std::endl;
  }
};

struct IMUParams {
  std::string imu_topic;
  Eigen::Vector3d g{0.0, 0.0, 9.8};
  double acc_n;
  double acc_w;
  double gyr_n;
  double gyr_w;

  void print() {
    std::cout << "IMUParams: " << std::endl;
    std::cout << "  imu_topic: " << imu_topic << std::endl;
    std::cout << "  g: " << g.transpose() << std::endl;
    std::cout << "  acc_n: " << acc_n << std::endl;
    std::cout << "  acc_w: " << acc_w << std::endl;
    std::cout << "  gyr_n: " << gyr_n << std::endl;
    std::cout << "  gyr_w: " << gyr_w << std::endl;
  }
};

struct Options {
  CameraOptions camera_options;
  LidarOptions lidar_options;
  FeatureTrackingOptions feature_tracking_options;
  OptimizationOptions optimization_options;
  IMUParams imu_params;

  void print() {
    std::cout << "Options: " << std::endl;
    camera_options.print();
    lidar_options.print();
    feature_tracking_options.print();
    optimization_options.print();
    imu_params.print();
  }
};

std::vector<std::string>
readCamNamesFromYamlFile(const cv::FileStorage &settings) {
  std::vector<std::string> cam_names;
  const auto root = settings.root();
  for (const auto &e : root) {
    if (e.name().rfind("cam", 0) == 0) {
      cam_names.push_back(e.name());
    }
  }
  return cam_names;
}

Options loadInputParameters(ros::NodeHandle &n) {
  CameraOptions camera_options;
  LidarOptions lidar_options;
  FeatureTrackingOptions feature_tracking_options;
  OptimizationOptions optimization_options;
  IMUParams imu_params;

  std::string config_file;
  n.getParam("vins_config_file", config_file);
  cv::FileStorage fsSettings(config_file, cv::FileStorage::READ);
  if (!fsSettings.isOpened()) {
    throw std::runtime_error("ERROR: Wrong path to settings");
  }

  /* // Common parameters */
  /* fsSettings["project_name"] >> PROJECT_NAME; */
  /* std::string pkg_path = ros::package::getPath(PROJECT_NAME); */

  // LIDAR options
  fsSettings["point_cloud_topic"] >> lidar_options.lidar_topic;
  fsSettings["lidar_skip"] >> lidar_options.skip;
  fsSettings["use_lidar"] >> lidar_options.use;

  // IMU options
  fsSettings["imu_topic"] >> imu_params.imu_topic;
  fsSettings["acc_n"] >> imu_params.acc_n;
  fsSettings["acc_w"] >> imu_params.acc_w;
  fsSettings["gyr_n"] >> imu_params.gyr_n;
  fsSettings["gyr_w"] >> imu_params.gyr_w;
  fsSettings["g_norm"] >> imu_params.g.z();

  // feature and image settings
  fsSettings["max_cnt"] >> feature_tracking_options.max_cnt;
  fsSettings["min_dist"] >> feature_tracking_options.min_dist;
  fsSettings["F_threshold"] >> feature_tracking_options.f_threshold;
  fsSettings["equalize"] >> feature_tracking_options.equalize;

  // Optimization options
  fsSettings["max_solver_time"] >> optimization_options.max_solver_time;
  fsSettings["keyframe_parallax"] >> optimization_options.keyframe_parallax;
  fsSettings["max_num_iterations"] >> optimization_options.max_num_iterations;
  fsSettings["estimate_td"] >> optimization_options.estimate_td;
  optimization_options.window_size = 20;
  optimization_options.focal_length = 460; // TODO: better understand this
  optimization_options.keyframe_parallax =
      optimization_options.keyframe_parallax /
      optimization_options.focal_length;

  // Camera Options
  auto cam_names = readCamNamesFromYamlFile(fsSettings);
  ROS_INFO_STREAM("Found " << cam_names.size() << " cameras in config file");

  return Options{CameraOptions{}, lidar_options, feature_tracking_options,
                 optimization_options, imu_params};

  /* // Others - TODO: understand these */
  /* fsSettings["freq"] >> FREQ; */
  /* if (FREQ == 0) */
  /*     FREQ = 100; */

  /* fsSettings["show_track"] >> SHOW_TRACK; */
  /* fsSettings["align_camera_lidar_estimation"] >>
   * ALIGN_CAMERA_LIDAR_COORDINATE; */
  /* INIT_DEPTH = 5.0; */
  /* BIAS_ACC_THRESHOLD = 0.1; */
  /* BIAS_GYR_THRESHOLD = 0.1; */

  /* auto cam_names = getCamNamesFromYamlFile(fsSettings); */
  /* ROS_INFO_STREAM("Found " << cam_names.size() << " cameras in config file");
   */

  /* for (const auto& cam_name : cam_names) { */
  /*   auto cam_settings = fsSettings[cam_name]; */
  /*   int width, height; */
  /*   cv::Mat cv_R, cv_T; */
  /*   cam_settings["extrinsicRotation"] >> cv_R; */
  /*   cam_settings["extrinsicTranslation"] >> cv_T; */
  /*   cam_settings["image_width"] >> width; */
  /*   cam_settings["image_height"] >> height; */
  /*   Eigen::Matrix3d eigen_R; */
  /*   Eigen::Vector3d eigen_T; */
  /*   cv::cv2eigen(cv_R, eigen_R); */
  /*   cv::cv2eigen(cv_T, eigen_T); */
  /*   Eigen::Quaterniond Q(eigen_R); */
  /*   eigen_R = Q.normalized(); */
  /*   RIC.push_back(eigen_R); */
  /*   TIC.push_back(eigen_T); */
  /*   IMAGE_SIZES.push_back(cv::Size(width, height)); */
  /* } */

  /* // write first camera images sizes as a temporary hack, since other modules
   * depend on this */
  /* ROW = IMAGE_SIZES[0].height; */
  /* COL = IMAGE_SIZES[0].width; */
  /* ROS_INFO_STREAM("ROW: " << ROW << " COL: " << COL); */

  /* for (size_t i = 0; i < RIC.size(); i++) */
  /* { */
  /*     ROS_INFO_STREAM(cam_names[i] << " - Extrinsic_R : " << std::endl <<
   * RIC[i]); */
  /*     ROS_INFO_STREAM(cam_names[i] << " - Extrinsic_T : " << std::endl <<
   * TIC[i].transpose()); */
  /* } */

  /* ROLLING_SHUTTER = fsSettings["rolling_shutter"]; */
  /* if (ROLLING_SHUTTER) */
  /* { */
  /*     TR = fsSettings["rolling_shutter_tr"]; */
  /*     ROS_INFO_STREAM("rolling shutter camera, read out time per line: " <<
   * TR); */
  /* } */
  /* else */
  /* { */
  /*     TR = 0; */
  /* } */

  /* // Common parameters for Module 1 and 2 */
  /* std::vector<double> t_imu_lidar_V; */
  /* std::vector<double> R_imu_lidar_V; */
  /* n.param<std::vector<double>>(PROJECT_NAME + "/extrinsicTranslation",
   * t_imu_lidar_V, std::vector<double>()); */
  /* n.param<std::vector<double>>(PROJECT_NAME + "/extrinsicRotation",
   * R_imu_lidar_V, std::vector<double>()); */
  /* Eigen::Vector3d t_imu_lidar = Eigen::Map<const Eigen::Matrix<double, -1,
   * -1, Eigen::RowMajor>>(t_imu_lidar_V.data(), 3, 1); */
  /* Eigen::Matrix3d R_tmp = Eigen::Map<const Eigen::Matrix<double, -1, -1,
   * Eigen::RowMajor>>(R_imu_lidar_V.data(), 3, 3); */
  /* ROS_ASSERT(abs(R_tmp.determinant()) > 0.9);   //
   * 防止配置文件中写错，这里加一个断言判断一下 */
  /* Eigen::Quaterniond Q_imu_lidar = Eigen::Quaterniond(R_tmp).normalized(); */
  /* Eigen::Matrix3d R_imu_lidar = Q_imu_lidar.toRotationMatrix(); */

  /* Transform_imu_lidar = tf::Transform(tf::Quaternion(Q_imu_lidar.x(),
   * Q_imu_lidar.y(), Q_imu_lidar.z(), Q_imu_lidar.w()), */
  /*                                     tf::Vector3(t_imu_lidar(0),
   * t_imu_lidar(1), t_imu_lidar(2))); */

  /* ROS_WARN_STREAM("=vins-feature_tracker/estimator read R_imu_lidar :
   * ====================="); */
  /* std::cout << R_imu_lidar << std::endl; */
  /* ROS_WARN_STREAM("=vins-feature_tracker/estimator read t_imu_lidar :
   * ====================="); */
  /* std::cout << t_imu_lidar.transpose() << std::endl; */

  /* usleep(100); */
  /* fsSettings.release(); */
}
