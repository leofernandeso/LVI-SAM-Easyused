#include "parameters_utils.h"

void CameraOptions::print() const {
    std::cout << "CameraOptions: " << std::endl;
    std::cout << "  cam_name: " << cam_name << std::endl;
    std::cout << "  image_topic: " << image_topic << std::endl;
    std::cout << "  image_width: " << image_width << std::endl;
    std::cout << "  image_height: " << image_height << std::endl;
    std::cout << "  model_type: " << model_type << std::endl;
    std::cout << "  is_fisheye: " << is_fisheye << std::endl;
    std::cout << "  is_rolling_shutter: " << is_rolling_shutter << std::endl;
    std::cout << "  fisheye_mask_path: " << fisheye_mask_path << std::endl;
    std::cout << "  rolling_shutter_readout_time: "
              << rolling_shutter_readout_time << std::endl;
}

void LidarOptions::print() const {
    std::cout << "LidarOptions: " << std::endl;
    std::cout << "  lidar_topic: " << lidar_topic << std::endl;
    std::cout << "  skip: " << skip << std::endl;
    std::cout << "  use: " << use << std::endl;
    std::cout << "  R: " << std::endl << R << std::endl;
    std::cout << "  t: " << t.transpose() << std::endl;
}

void FeatureTrackingOptions::print() const {
    std::cout << "FeatureTrackingOptions: " << std::endl;
    std::cout << "  f_threshold: " << f_threshold << std::endl;
    std::cout << "  max_cnt: " << max_cnt << std::endl;
    std::cout << "  min_dist: " << min_dist << std::endl;
    std::cout << "  equalize: " << equalize << std::endl;
}

void EstimatorOptions::print() const {
    std::cout << "EstimatorOptions: " << std::endl;
    std::cout << "  max_solver_time: " << max_solver_time << std::endl;
    std::cout << "  keyframe_parallax: " << keyframe_parallax << std::endl;
    std::cout << "  window_size: " << window_size << std::endl;
    std::cout << "  max_num_iterations: " << max_num_iterations << std::endl;
    std::cout << "  focal_length: " << focal_length << std::endl;
    std::cout << "  estimate_td: " << estimate_td << std::endl;
}

IMUParams IMUParams::fromParameterServer(const std::string& prefix) {
    IMUParams imu_params;
    ros::NodeHandle nh;
    
    // Read IMU parameters from parameter server
    if (!nh.getParam(prefix + "/topic", imu_params.imu_topic) ||
        !nh.getParam(prefix + "/acc_n", imu_params.acc_n) ||
        !nh.getParam(prefix + "/acc_w", imu_params.acc_w) ||
        !nh.getParam(prefix + "/gyr_n", imu_params.gyr_n) ||
        !nh.getParam(prefix + "/gyr_w", imu_params.gyr_w)) {
        ROS_ERROR("Failed to load IMU parameters from parameter server.");
        return imu_params;
    }
    
    // Read gravity norm and update gravity vector
    double g_norm;
    if (nh.getParam(prefix + "/g_norm", g_norm)) {
        imu_params.g = Eigen::Vector3d(0.0, 0.0, g_norm);
    }
    
    return imu_params;
}

void IMUParams::print() const {
    std::cout << "IMUParams: " << std::endl;
    std::cout << "  imu_topic: " << imu_topic << std::endl;
    std::cout << "  g: " << g.transpose() << std::endl;
    std::cout << "  acc_n: " << acc_n << std::endl;
    std::cout << "  acc_w: " << acc_w << std::endl;
    std::cout << "  gyr_n: " << gyr_n << std::endl;
    std::cout << "  gyr_w: " << gyr_w << std::endl;
}

void Options::print() const {
    for (const auto &cam : per_camera_options) {
        cam.print();
    }
    lidar_options.print();
    feature_tracking_options.print();
    estimator_options.print();
    imu_params.print();
}

Options loadInputParameters(ros::NodeHandle &n) {

  LidarOptions lidar_options;
  FeatureTrackingOptions feature_tracking_options;
  EstimatorOptions estimator_options;
  IMUParams imu_params;

  std::string ns{"lvi_sam"};

  // LIDAR options
  n.getParam(ns + "/lidar_topic", lidar_options.lidar_topic);
  n.getParam(ns + "/lidar_skip", lidar_options.skip);
  n.getParam(ns + "/use_lidar", lidar_options.use);

  // Lidar extrinsics w.r.t. IMU
  // we need to read the project name first since lidar parameters are loaded to the ROS param server
  std::string project_name{"lvi_sam"};
  std::string pkg_path = ros::package::getPath(project_name);
  std::vector<double> t_imu_lidar_V;
  std::vector<double> R_imu_lidar_V;
  n.param<std::vector<double>>(project_name + "/extrinsicTranslation", t_imu_lidar_V, std::vector<double>());
  n.param<std::vector<double>>(project_name + "/extrinsicRotation", R_imu_lidar_V, std::vector<double>());
  Eigen::Vector3d t_imu_lidar{t_imu_lidar_V[0], t_imu_lidar_V[1], t_imu_lidar_V[2]};
  Eigen::Matrix3d R_tmp = Eigen::Map<Eigen::Matrix<double, 3, 3, Eigen::RowMajor>>(R_imu_lidar_V.data());
  Eigen::Quaterniond Q_imu_lidar = Eigen::Quaterniond(R_tmp).normalized();
  Eigen::Matrix3d R_imu_lidar = Q_imu_lidar.toRotationMatrix();
  lidar_options.R = R_imu_lidar;
  lidar_options.t = t_imu_lidar;

  lidar_options.print();

  // IMU options
  imu_params = IMUParams::fromParameterServer(ns + "/imu");
  imu_params.print();


  // feature and image settings
  n.getParam(ns + "/max_cnt", feature_tracking_options.max_cnt);
  n.getParam(ns + "/min_dist", feature_tracking_options.min_dist);
  n.getParam(ns + "/F_threshold", feature_tracking_options.f_threshold);
  n.getParam(ns + "/equalize", feature_tracking_options.equalize);
  feature_tracking_options.print();

  /* // Optimization options */
  n.getParam(ns + "/max_solver_time", estimator_options.max_solver_time);
  n.getParam(ns + "/keyframe_parallax", estimator_options.keyframe_parallax);
  n.getParam(ns + "/max_num_iterations", estimator_options.max_num_iterations);
  n.getParam(ns + "/estimate_td", estimator_options.estimate_td);
  estimator_options.window_size = 20;
  estimator_options.focal_length = 460; // TODO: better understand this
  estimator_options.keyframe_parallax =
      estimator_options.keyframe_parallax /
      estimator_options.focal_length;

  estimator_options.print();

  // Camera Options
  std::vector<CameraOptions> per_camera_options;
  std::vector<std::string> cams;
  std::string cams_base_folder;
  n.getParam(ns + "/cams", cams);
  n.getParam("/cam_configs_dir", cams_base_folder);
  for (const auto& cam : cams) {

    std::string cam_file{cams_base_folder + "/" + cam + ".yaml"};
    cv::FileStorage settings_file(cam_file, cv::FileStorage::READ);
    ROS_INFO("Loading camera parameters from %s", cam_file.c_str());
    if (!settings_file.isOpened()) {
      throw std::runtime_error("Could not open camera settings file: " + cam_file);
    }

    CameraOptions cam_options;
    cam_options.cam_name = cam;
    settings_file["image_topic"] >> cam_options.image_topic;
    settings_file["image_width"] >> cam_options.image_width;
    settings_file["image_height"] >> cam_options.image_height;
    settings_file["model_type"] >> cam_options.model_type;
    settings_file["fisheye"] >> cam_options.is_fisheye;
    settings_file["rolling_shutter"] >> cam_options.is_rolling_shutter;
    settings_file["fisheye_mask_path"] >> cam_options.fisheye_mask_path;
    settings_file["rolling_shutter_tr"] >> cam_options.rolling_shutter_readout_time;
    
    cv::Mat cv_R, cv_T;
    settings_file["extrinsicRotation"] >> cv_R;
    settings_file["extrinsicTranslation"] >> cv_T;
    Eigen::Matrix3d eigen_R;
    Eigen::Vector3d eigen_T;
    cv::cv2eigen(cv_R, eigen_R);
    cv::cv2eigen(cv_T, eigen_T);
    Eigen::Quaterniond Q(eigen_R);
    eigen_R = Q.normalized();
    cam_options.R = eigen_R;
    cam_options.t = eigen_T;

    per_camera_options.push_back(cam_options);

    settings_file.release();


  }

  for (const auto& cam : per_camera_options) {
    cam.print();
  }


  return Options{per_camera_options, lidar_options,
                 feature_tracking_options, estimator_options, imu_params};

}


