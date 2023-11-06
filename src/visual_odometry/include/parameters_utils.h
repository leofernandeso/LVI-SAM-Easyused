#pragma once

#include <Eigen/Dense>
#include <iostream>
#include <opencv2/core/eigen.hpp>
#include <opencv2/core/persistence.hpp>
#include <ros/package.h>
#include <ros/ros.h>
#include <tf/transform_datatypes.h>
#include <vector>

struct CameraOptions {
  // Camera pose in IMU frame
  Eigen::Matrix3d R;
  Eigen::Vector3d t;
  std::string cam_name;
  std::string image_topic;
  std::string fisheye_mask_path;
  std::string model_type;
  double rolling_shutter_readout_time;
  int image_width;
  int image_height;
  int is_fisheye;
  int is_rolling_shutter;

  void print() const;
};

struct LidarOptions {
  // Lidar pose in IMU frame
  Eigen::Matrix3d R;
  Eigen::Vector3d t;
  std::string lidar_topic;
  int skip;
  int use;

  void print() const;
};

struct FeatureTrackingOptions {
  double f_threshold;
  int max_cnt;
  int min_dist;
  int equalize;

  void print() const;
};

struct EstimatorOptions {
  double max_solver_time;
  double keyframe_parallax; // ??
  int window_size;
  int max_num_iterations;
  int focal_length; // ??
  int estimate_td;

  void print() const;
};

struct IMUParams {
  std::string imu_topic;
  Eigen::Vector3d g{0.0, 0.0, 9.8};
  double acc_n;
  double acc_w;
  double gyr_n;
  double gyr_w;

  static IMUParams fromParameterServer(const std::string& prefix);
  void print() const;
};

struct Options {
  std::vector<CameraOptions> per_camera_options;
  LidarOptions lidar_options;
  FeatureTrackingOptions feature_tracking_options;
  EstimatorOptions estimator_options;
  IMUParams imu_params;

  void print() const;
};

Options loadInputParameters(ros::NodeHandle &n);
