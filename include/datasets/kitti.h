#pragma once

#include <pcl/io/pcd_io.h>

#include <ctime>
#include <Eigen/Dense>
#include <map>
#include <opencv2/opencv.hpp>
#include <string>

using TransformationMap = std::map<std::string, Eigen::MatrixXd>;

struct PointRaw {
  float x;
  float y;
  float z;
  float intensity;
};

class KittiDatasetManager {
 public:
  KittiDatasetManager(const std::string &input_folder);

  size_t getDatasetSize() const;
  std::pair<double, pcl::PointCloud<pcl::PointXYZI>::Ptr> readPointCloud(size_t scan_id) const;
  std::pair<double, cv::Mat> readImage(size_t id, size_t camera_id) const;
  void setInputFolder(const std::string &input_folder);
  TransformationMap parseCalibFiles();
  TransformationMap getCalibData() const;
  std::vector<double> getTimestamps(const std::string &sensor) const;

 private:
  void buildPinholeCamIntrinsics();
  void buildCamLidarExtrinsics();
  TransformationMap parseCalibFile(const std::string &filepath, std::string prefix = "");
  void parseTimestamps();
  size_t computeSize() const;

  TransformationMap calib_data_;
  std::map<std::string, std::vector<double>> timestamps_;
  std::string input_folder_;
  size_t dataset_size_;
};
