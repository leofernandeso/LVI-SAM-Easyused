#include <datasets/kitti.h>
#include <pcl/io/pcd_io.h>

#include <Eigen/Dense>
#include <boost/filesystem.hpp>
#include <fstream>
#include <iostream>
#include <map>
#include <opencv2/opencv.hpp>
#include <sstream>
#include <string>
#include <vector>

std::string intToStringWithLeadingZeros(size_t value) {
  std::ostringstream oss;
  oss << std::setw(10) << std::setfill('0') << value;
  return oss.str();
}

std::pair<std::string, Eigen::VectorXd> parseLine(const std::string &line) {
  size_t pos = line.find(": ");
  if (pos == std::string::npos) {
    throw std::runtime_error("Could not parse line: " + line);
  }

  std::string prefix = line.substr(0, pos);
  std::stringstream suffix = std::stringstream(line.substr(pos + 2));

  std::string key;
  std::vector<double> v;
  while (std::getline(suffix, key, ' ')) {
    v.push_back(std::stod(key));
  }
  Eigen::VectorXd vec{Eigen::VectorXd::Map(v.data(), v.size())};

  return std::make_pair(prefix, vec);
}

double convertTimestampToSeconds(const std::string &timestamp_str) {
  std::tm tm = {};
  std::istringstream ss(timestamp_str);
  ss >> std::get_time(&tm, "%Y-%m-%d %H:%M:%S");

  if (ss.fail()) {
    throw std::runtime_error("Failed to parse date and time.");
  }

  std::time_t time_seconds = std::mktime(&tm);
  double fractional_seconds = 0.0;

  auto pos = timestamp_str.find('.');
  if (pos != std::string::npos && pos + 1 < timestamp_str.size()) {
    std::string fractional_seconds_str = timestamp_str.substr(pos + 1);
    fractional_seconds = std::stod("0." + fractional_seconds_str);
  }

  return time_seconds + fractional_seconds;
}

std::vector<double> parseTimestampFile(const std::string &filepath) {
  std::ifstream timestamp_file{filepath};
  if (!timestamp_file) {
    throw std::runtime_error("Could not read " + filepath);
  }

  std::vector<double> timestamps;
  std::string line;
  while (std::getline(timestamp_file, line)) {
    timestamps.push_back(convertTimestampToSeconds(line));
  }

  timestamp_file.close();

  return timestamps;
}

Eigen::Vector4d rectifiedProjectionMatrixToPinholeCameraModelVector(
    const Eigen::MatrixXd &projectionMatrix) {
  return Eigen::Vector4d(projectionMatrix(0), projectionMatrix(2),
                         projectionMatrix(5), projectionMatrix(6));
}

KittiDatasetManager::KittiDatasetManager(const std::string &input_folder)
    : input_folder_(input_folder), dataset_size_(computeSize()) {
  parseCalibFiles();
  parseTimestamps();
}

size_t KittiDatasetManager::getDatasetSize() const { return dataset_size_; }

std::pair<double, pcl::PointCloud<pcl::PointXYZI>::Ptr>
KittiDatasetManager::readPointCloud(size_t scan_id) const {
  std::string scan_file = input_folder_ + "/velodyne_points/data/" +
                          intToStringWithLeadingZeros(scan_id) + ".bin";
  std::ifstream file{scan_file, std::ifstream::binary};
  if (!file) {
    throw std::runtime_error("Could not read " + scan_file);
  }

  file.seekg(0, std::ios::end);
  const size_t fileSize = file.tellg();
  file.seekg(0, std::ios::beg);

  const size_t numPoints = fileSize / 16;

  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_ptr(
      new pcl::PointCloud<pcl::PointXYZI>);
  cloud_ptr->resize(numPoints);

  PointRaw point;
  for (size_t i = 0;
       file.read(reinterpret_cast<char *>(&point), sizeof(PointRaw)); ++i) {
    pcl::PointXYZI p;
    p.x = point.x;
    p.y = point.y;
    p.z = point.z;
    p.intensity = point.intensity;
    cloud_ptr->points[i] = p;
  }
  file.close();

  return {timestamps_.at("velodyne")[scan_id], cloud_ptr};
}

std::pair<double, cv::Mat> KittiDatasetManager::readImage(size_t id, size_t camera_id) const {
  std::string image_file = input_folder_ + "image_0" +
                           std::to_string(camera_id) + "/data/" +
                           intToStringWithLeadingZeros(id) + ".png";
  cv::Mat image = cv::imread(image_file, cv::IMREAD_GRAYSCALE);
  return {timestamps_.at("cam" + std::to_string(camera_id))[id], image};
}

void KittiDatasetManager::setInputFolder(const std::string &input_folder) {
  input_folder_ = input_folder;
  dataset_size_ = computeSize();
  parseCalibFiles();
  parseTimestamps();
}

TransformationMap KittiDatasetManager::parseCalibFiles() {
  auto cam_to_cam = parseCalibFile(input_folder_ + "../calib_cam_to_cam.txt");
  auto imu_to_velo = parseCalibFile(input_folder_ + "../calib_imu_to_velo.txt",
                                    "imu_to_velo_");
  auto velo_to_cam = parseCalibFile(input_folder_ + "../calib_velo_to_cam.txt",
                                    "velo_to_cam_");

  calib_data_.insert(cam_to_cam.begin(), cam_to_cam.end());
  calib_data_.insert(imu_to_velo.begin(), imu_to_velo.end());
  calib_data_.insert(velo_to_cam.begin(), velo_to_cam.end());

  buildCamLidarExtrinsics();
  buildPinholeCamIntrinsics();

  return calib_data_;
}

void KittiDatasetManager::buildPinholeCamIntrinsics() {
  const std::array<std::string, 4> cam_names{"cam0", "cam1", "cam2", "cam3"};
  const std::array<std::string, 4> P_matrix_suffixes{"00", "01", "02", "03"};
  for (size_t i = 0; i < cam_names.size(); i++) {
    calib_data_["pinhole_" + cam_names[i]] =
        rectifiedProjectionMatrixToPinholeCameraModelVector(
            calib_data_.at("P_rect_" + P_matrix_suffixes[i]));
  }
}

void KittiDatasetManager::buildCamLidarExtrinsics() {
  // Velodyne to cam0
  Eigen::Matrix4d velo_to_cam0 = Eigen::Matrix4d::Identity();
  velo_to_cam0.block<3, 3>(0, 0) =
      Eigen::Map<Eigen::Matrix3d>(calib_data_.at("velo_to_cam_R").data())
          .transpose();
  velo_to_cam0.block<3, 1>(0, 3) = calib_data_.at("velo_to_cam_T");

  // cam1 to cam0
  Eigen::Matrix4d cam1_to_cam0 = Eigen::Matrix4d::Identity();
  cam1_to_cam0.block<3, 3>(0, 0) =
      Eigen::Map<Eigen::Matrix3d>(calib_data_.at("R_01").data());
  cam1_to_cam0.block<3, 1>(0, 3) = calib_data_.at("T_01");

  // Velodyne to cam1
  Eigen::Matrix4d velo_to_cam1 = cam1_to_cam0.inverse() * velo_to_cam0;

  // cam2 to cam0
  Eigen::Matrix4d cam2_to_cam0 = Eigen::Matrix4d::Identity();
  cam2_to_cam0.block<3, 3>(0, 0) =
      Eigen::Map<Eigen::Matrix3d>(calib_data_.at("R_02").data());
  cam2_to_cam0.block<3, 1>(0, 3) = calib_data_.at("T_02");

  // Velodyne to cam2
  Eigen::Matrix4d velo_to_cam2 = cam2_to_cam0.inverse() * velo_to_cam0;

  // cam3 to cam0
  Eigen::Matrix4d cam3_to_cam0 = Eigen::Matrix4d::Identity();
  cam3_to_cam0.block<3, 3>(0, 0) =
      Eigen::Map<Eigen::Matrix3d>(calib_data_.at("R_03").data());
  cam3_to_cam0.block<3, 1>(0, 3) = calib_data_.at("T_03");

  // Velodyne to cam3
  Eigen::Matrix4d velo_to_cam3 = cam3_to_cam0.inverse() * velo_to_cam0;

  calib_data_.insert({{"velo_to_cam0", velo_to_cam0},
                      {"velo_to_cam1", velo_to_cam1},
                      {"velo_to_cam2", velo_to_cam2},
                      {"velo_to_cam3", velo_to_cam3}});
}

TransformationMap
KittiDatasetManager::parseCalibFile(const std::string &filepath,
                                    std::string prefix) {
  TransformationMap calib_data;
  std::string calib_filepath{filepath};
  std::ifstream calib_file{calib_filepath};
  if (!calib_file) {
    throw std::runtime_error("Could not read " + calib_filepath);
  }

  std::string line;
  while (std::getline(calib_file, line)) {
    auto [key, vec] = parseLine(line);
    calib_data.emplace(prefix + key, vec);
  }

  calib_file.close();

  return calib_data;
}

TransformationMap KittiDatasetManager::getCalibData() const {
  return calib_data_;
}

std::vector<double>
KittiDatasetManager::getTimestamps(const std::string &sensor) const {
  auto it = timestamps_.find(sensor);
  if (it == timestamps_.end()) {
    throw std::runtime_error("Could not find timestamps for " + sensor);
  }
  return it->second;
}

void KittiDatasetManager::parseTimestamps() {
  timestamps_["cam0"] =
      parseTimestampFile(input_folder_ + "image_00/timestamps.txt");
  timestamps_["cam1"] =
      parseTimestampFile(input_folder_ + "image_01/timestamps.txt");
  timestamps_["cam2"] =
      parseTimestampFile(input_folder_ + "image_02/timestamps.txt");
  timestamps_["cam3"] =
      parseTimestampFile(input_folder_ + "image_03/timestamps.txt");
  timestamps_["velodyne"] =
      parseTimestampFile(input_folder_ + "velodyne_points/timestamps.txt");
  timestamps_["imu"] =
      parseTimestampFile(input_folder_ + "oxts/timestamps.txt");
}

size_t KittiDatasetManager::computeSize() const {
  boost::filesystem::path p(input_folder_ + "velodyne_points/data/");
  return std::distance(boost::filesystem::directory_iterator(p),
                       boost::filesystem::directory_iterator{});
}
