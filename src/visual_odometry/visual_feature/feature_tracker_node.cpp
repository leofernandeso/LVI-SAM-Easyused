#include "camera_models/Camera.h"
#include "camera_models/PinholeCamera.h"
#include "feature_tracker.h"
#include <opencv2/core/eigen.hpp>
#include "sensor_msgs/ChannelFloat32.h"
#include "sensor_msgs/Image.h"
#include "sensor_msgs/image_encodings.h"

#define SHOW_UNDISTORTION 0


const std::string DENSE_DEPTH_MAP_TOPIC = "/dense_depth_map";

// mtx lock for two threads
std::mutex lidar_mtx;

// global variable for saving the point cloud
pcl::PointCloud<PointType>::Ptr point_cloud_ptr(new pcl::PointCloud<PointType>());

// global variables saving the lidar point cloud
deque<pcl::PointCloud<PointType>> cloudQueue;
deque<double> timeQueue;

// feature publisher for VINS estimator
ros::Publisher pub_feature;
ros::Publisher pub_match;
ros::Publisher pub_restart;
ros::Publisher depth_map_pub;

// feature tracker variables
FeatureTracker trackerData[NUM_OF_CAM];
double first_image_time;
int pub_count = 1;
bool first_image_flag = true;
double last_image_time = 0;
bool init_pub = 0;

cv_bridge::CvImagePtr right_img_ptr;
cv_bridge::CvImagePtr left_img_ptr;
cv::Mat dense_depth_map;

Eigen::Matrix4d T_cam_lidar;

cv::Mat getDepthMapFromLidarScan(const pcl::PointCloud<PointType>::Ptr& points, const cv::Mat& image, const Eigen::Matrix4d& T, const CameraPtr& camera_ptr) {

  auto height = camera_ptr->imageHeight();
  auto width = camera_ptr->imageWidth();

  // project points to image
  lidar_mtx.lock();
  pcl::PointCloud<PointType>::Ptr transformed_cloud(new pcl::PointCloud<PointType>());
  pcl::transformPointCloud(*points, *transformed_cloud, T);
  lidar_mtx.unlock();

  auto rvec = cv::Mat::zeros(3, 1, CV_64FC1);
  auto tvec = cv::Mat::zeros(3, 1, CV_64FC1);
  
  std::vector<cv::Point3f> points_cv;
  for (int i = 0; i < transformed_cloud->size(); i++) {
    auto p = transformed_cloud->points[i];
    points_cv.push_back(cv::Point3f(p.x, p.y, p.z));
  }

  std::vector<cv::Point2f> projected_points;
  camera_ptr->projectPoints(points_cv, rvec, tvec, projected_points);

  // initialize depth map as -1
  cv::Mat depth_map = cv::Mat::ones(image.rows, image.cols, CV_32FC1) * -1.;
  for (size_t i = 0 ; i < projected_points.size(); i++) {
    auto p = projected_points[i];
    auto depth = points_cv[i].z;
    if (depth > 0 && p.x >= 0 && p.x < width && p.y >= 0 && p.y < height) {
      auto row = static_cast<int>(p.y);
      auto col = static_cast<int>(p.x);
      depth_map.at<float>(row, col) = depth;
    }
  }

  // show depth map
  /* cv::Mat depth_map_show; */
  /* cv::normalize(depth_map, depth_map_show, 0, 1, cv::NORM_MINMAX); */
  /* cv::imshow("depth map", depth_map_show); */
  /* cv::waitKey(1); */

  return depth_map;

  /* // draw points on image, using 3D depth as color */
  /* cv::Mat color_image; */
  /* cv::cvtColor(image, color_image, CV_GRAY2RGB); */
  /* for (int i = 0; i < projected_points.size(); i++) { */
  /*   auto p = projected_points[i]; */
  /*   auto depth = transformed_cloud->points[i].z; */
  /*   std::cout << "depth: " << depth << std::endl; */
  /*   if (depth > 0) { */
  /*     cv::circle(color_image, p, 1, cv::Scalar(0, 0, static_cast<int>(255 * depth/25.)), 1); */
  /*   } */
  /* } */

  /* cv::imshow("projected points", color_image); */
  /* cv::waitKey(1); */

}

void right_img_callback(const sensor_msgs::ImageConstPtr& img_msg) {

  cv_bridge::CvImagePtr ptr;
  if (img_msg->encoding == "8UC1")
  {
      sensor_msgs::Image img;
      img.header = img_msg->header;
      img.height = img_msg->height;
      img.width = img_msg->width;
      img.is_bigendian = img_msg->is_bigendian;
      img.step = img_msg->step;
      img.data = img_msg->data;
      img.encoding = "mono8";
      ptr = cv_bridge::toCvCopy(img, sensor_msgs::image_encodings::MONO8);
  }
  else
      ptr = cv_bridge::toCvCopy(img_msg, sensor_msgs::image_encodings::MONO8);

  right_img_ptr = ptr;

  ROS_WARN("Got right image!");
}

cv::Mat readMatrixFromFile(const std::string& filepath, const std::string& key) {
    cv::FileStorage file(filepath, cv::FileStorage::READ);
    if(!file.isOpened())
    {
        std::cerr << "ERROR: readMatrixFromFile - could not read: " << filepath << std::endl;
    }
    
    cv::Mat matrix;
    file[key] >> matrix;
    file.release();
    return matrix;
}

void img_callback(const sensor_msgs::ImageConstPtr &img_msg)
{


    // convert img_msg to cv::Mat
    cv::Mat color_left_image = cv_bridge::toCvShare(img_msg, "bgr8")->image;

    double cur_img_time = img_msg->header.stamp.toSec();

    if(first_image_flag)
    {
        first_image_flag = false;
        first_image_time = cur_img_time;
        last_image_time = cur_img_time;
        return;
    }
    // detect unstable camera stream
    if (cur_img_time - last_image_time > 1.0 || cur_img_time < last_image_time)
    {
        ROS_WARN("image discontinue! reset the feature tracker!");
        first_image_flag = true; 
        last_image_time = 0;
        pub_count = 1;
        std_msgs::Bool restart_flag;
        restart_flag.data = true;
        pub_restart.publish(restart_flag);
        return;
    }
    last_image_time = cur_img_time;
    // frequency control
    if (round(1.0 * pub_count / (cur_img_time - first_image_time)) <= FREQ)
    {
        PUB_THIS_FRAME = true;
        // reset the frequency control
        if (abs(1.0 * pub_count / (cur_img_time - first_image_time) - FREQ) < 0.01 * FREQ)
        {
            first_image_time = cur_img_time;
            pub_count = 0;
        }
    }
    else
    {
        PUB_THIS_FRAME = false;
    }

    cv_bridge::CvImageConstPtr ptr;
    if (img_msg->encoding == "8UC1")
    {
        sensor_msgs::Image img;
        img.header = img_msg->header;
        img.height = img_msg->height;
        img.width = img_msg->width;
        img.is_bigendian = img_msg->is_bigendian;
        img.step = img_msg->step;
        img.data = img_msg->data;
        img.encoding = "mono8";
        ptr = cv_bridge::toCvCopy(img, sensor_msgs::image_encodings::MONO8);
    }
    else
        ptr = cv_bridge::toCvCopy(img_msg, sensor_msgs::image_encodings::MONO8);

    cv::Mat show_img = ptr->image;

    auto right_img = right_img_ptr->image;
    TicToc t_r;
    for (int i = 0; i < NUM_OF_CAM; i++)
    {
        if (i != 1 || !STEREO_TRACK) {
            trackerData[i].readImage(ptr->image.rowRange(ROW * i, ROW * (i + 1)), right_img, cur_img_time);
            auto current_left_pts = trackerData[i].cur_left_pts;
            
        }
        else
        {
            if (EQUALIZE)
            {
                cv::Ptr<cv::CLAHE> clahe = cv::createCLAHE();
                clahe->apply(ptr->image.rowRange(ROW * i, ROW * (i + 1)), trackerData[i].cur_right_img);
            }
            else
                trackerData[i].cur_right_img = ptr->image.rowRange(ROW * i, ROW * (i + 1));
        }

        #if SHOW_UNDISTORTION
            trackerData[i].showUndistortion("undistrotion_" + std::to_string(i));
        #endif
    }

    for (unsigned int i = 0;; i++)
    {
        bool completed = false;
        for (int j = 0; j < NUM_OF_CAM; j++)
            if (j != 1 || !STEREO_TRACK)
                completed |= trackerData[j].updateID(i);
        if (!completed)
            break;
    }

   if (PUB_THIS_FRAME)
   {
        pub_count++;
        sensor_msgs::PointCloudPtr feature_points(new sensor_msgs::PointCloud);
        sensor_msgs::ChannelFloat32 id_of_point;
        sensor_msgs::ChannelFloat32 u_of_point;
        sensor_msgs::ChannelFloat32 v_of_point;
        sensor_msgs::ChannelFloat32 velocity_x_of_point;
        sensor_msgs::ChannelFloat32 velocity_y_of_point;

        feature_points->header.stamp = img_msg->header.stamp;
        feature_points->header.frame_id = "vins_body";

        vector<set<int>> hash_ids(NUM_OF_CAM);
        for (int i = 0; i < NUM_OF_CAM; i++)
        {
            auto &un_pts = trackerData[i].cur_undist_left_pts;
            auto &cur_pts = trackerData[i].cur_left_pts;
            auto &ids_left = trackerData[i].ids_left;
            ROS_WARN("ids size: %d", ids_left.size());
            auto &pts_velocity = trackerData[i].pts_velocity;
            for (unsigned int j = 0; j < ids_left.size(); j++)
            {
                if (trackerData[i].track_cnt_left[j] > 1)
                {
                    int p_id = ids_left[j];
                    hash_ids[i].insert(p_id);
                    geometry_msgs::Point32 p;
                    p.x = un_pts[j].x;
                    p.y = un_pts[j].y;
                    p.z = 1;

                    feature_points->points.push_back(p);
                    id_of_point.values.push_back(p_id * NUM_OF_CAM + i);
                    u_of_point.values.push_back(cur_pts[j].x);
                    v_of_point.values.push_back(cur_pts[j].y);
                    velocity_x_of_point.values.push_back(pts_velocity[j].x);
                    velocity_y_of_point.values.push_back(pts_velocity[j].y);
                }
            }
        }

        feature_points->channels.push_back(id_of_point);
        feature_points->channels.push_back(u_of_point);
        feature_points->channels.push_back(v_of_point);
        feature_points->channels.push_back(velocity_x_of_point);
        feature_points->channels.push_back(velocity_y_of_point);

        // getting sparse depth map
        cv::Mat sparse_depth_map = getDepthMapFromLidarScan(point_cloud_ptr, show_img.clone(), T_cam_lidar, trackerData[0].m_camera);

        // show sparse depth map
        /* cv::Mat sparse_depth_map_show; */
        /* cv::normalize(sparse_depth_map, sparse_depth_map_show, 0, 1, cv::NORM_MINMAX); */
        /* cv::imshow("sparse depth map", sparse_depth_map_show); */
        /* cv::waitKey(1); */

        std_msgs::Header depth_map_msg_header;
        depth_map_msg_header.stamp = img_msg->header.stamp;
        depth_map_msg_header.frame_id = "left_camera";
        depth_map_pub.publish(cv_bridge::CvImage(depth_map_msg_header, "32FC1", sparse_depth_map).toImageMsg());

        // wait for dense depth map message
        while (dense_depth_map.empty()) {
          ros::spinOnce();
        }

        sensor_msgs::ChannelFloat32 depth_of_points;
        for (size_t i = 0 ; i < feature_points->points.size(); i++) {
          auto u = feature_points->channels[1].values[i];
          auto v = feature_points->channels[2].values[i];
          auto row = static_cast<int>(v);
          auto col = static_cast<int>(u);
          auto depth = sparse_depth_map.at<float>(row, col);
          depth_of_points.values.push_back(depth);
        }
    
        feature_points->channels.push_back(depth_of_points);
        
        // skip the first image; since no optical speed on frist image
        if (!init_pub)
        {
            init_pub = 1;
        }
        else
            pub_feature.publish(feature_points);

        // publish features in image
        if (pub_match.getNumSubscribers() != 0)
        {
            ptr = cv_bridge::cvtColor(ptr, sensor_msgs::image_encodings::RGB8);
            //cv::Mat stereo_img(ROW * NUM_OF_CAM, COL, CV_8UC3);
            cv::Mat stereo_img = ptr->image;

            for (int i = 0; i < NUM_OF_CAM; i++)
            {
                cv::Mat tmp_img = stereo_img.rowRange(i * ROW, (i + 1) * ROW);
                cv::cvtColor(show_img, tmp_img, CV_GRAY2RGB);

                for (unsigned int j = 0; j < trackerData[i].cur_left_pts.size(); j++)
                {
                    if (SHOW_TRACK)
                    {
                        // track count
                        double len = std::min(1.0, 1.0 * trackerData[i].track_cnt_left[j] / WINDOW_SIZE);
                        cv::circle(tmp_img, trackerData[i].cur_left_pts[j], 4, cv::Scalar(255 * (1 - len), 255 * len, 0), 4);
                    } else {
                        // depth 
                        if(j < depth_of_points.values.size())
                        {
                            if (depth_of_points.values[j] > 0)
                                cv::circle(tmp_img, trackerData[i].cur_left_pts[j], 4, cv::Scalar(0, 255, 0), 4);
                            else
                                cv::circle(tmp_img, trackerData[i].cur_left_pts[j], 4, cv::Scalar(0, 0, 255), 4);
                        }
                    }
                }
            }

            pub_match.publish(ptr->toImageMsg());
        }
    }
}


void lidar_callback(const sensor_msgs::PointCloud2ConstPtr& laser_msg)
{

  // just store the point cloud in a global variable
  lidar_mtx.lock();
  pcl::fromROSMsg(*laser_msg, *point_cloud_ptr);
  lidar_mtx.unlock();

}

void depth_map_callback(const sensor_msgs::ImageConstPtr& img_msg) {
  dense_depth_map = cv_bridge::toCvShare(img_msg, "32FC1")->image;
}

int main(int argc, char **argv)
{
    // initialize ROS node
    ros::init(argc, argv, "vins");
    ros::NodeHandle n;
    ROS_INFO("\033[1;32m----> Visual Feature Tracker Started.\033[0m");
    ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Info);
    readParameters(n);

    // read camera params
    for (int i = 0; i < NUM_OF_CAM; i++) {
        trackerData[i].readLeftCameraParameters(CAM_NAMES[i]);
    }

    // reading projection matrices
    trackerData[0].setLeftCameraProjectionMatrix(readMatrixFromFile(CAM_NAMES[0], "leftCameraProjectionMatrix"));
    trackerData[0].setRightCameraProjectionMatrix(readMatrixFromFile(CAM_NAMES[0], "rightCameraProjectionMatrix"));

    // reading extrinsics with IMU
    cv::Mat rotation_matrix = readMatrixFromFile(CAM_NAMES[0], "extrinsicRotation");
    cv::Mat translation_vector = readMatrixFromFile(CAM_NAMES[0], "extrinsicTranslation");
    Eigen::Vector3d eigen_translation;
    cv::cv2eigen(translation_vector, eigen_translation);

    Eigen::Matrix3d eigen_rotation;
    cv::cv2eigen(rotation_matrix, eigen_rotation);

    Eigen::Matrix4d T_imu_cam = Eigen::Matrix4d::Identity();
    T_imu_cam.block<3,3>(0,0) = eigen_rotation;
    T_imu_cam.block<3,1>(0,3) = eigen_translation;

    auto imu_lidar_origin = Transform_imu_lidar.getOrigin();
    auto imu_lidar_rotation = Transform_imu_lidar.getRotation();
    Eigen::Matrix4d T_imu_lidar = Eigen::Matrix4d::Identity();
    T_imu_lidar.block<3,3>(0,0) = Eigen::Quaterniond(imu_lidar_rotation.w(), imu_lidar_rotation.x(), imu_lidar_rotation.y(), imu_lidar_rotation.z()).toRotationMatrix();
    T_imu_lidar.block<3,1>(0,3) = Eigen::Vector3d(imu_lidar_origin.x(), imu_lidar_origin.y(), imu_lidar_origin.z());

    T_cam_lidar = T_imu_cam.inverse() * T_imu_lidar;

    std::cout << "T_cam_lidar: " << std::endl;

    // load fisheye mask to remove features on the boundry
    if(FISHEYE)
    {
        for (int i = 0; i < NUM_OF_CAM; i++)
        {
            trackerData[i].fisheye_mask = cv::imread(FISHEYE_MASK, 0);
            if(!trackerData[i].fisheye_mask.data)
            {
                ROS_ERROR("load fisheye mask fail");
                ROS_BREAK();
            }
            else
                ROS_INFO("load mask success");
        }
    }
    
    // subscriber to image and lidar
    ROS_WARN("Right image topic: %s", RIGHT_IMAGE_TOPIC.c_str());
    ros::Subscriber sub_img   = n.subscribe(IMAGE_TOPIC,       5,    img_callback);
    ros::Subscriber sub_image_right = n.subscribe(RIGHT_IMAGE_TOPIC, 5, right_img_callback);
    ros::Subscriber sub_lidar = n.subscribe(POINT_CLOUD_TOPIC, 5,    lidar_callback);
    ros::Subscriber dense_depth_map_sub = n.subscribe(DENSE_DEPTH_MAP_TOPIC, 5, depth_map_callback);
    if (!USE_LIDAR)
        sub_lidar.shutdown();

    // messages to vins estimator
    pub_feature = n.advertise<sensor_msgs::PointCloud>(PROJECT_NAME + "/vins/feature/feature",     5);
    pub_match   = n.advertise<sensor_msgs::Image>     (PROJECT_NAME + "/vins/feature/feature_img", 5);
    pub_restart = n.advertise<std_msgs::Bool>         (PROJECT_NAME + "/vins/feature/restart",     5);
    depth_map_pub = n.advertise<sensor_msgs::Image>   (PROJECT_NAME + "/vins/feature/depth_map",   5);

    // two ROS spinners for parallel processing (images and lidar)
    ros::MultiThreadedSpinner spinner(3);
    spinner.spin();

    return 0;
}
