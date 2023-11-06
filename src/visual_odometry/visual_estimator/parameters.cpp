#include "parameters.h"
#include "parameters_utils.h"

std::string PROJECT_NAME;

double INIT_DEPTH;
double MIN_PARALLAX;
double ACC_N, ACC_W;
double GYR_N, GYR_W;

std::vector<Eigen::Matrix3d> RIC;
std::vector<Eigen::Vector3d> TIC;

Eigen::Vector3d G{0.0, 0.0, 9.8};

double BIAS_ACC_THRESHOLD;
double BIAS_GYR_THRESHOLD;
double SOLVER_TIME;
int NUM_ITERATIONS;
int ESTIMATE_EXTRINSIC;
int ESTIMATE_TD;
int ROLLING_SHUTTER;
std::string EX_CALIB_RESULT_PATH;
std::string IMU_TOPIC;
double ROW, COL;
double TD, TR;

int USE_LIDAR;
int ALIGN_CAMERA_LIDAR_COORDINATE;


Eigen::Matrix3d R_imu_lidar = Eigen::Matrix3d::Identity(); 
Eigen::Vector3d t_imu_lidar = Eigen::Vector3d::Zero();

Options readParameters(ros::NodeHandle &n)
{

    Options options = loadInputParameters(n);

    PROJECT_NAME = "lvi_sam";

    // project name
    std::string pkg_path = ros::package::getPath(PROJECT_NAME);

    // IMU config
    IMU_TOPIC = options.imu_params.imu_topic;
    ACC_N = options.imu_params.acc_n;
    ACC_W = options.imu_params.acc_w;
    GYR_N = options.imu_params.gyr_n;
    GYR_W = options.imu_params.gyr_w;
    G.z() = options.imu_params.g.z();

    // Lidar config
    USE_LIDAR = options.lidar_options.use;
    
    // estimator config
    n.getParam(PROJECT_NAME+ "/align_camera_lidar_estimation", ALIGN_CAMERA_LIDAR_COORDINATE);
    SOLVER_TIME = options.estimator_options.max_solver_time;
    NUM_ITERATIONS = options.estimator_options.max_num_iterations;
    MIN_PARALLAX = options.estimator_options.keyframe_parallax;

    for (const auto& cam : options.per_camera_options)
    {
        RIC.push_back(cam.R);
        TIC.push_back(cam.t);
    }
    
    ROW = options.per_camera_options[0].image_height;
    COL = options.per_camera_options[0].image_width;
    ROS_INFO("Image dimention: ROW: %f COL: %f ", ROW, COL);

    ESTIMATE_EXTRINSIC = 0;       // remove support

    for (int i = 0; i < NUM_OF_CAM; i++)
    {
        ROS_INFO_STREAM("Extrinsic_R : " << std::endl
                                         << RIC[i]);
        ROS_INFO_STREAM("Extrinsic_T : " << std::endl
                                         << TIC[i].transpose());
    }

    INIT_DEPTH = 5.0;
    BIAS_ACC_THRESHOLD = 0.1;
    BIAS_GYR_THRESHOLD = 0.1;

    n.getParam(PROJECT_NAME+ "/td", TD);
    ESTIMATE_TD = options.estimator_options.estimate_td;
    if (ESTIMATE_TD)
        ROS_INFO_STREAM("Unsynchronized sensors, online estimate time offset, initial td: " << TD);
    else
        ROS_INFO_STREAM("Synchronized sensors, fix time offset: " << TD);

    ROLLING_SHUTTER = options.per_camera_options[0].is_rolling_shutter;
    if (ROLLING_SHUTTER)
    {
        TR = options.per_camera_options[0].rolling_shutter_readout_time;
        ROS_INFO_STREAM("rolling shutter camera, read out time per line: " << TR);
    }
    else
    {
        TR = 0;
    }

    std::vector<double> t_imu_lidar_V;
    std::vector<double> R_imu_lidar_V;
    n.param<std::vector<double>>(PROJECT_NAME+ "/extrinsicTranslation", t_imu_lidar_V, std::vector<double>());
    n.param<std::vector<double>>(PROJECT_NAME+ "/extrinsicRotation", R_imu_lidar_V, std::vector<double>());
    t_imu_lidar = Eigen::Map<const Eigen::Matrix<double, -1, -1, Eigen::RowMajor>>(t_imu_lidar_V.data(), 3, 1);
    Eigen::Matrix3d R_tmp = Eigen::Map<const Eigen::Matrix<double, -1, -1, Eigen::RowMajor>>(R_imu_lidar_V.data(), 3, 3);
    ROS_ASSERT(abs(R_tmp.determinant()) > 0.9);   // 防止配置文件中写错，这里加一个断言判断一下
    R_imu_lidar = Eigen::Quaterniond(R_tmp).normalized().toRotationMatrix();
    
    ROS_WARN_STREAM("=vins-estimator read R_imu_lidar : =====================");
    std::cout << R_imu_lidar << std::endl;
    ROS_WARN_STREAM("=vins-estimator read t_imu_lidar : =====================");
    std::cout << t_imu_lidar.transpose() << std::endl;

    usleep(100);

    return options;
}
