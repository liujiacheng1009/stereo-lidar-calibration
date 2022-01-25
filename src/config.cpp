#include "config.hpp"

Config& Config::getInstance()
{
    static Config instance; // 只会创建一次
    return instance;
}

template<typename T>
inline T loadSafe(const YAML::Node &config, std::string param, T default_value = T()) {

    if (YAML::Node parameter = config[param])
        return parameter.as<T>();
    else
        return default_value;
}

static string getLeftImageDatasetPath(const YAML::Node& config)
{
    auto& cam0 = config["cam0"];
    return loadSafe(cam0, "image_path", Config::leftImageDatasetPath());
}

static string getRightImageDatasetPath(const YAML::Node& config)
{
    auto& cam0 = config["cam1"];
    return loadSafe(cam0, "image_path", Config::rightImageDatasetPath());
}

static string getLidarCloudDatasetPath(const YAML::Node& config)
{
    auto& lidar0 = config["lidar0"];
    return loadSafe(lidar0, "cloud_path", Config::lidarCloudDatasetPath());
}

static string getImageFormat(const YAML::Node& config)
{
    auto& cam0 = config["cam0"];
    return loadSafe(cam0, "image_format", Config::imageFormat());
}

static string getCloudFormat(const YAML::Node& config)
{
    auto& lidar0 = config["lidar0"];
    return loadSafe(lidar0, "cloud_format", Config::cloudFormat());
}


static cv::Mat getLeftCameraMatrix(const YAML::Node& config)
{
    cv::Mat left_camera_matrix = cv::Mat::eye(3, 3, CV_64F);
    auto& cam0 = config["cam0"];
    left_camera_matrix.at<double>(0, 0) = loadSafe(cam0, "cam_fx",0.0);
    left_camera_matrix.at<double>(0, 2) = loadSafe(cam0, "cam_cx",0.0);
    left_camera_matrix.at<double>(1, 1) = loadSafe(cam0, "cam_fy",0.0);
    left_camera_matrix.at<double>(1, 2) = loadSafe(cam0, "cam_cy",0.0);
    return left_camera_matrix;
}

static cv::Mat getRightCameraMatrix(const YAML::Node& config)
{
    cv::Mat camera_matrix = cv::Mat::eye(3, 3, CV_64F);
    auto& cam = config["cam1"];
    camera_matrix.at<double>(0, 0) = loadSafe(cam, "cam_fx",0.0);
    camera_matrix.at<double>(0, 2) = loadSafe(cam, "cam_cx",0.0);
    camera_matrix.at<double>(1, 1) = loadSafe(cam, "cam_fy",0.0);
    camera_matrix.at<double>(1, 2) = loadSafe(cam, "cam_cy",0.0);
    return camera_matrix;
}

static cv::Mat getLeftCameraDistCoeffs(const YAML::Node& config)
{
    cv::Mat left_camera_dist_coeffs = cv::Mat::zeros(5, 1, CV_64F);
    auto& cam0 = config["cam0"];
    left_camera_dist_coeffs.at<double>(0, 0) = loadSafe(cam0, "cam_k1",0.0);
    left_camera_dist_coeffs.at<double>(1, 0) = loadSafe(cam0, "cam_k2",0.0);
    left_camera_dist_coeffs.at<double>(2, 0) = loadSafe(cam0, "cam_p1",0.0);
    left_camera_dist_coeffs.at<double>(3, 0) = loadSafe(cam0, "cam_p2",0.0);
    left_camera_dist_coeffs.at<double>(4, 0) = loadSafe(cam0, "cam_k3",0.0);
    return left_camera_dist_coeffs;
}

static cv::Mat getRightCameraDistCoeffs(const YAML::Node& config)
{
    cv::Mat left_camera_dist_coeffs = cv::Mat::zeros(5, 1, CV_64F);
    auto& cam0 = config["cam1"];
    left_camera_dist_coeffs.at<double>(0, 0) = loadSafe(cam0, "cam_k1",0.0);
    left_camera_dist_coeffs.at<double>(1, 0) = loadSafe(cam0, "cam_k2",0.0);
    left_camera_dist_coeffs.at<double>(2, 0) = loadSafe(cam0, "cam_p1",0.0);
    left_camera_dist_coeffs.at<double>(3, 0) = loadSafe(cam0, "cam_p2",0.0);
    left_camera_dist_coeffs.at<double>(4, 0) = loadSafe(cam0, "cam_k3",0.0);
    return left_camera_dist_coeffs;
}

static double getCheckerboardSquareSize(const YAML::Node& config)
{
    auto& checkerboard = config["checkerboard"];
    return loadSafe(checkerboard, "square_size", 0.0);
}

static cv::Size getCheckerboardGridSize(const YAML::Node& config)
{
    cv::Size checkerboard_grid_size;
    auto& checkerboard = config["checkerboard"];
    checkerboard_grid_size.width = loadSafe(checkerboard, "width",0.0);
    checkerboard_grid_size.height = loadSafe(checkerboard, "height",0.0);
    return checkerboard_grid_size;
}

static vector<double> getCheckerboardPadding(const YAML::Node& config)
{
    vector<double> checkerboard_padding(4);
    auto& padding = config["checkerboard"]["padding"];
    checkerboard_padding[0] = loadSafe(padding, "p1",0.0);
    checkerboard_padding[1] = loadSafe(padding, "p2",0.0);
    checkerboard_padding[2] = loadSafe(padding, "p3",0.0);
    checkerboard_padding[3] = loadSafe(padding, "p4",0.0);
    return checkerboard_padding;
}

static Eigen::Matrix4d getTformL1C1(const YAML::Node& config)
{
    Eigen::Matrix4d matlab_tform = Eigen::Matrix4d::Identity();
    auto& matlab_result = config["tform_l1_c1"];
    matlab_tform(0, 0) = loadSafe(matlab_result, "m00",0.0);
    matlab_tform(0, 1) = loadSafe(matlab_result, "m01",0.0);
    matlab_tform(0, 2) = loadSafe(matlab_result, "m02",0.0);
    matlab_tform(0, 3) = loadSafe(matlab_result, "m03",0.0);
    matlab_tform(1, 0) = loadSafe(matlab_result, "m10",0.0);
    matlab_tform(1, 1) = loadSafe(matlab_result, "m11",0.0);
    matlab_tform(1, 2) = loadSafe(matlab_result, "m12",0.0);
    matlab_tform(1, 3) = loadSafe(matlab_result, "m13",0.0);
    matlab_tform(2, 0) = loadSafe(matlab_result, "m20",0.0);
    matlab_tform(2, 1) = loadSafe(matlab_result, "m21",0.0);
    matlab_tform(2, 2) = loadSafe(matlab_result, "m22",0.0);
    matlab_tform(2, 3) = loadSafe(matlab_result, "m23",0.0);
    return matlab_tform;
}

static Eigen::Matrix4d getTformL1C2(const YAML::Node& config)
{
    Eigen::Matrix4d matlab_tform = Eigen::Matrix4d::Identity();
    auto& matlab_result = config["tform_l1_c2"];
    matlab_tform(0, 0) = loadSafe(matlab_result, "m00",0.0);
    matlab_tform(0, 1) = loadSafe(matlab_result, "m01",0.0);
    matlab_tform(0, 2) = loadSafe(matlab_result, "m02",0.0);
    matlab_tform(0, 3) = loadSafe(matlab_result, "m03",0.0);
    matlab_tform(1, 0) = loadSafe(matlab_result, "m10",0.0);
    matlab_tform(1, 1) = loadSafe(matlab_result, "m11",0.0);
    matlab_tform(1, 2) = loadSafe(matlab_result, "m12",0.0);
    matlab_tform(1, 3) = loadSafe(matlab_result, "m13",0.0);
    matlab_tform(2, 0) = loadSafe(matlab_result, "m20",0.0);
    matlab_tform(2, 1) = loadSafe(matlab_result, "m21",0.0);
    matlab_tform(2, 2) = loadSafe(matlab_result, "m22",0.0);
    matlab_tform(2, 3) = loadSafe(matlab_result, "m23",0.0);
    return matlab_tform;
}

static Eigen::Matrix4d getTformC1C2(const YAML::Node& config)
{
    Eigen::Matrix4d matlab_tform = Eigen::Matrix4d::Identity();
    auto& matlab_result = config["tform_c1_c2"];
    matlab_tform(0, 0) = loadSafe(matlab_result, "m00",0.0);
    matlab_tform(0, 1) = loadSafe(matlab_result, "m01",0.0);
    matlab_tform(0, 2) = loadSafe(matlab_result, "m02",0.0);
    matlab_tform(0, 3) = loadSafe(matlab_result, "m03",0.0);
    matlab_tform(1, 0) = loadSafe(matlab_result, "m10",0.0);
    matlab_tform(1, 1) = loadSafe(matlab_result, "m11",0.0);
    matlab_tform(1, 2) = loadSafe(matlab_result, "m12",0.0);
    matlab_tform(1, 3) = loadSafe(matlab_result, "m13",0.0);
    matlab_tform(2, 0) = loadSafe(matlab_result, "m20",0.0);
    matlab_tform(2, 1) = loadSafe(matlab_result, "m21",0.0);
    matlab_tform(2, 2) = loadSafe(matlab_result, "m22",0.0);
    matlab_tform(2, 3) = loadSafe(matlab_result, "m23",0.0);
    return matlab_tform;
}

static vector<double> getPassFilterParams(const YAML::Node& config)
{
    vector<double> pass_filter_params(6);
    auto& RoI = config["RoI"];
    pass_filter_params[0] = loadSafe(RoI, "min_x",0.0);
    pass_filter_params[1] = loadSafe(RoI, "max_x",0.0);
    pass_filter_params[2] = loadSafe(RoI, "min_y",0.0);
    pass_filter_params[3] = loadSafe(RoI, "max_y",0.0);
    pass_filter_params[4] = loadSafe(RoI, "min_z",0.0);
    pass_filter_params[5] = loadSafe(RoI, "max_z",0.0);
    return pass_filter_params;
}

static std::string getCalibType(const YAML::Node& config)
{
    return loadSafe(config, "type",Config::calibType());
}

void Config::loadFromFile(const string& config_file)
{
    YAML::Node config = YAML::LoadFile(config_file);
    Config::calibType() = getCalibType(config);
    Config::leftImageDatasetPath() = getLeftImageDatasetPath(config);
    Config::lidarCloudDatasetPath() = getLidarCloudDatasetPath(config);
    Config::imageFormat() = getImageFormat(config);
    Config::cloudFormat() = getCloudFormat(config);
    Config::leftCameraMatrix() = getLeftCameraMatrix(config);
    Config::leftCameraDistCoeffs() = getLeftCameraDistCoeffs(config);
    Config::checkerboardSquareSize() = getCheckerboardSquareSize(config);
    Config::checkerboardGridSize() = getCheckerboardGridSize(config);
    Config::checkerboardPadding() = getCheckerboardPadding(config);
    Config::tformL1C1() = getTformL1C1(config);
    Config::passFilterParams() = getPassFilterParams(config);
    if(Config::calibType()=="stereo_lidar"){
        Config::rightImageDatasetPath() = getRightImageDatasetPath(config);
        Config::rightCameraMatrix() = getRightCameraMatrix(config);
        Config::rightCameraDistCoeffs() = getRightCameraDistCoeffs(config);
        Config::tformL1C2() = getTformL1C2(config);
        Config::tformC1C2() = getTformC1C2(config);
    }
    
}