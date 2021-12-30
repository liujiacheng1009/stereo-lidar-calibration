#ifndef STEREO_LIDAR_CALIBRATION_CONFIG_HPP
#define STEREO_LIDAR_CALIBRATION_CONFIG_HPP
#include <vector>
#include <string>
#include <opencv2/core.hpp>
#include <Eigen/Core>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/core/utility.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/videoio.hpp>

using namespace cv;
using namespace Eigen;
using namespace std;

class Config
{
public:
    Config()
    {
        // 数据集路径参数
        left_images_dataset_path =
            "/home/jc/Documents/stereo-lidar-calibration/exclude_dir/lcc/HDL64/images";
        lidar_clouds_dataset_path =
            "/home/jc/Documents/stereo-lidar-calibration/exclude_dir/lcc/HDL64/pointCloud";
        // 数据类型
        image_format = ".png";
        cloud_format = ".pcd";
        // 标定板参数
        cheackerboard_square_size = 0.200;        // 单位是m
        cheackerboard_grid_size = Size(7, 4);     // 标定板的内部方格数，注意水平方向为7
        cheackerboard_padding = {0., 0., 0., 0.}; // 标定板相对靶纸边缘的padding,方向？

        // 左目相机内参
        left_camera_matrix.at<double>(0, 0) = 1.614546232069338e+03;
        left_camera_matrix.at<double>(0, 2) = 6.412276358621397e+02;
        left_camera_matrix.at<double>(1, 1) = 1.614669013419422e+03;
        left_camera_matrix.at<double>(1, 2) = 4.801410561665820e+02;
        // 左目相机畸变
        left_camera_dist_coeffs.at<double>(0, 0) = -0.004497294509341;
        left_camera_dist_coeffs.at<double>(1, 0) = 0.020426051162860;
        // matlab HDL64 数据集的标定结果
        matlab_hdl64_tform << -0.00565596282356831, -0.997885643377717, 0.0647476086444744, -0.00305628991168519,
            -0.156655954239030, -0.0630649871213768, -0.985637722188452, -0.402445253372315,
            0.987637039648674, -0.0157178287310998, -0.155968034462138, -0.826776648946616,
            0, 0, 0, 1;   
    }

public:
    // 数据集路径参数
    string left_images_dataset_path =
        "/home/jc/Documents/stereo-lidar-calibration/exclude_dir/lcc/HDL64/images";
    string lidar_clouds_dataset_path =
        "/home/jc/Documents/stereo-lidar-calibration/exclude_dir/lcc/HDL64/pointCloud";
    // 数据类型
    string image_format = ".png";
    string cloud_format = ".pcd";

    // 左目相机内参
    cv::Mat left_camera_matrix = cv::Mat::eye(3, 3, CV_64F);

    // 左目相机畸变
    cv::Mat left_camera_dist_coeffs = cv::Mat::zeros(5, 1, CV_64F);

    // 标定板参数
    double cheackerboard_square_size;
    Size cheackerboard_grid_size;
    vector<double> cheackerboard_padding;
    // matlab HDL64 数据集的标定结果
    Matrix4d matlab_hdl64_tform;

    // 点云的带通滤波参数
    vector<double> pass_filter_params = {-10,10,-10,10,-10,10};
};

#endif