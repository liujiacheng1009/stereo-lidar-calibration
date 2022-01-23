#ifndef STEREO_LIDAR_CALIBRATION_EVALUATION_HPP
#define STEREO_LIDAR_CALIBRATION_EVALUATION_HPP
#include "utils.hpp"
#include "config.hpp"

void projectLidarOnImage(cv::Mat& img, pcl::PointCloud<pcl::PointXYZ>& cloud, cv::Mat& rvec, cv::Mat& tvec, cv::Mat K, cv::Mat dist);

pcl::PointCloud<pcl::PointXYZRGB>::Ptr colorizeCloud(cv::Mat& img, pcl::PointCloud<pcl::PointXYZ>& cloud, cv::Mat& rvec, cv::Mat& tvec, cv::Mat K, cv::Mat dist);

std::vector<double> computeReprojectionError(std::vector<std::vector<Eigen::Vector3d>> &image_corners_3d,
                                            std::vector<std::vector<Eigen::Vector3d>> &cloud_corners_3d,
                                            Eigen::Matrix4d &tform, 
                                            cv::Mat camera_matrix = Config::leftCameraMatrix(), 
                                            cv::Mat dist_coeff = Config::leftCameraDistCoeffs());

std::vector<double> computeRotationError(std::vector<std::vector<Eigen::Vector3d>> &image_corners_3d,
                                            std::vector<std::vector<Eigen::Vector3d>> &cloud_corners_3d,
                                            Eigen::Matrix4d &tform);

std::vector<double> computeTranslationError(std::vector<std::vector<Eigen::Vector3d>> &image_corners_3d,
                                        std::vector<std::vector<Eigen::Vector3d>> &cloud_corners_3d,
                                        Eigen::Matrix4d &tform);  

void project_to_image_axis(std::vector<std::vector<Eigen::Vector3d>> &cloud_corners_3d,
                                            std::vector<std::vector<Eigen::Vector3d>> &transformed_cloud_corners_3d,
                                            Eigen::Matrix4d &tform);

Eigen::Vector3d get_normal(std::vector<Eigen::Vector3d>& corners_3d);

Eigen::Vector3d get_centroid(std::vector<Eigen::Vector3d>& corners_3d);
#endif