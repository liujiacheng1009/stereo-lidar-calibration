#ifndef STEREO_LIDAR_CALIBRATION_EVALUATION_HPP
#define STEREO_LIDAR_CALIBRATION_EVALUATION_HPP
#include "utils.hpp"

void projectLidarOnImage(cv::Mat& img, pcl::PointCloud<pcl::PointXYZ>& cloud, cv::Mat& rvec, cv::Mat& tvec, cv::Mat K, cv::Mat dist);

pcl::PointCloud<pcl::PointXYZRGB>::Ptr colorizeCloud(cv::Mat& img, pcl::PointCloud<pcl::PointXYZ>& cloud, cv::Mat& rvec, cv::Mat& tvec, cv::Mat K, cv::Mat dist);

#endif