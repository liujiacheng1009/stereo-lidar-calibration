#ifndef STEREO_LIDAR_CALIBRATION_UTILS_HPP
#define STEREO_LIDAR_CALIBRATION_UTILS_HPP
#include <pcl/ModelCoefficients.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/io.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/point_types.h>
#include <pcl/conversions.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/common/geometry.h>
#include <pcl/registration/icp.h>

#include <Eigen/Dense>
#include <Eigen/Dense>
#include <Eigen/SVD>
#include <Eigen/Geometry>
#include <unsupported/Eigen/MatrixFunctions>
#include <set>
#include <string>
#include <cmath>
#include <unistd.h>
#include <vector>
#include <algorithm>
#include <boost/filesystem.hpp>
#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/calib3d.hpp>
#include <sophus/so3.hpp>

using namespace Eigen;
using namespace std;



class ImageResults{
public:
    vector<int> valid_index; // 有效的图像index
    vector<vector<Vector3d>> corners_3d; // 图像3d角点
    vector<vector<Vector2d>> chessboard_points_2d;
    vector<vector<Vector3d>> chessboard_points_3d;
    vector<VectorXd> planes_3d; // 图像的平面方程
    vector<vector<VectorXd>> lines_3d; // 图像边缘直线方程
};

class CloudResults{
public:
    vector<int> valid_index; // 有效的点云index
    vector<vector<Vector3d>> corners_3d; // 点云3d角点
    vector<pcl::PointCloud<pcl::PointXYZ>> plane_points_3d; 
    //vector<vector<Vector3d>> plane_points_3d;
    vector<vector<VectorXd>> lines_3d; // 点云边缘直线方程
    vector<vector<pcl::PointCloud<pcl::PointXYZ>>> lines_points_3d ;//直线包含的点云
};

void display_colored_by_depth(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);
void display_four_corners(vector<pcl::PointXYZ>& corners);
void display_multi_clouds(vector<pcl::PointCloud<pcl::PointXYZ>>& clouds);

// void show_cloud(pcl::PointCloud<pcl::PointXYZ>& cloud, std::string viewer_name="debug"){
//     pcl::visualization::CloudViewer viewer(viewer_name);
//     viewer.showCloud(cloud);
//     while (!viewer.wasStopped ())
//     {
//         sleep(0.1);
//     }
//     return;
// }
bool computeTransICP(const pcl::PointCloud<pcl::PointXYZ>::Ptr source_cloud,
                     const pcl::PointCloud<pcl::PointXYZ>::Ptr target_cloud, Eigen::Matrix4d &trans);

void calculateInitialRt(vector<vector<Vector3d>>& cloud_3d_corners, vector<vector<Vector3d>>& image_3d_corners, VectorXd& R_t);

pcl::visualization::PCLVisualizer::Ptr show_multi_clouds(std::vector<pcl::PointCloud<pcl::PointXYZ>>& clouds);

pcl::visualization::PCLVisualizer::Ptr show_multi_clouds_with_specified_colors(std::vector<pcl::PointCloud<pcl::PointXYZ>>& clouds);

cv::Point2d project(cv::Point3d p, cv::Mat CameraMat);

// 获取指定路径下指定后缀名的文件完整路径
void get_data_by_path(std::vector<std::string>& data, std::string& path, std::string ext);

// 检查图像和激光点云的文件是否对应
bool check_data_by_index(std::vector<std::string>& data1, std::vector<std::string>& data2);

bool loadPointXYZ(string& path, pcl::PointCloud<pcl::PointXYZ>::Ptr& pcd);
// 文件路径操作
bool isDirExist(const std::string& path_name);

void matrix2Vector(Matrix4d& m, VectorXd& v);

void vector2Matrix(VectorXd& v, Matrix4d& m);

void  vector2Affine(Eigen::VectorXd& Rt, Eigen::Affine3d& affine_mat);

void  affine2Vector( Eigen::Affine3d& affine_mat,Eigen::VectorXd& Rt);

void draw_axis(cv::Mat& img, cv::Mat& rvec, cv::Mat& tvec, cv::Mat K, cv::Mat dist);

void draw_corners_in_image(cv::Mat& img, std::vector<cv::Point3d>& corners, cv::Mat& rvec, cv::Mat& tvec, cv::Mat K, cv::Mat dist);

#endif