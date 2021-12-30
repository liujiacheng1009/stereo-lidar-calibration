#ifndef STEREO_LIDAR_CALIBRATION_UTILS_HPP
#define STEREO_LIDAR_CALIBRATION_UTILS_HPP
#include <pcl/ModelCoefficients.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
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
#include <boost/filesystem.hpp>
#include <pcl/common/geometry.h>
#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>
#include <pcl/registration/icp.h>
#include <Eigen/Dense>
#include <string>
#include <cmath>
#include <unistd.h>

using namespace Eigen;
using namespace std;
using namespace pcl;
using namespace cv;

void display_colored_by_depth(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);

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

pcl::visualization::PCLVisualizer::Ptr show_multi_clouds(std::vector<pcl::PointCloud<pcl::PointXYZ>>& clouds);

pcl::visualization::PCLVisualizer::Ptr show_multi_clouds_with_specified_colors(std::vector<pcl::PointCloud<pcl::PointXYZ>>& clouds);

// void MonoPattern::getOrderedCorner(const cv::Mat& cam_corners, pcl::PointCloud<pcl::PointXYZ>::Ptr corner_cloud)
// {
//   corner_cloud->clear();
//   // camera coordinate: x axis points to right, y axis points to down, z axis points to front which is vertical to x-y
//   // plane. So the top point's y is smallest.
//   double min_y = cam_corners.at<float>(1, 0);
//   int min_y_pos = 0;
//   for (int i = 1; i < 4; ++i)
//   {
//     if (cam_corners.at<float>(1, i) < min_y)
//     {
//       min_y = cam_corners.at<float>(1, i);
//       min_y_pos = i;
//     }
//   }
//   for (int i = 0; i < 4; ++i)
//   {
//     int cur_pos = (i + min_y_pos) % 4;
//     corner_cloud->points.emplace_back(cam_corners.at<float>(0, cur_pos), cam_corners.at<float>(1, cur_pos),
//                                       cam_corners.at<float>(2, cur_pos));
//   }
// }
// void reorder_corners1(std::vector<cv::Point3d>& ori_corners, std::vector<cv::Point3d>& reordered_corners)
// {
//     sort(ori_corners.begin(), ori_corners.end(),[](cv::Point3d& a, cv::Point3d& b){
//         return a.x>b.x;
//     });
//     if(ori_corners[0].y<ori_corners[1].y){
//         reordered_corners.push_back(ori_corners[0]);
//         reordered_corners.push_back(ori_corners[1]);
//     }else{
//         reordered_corners.push_back(ori_corners[1]);
//         reordered_corners.push_back(ori_corners[0]);
//     }
//     if(ori_corners[2].y>ori_corners[3].y){
//         reordered_corners.push_back(ori_corners[2]);
//         reordered_corners.push_back(ori_corners[3]);
//     }else{
//         reordered_corners.push_back(ori_corners[3]);
//         reordered_corners.push_back(ori_corners[2]);
//     }
//     return;
// }

// void reorder_corners1(std::vector<pcl::PointXYZ>& ori_corners, std::vector<pcl::PointXYZ>& reordered_corners)
// {
//     sort(ori_corners.begin(), ori_corners.end(),[](pcl::PointXYZ& a, pcl::PointXYZ& b){
//         return a.z>b.z;
//     });
// }
// 调整lidar points 的顺序
void reorder_corners(std::vector<pcl::PointXYZ>& ori_corners, std::vector<pcl::PointXYZ>& reordered_corners);

// 调整边缘直线上的点云顺序
void reorder_line_points(std::vector<pcl::PointXYZ>  &reordered_corners,
                         vector<PointCloud<PointXYZ>> &reordered_lines_points,
                         vector<pair<pair<PointXYZ, PointXYZ>, PointCloud<PointXYZ>>> &line_corners_to_points);

void reorder_corners(std::vector<cv::Point3d>& ori_corners, std::vector<cv::Point3d>& reordered_corners);

// pcl::visualization::PCLVisualizer::Ptr customColourVis (pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud)
// {
//   // --------------------------------------------
//   // -----Open 3D viewer and add point cloud-----
//   // --------------------------------------------
//   pcl::visualization::PCLVisualizer::Ptr viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
//   viewer->setBackgroundColor (0, 0, 0);
//   pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> single_color(cloud, 0, 255, 0);
//   viewer->addPointCloud<pcl::PointXYZ> (cloud, single_color, "sample cloud");
//   viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "sample cloud");
//   viewer->addCoordinateSystem (1.0);
//   viewer->initCameraParameters ();
//   return (viewer);
// }


cv::Point2d project(cv::Point3d p, cv::Mat CameraMat);

// 获取指定路径下指定后缀名的文件完整路径
void get_data_by_path(std::vector<std::string>& data, std::string& path, std::string ext);

// 检查图像和激光点云的文件是否对应
bool check_data_by_index(std::vector<std::string>& data1, std::vector<std::string>& data2);

// bool readConfigFile(const std::string& config_file_name)
// {
//   cv::FileStorage fs_reader(config_file_name, cv::FileStorage::READ);
//   if (!fs_reader.isOpened())
//   {
//     std::cout << config_file_name << " is wrong!" << std::endl;
//     return false;
//   }
//   fs_reader["calib_frame_num"] >> calib_frame_num_;
//   fs_reader["calib_result_file"] >> calib_result_file_;
//   fs_reader.release();
//   if (calib_frame_num_ < 0)
//   {
//     std::cout << "calib frame num should large than 0 or equal to -1!" << std::endl;
//     return false;:
//   }
//   return true;
// }


// 保存YAML 文件
// void saveYamlFile(const std::string file_name, const aruco::CameraParameters& cam_param,
//                   const std::vector<double>& pose, const Eigen::Matrix4d& transformation)
// {
//   cv::FileStorage fs_writer(file_name, cv::FileStorage::WRITE);
//   if (fs_writer.isOpened())
//   {
//     cv::Mat trans_mat;
//     cv::eigen2cv(transformation, trans_mat);
//     fs_writer << "CameraMat" << cam_param.CameraMatrix;
//     fs_writer << "DistCoeff" << cam_param.Distorsion;
//     fs_writer << "ImageSize" << cam_param.CamSize;
//     fs_writer << "CameraExtrinsicMat" << trans_mat;
//     fs_writer << "Pose" << pose;
//     fs_writer.release();
//     INFO << "Save result file successfully!" << REND;
//   }
//   else
//   {
//     WARN << "Fail to open yaml file" << REND;
//   }
//   fs_writer.release();
// }
// }  // namespace cicv

// 文件路径操作
bool isDirExist(const std::string& path_name);

// bool createNewDir(const std::string& path_name)
// {
//   if (isDirExist(path_name))
//   {
//     return true;
//   }
//   return boost::filesystem::create_directories(path_name);
// }

// bool isFileExist(const std::string& file_name)
// {
//   if (boost::filesystem::exists(file_name) && boost::filesystem::is_regular_file(file_name))
//   {
//     return true;
//   }
//   return false;
// }

// bool createNewFile(const std::string& file_name)
// {
//   if (isFileExist(file_name))
//   {
//     return true;
//   }
//   boost::filesystem::ofstream file(file_name);
//   file.close();
//   return isFileExist(file_name);
// }


void matrix2Vector(MatrixXd& m, VectorXd& v);

void vector2Matrix(VectorXd& v, MatrixXd& m);

void  vector2Affine(Eigen::VectorXd& Rt, Eigen::Affine3d& affine_mat);

void  affine2Vector( Eigen::Affine3d& affine_mat,Eigen::VectorXd& Rt);
// for (int i = 0; i < image_3d_corners.size(); i++)
// {
//     // cv::Mat image = cv::imread(images[valid_image_index[i]], cv::IMREAD_COLOR);
//     std::vector<cv::Point3d> &corners = image_3d_corners[i];
//     for (auto &corner : corners)
//     {
//         // cv::Point2d p = project(corner, camera_matrix);
//         // cv::circle(image, p, 5, cv::Scalar(0, 255, 0), -1);
//         std::cout<<corner<<std::endl;
//     }
//     std::cout<<std::endl;
//     // cv::imwrite("/home/jc/Documents/stereo-lidar-calibration/exclude_dir/test_data/" 
//     //     + to_string(i) + ".png", image);
// }


#endif