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
#include <pcl/point_types.h>
#include <boost/filesystem.hpp>
#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>

#include <string>
#include <unistd.h>


void display_colored_by_depth(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
{
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));

    pcl::visualization::PointCloudColorHandlerGenericField<pcl::PointXYZ> fildColor(cloud, "z"); // 按照z字段进行渲染

    viewer->addPointCloud<pcl::PointXYZ>(cloud, fildColor, "sample cloud");
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "sample cloud"); // 设置点云大小

    while (!viewer->wasStopped())
    {
        viewer->spinOnce(100);
        //boost::this_thread::sleep(boost::posix_time::microseconds(100000));
        sleep(0.1);
    }
}

cv::Point2d project(cv::Point3d p, cv::Mat CameraMat)
{
    cv::Point2d pix;
    pix.x = (p.x * CameraMat.at<double>(0, 0)) / p.z + CameraMat.at<double>(0, 2);
    pix.y = ((p.y * CameraMat.at<double>(1, 1)) / p.z + CameraMat.at<double>(1, 2));
    return pix;
}

// 获取指定路径下指定后缀名的文件完整路径
void get_data_by_path(std::vector<std::string>& data, std::string& path, std::string ext)
{

    data.clear();
    boost::filesystem::path data_path(path);
    if(!boost::filesystem::exists(data_path)) return;
    boost::filesystem::recursive_directory_iterator end_iter;
    for( boost::filesystem::recursive_directory_iterator iter(data_path);iter!=end_iter;iter++)
    {
        if(boost::filesystem::is_directory( *iter )) continue;
        boost::filesystem::path file_path(iter->path().string());
        if(file_path.extension()==ext){
            data.push_back(iter->path().string());
        }
    }
    return;
}

// 检查图像和激光点云的文件是否对应
bool check_data_by_index(std::vector<std::string>& data1, std::vector<std::string>& data2)
{
    assert(data1.size()>0&& data1.size()==data2.size());
    for(int i=0;i<data1.size();++i){
        std::string& file1 = data1[i];
        std::string& file2 = data2[i];
        boost::filesystem::path bfile1(file1);
        if(!boost::filesystem::exists(bfile1)) return false;
        boost::filesystem::path bfile2(file2);
        if(!boost::filesystem::exists(bfile2)) return false;
        std::string file_name1 = bfile1.stem().string();
        std::string file_name2 = bfile2.stem().string();
        if(file_name1!=file_name2) return false;
    }
    return true;
}

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
bool isDirExist(const std::string& path_name)
{
  if (boost::filesystem::exists(path_name) && boost::filesystem::is_directory(path_name))
  {
    return true;
  }
  return false;
}

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


#endif