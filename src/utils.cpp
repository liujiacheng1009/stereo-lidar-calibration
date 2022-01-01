#include"utils.hpp"

using namespace Eigen;
using namespace std;
using namespace pcl;
using namespace cv;

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
                     const pcl::PointCloud<pcl::PointXYZ>::Ptr target_cloud, Eigen::Matrix4d &trans)
{
    pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
    icp.setMaximumIterations (1000);
    icp.setInputSource(source_cloud);
    icp.setInputTarget(target_cloud);
    icp.setTransformationEpsilon(1e-8);
    pcl::PointCloud<pcl::PointXYZ> Final;
    icp.align(Final);
    if (!icp.hasConverged())
    {
        return false;
    }
    Eigen::Matrix4f temp_trans = icp.getFinalTransformation(); // colomn major
    trans = temp_trans.cast<double>();
    std::cout<<icp.getFitnessScore()<<std::endl;
    return true;
}

pcl::visualization::PCLVisualizer::Ptr show_multi_clouds(std::vector<pcl::PointCloud<pcl::PointXYZ>>& clouds)
{
    pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer ("debug"));
    viewer->setBackgroundColor(0,0,0);
    std::stringstream cloud_name;
    int counter = 0;
    for(auto& cloud:clouds){
        counter ++;
        cloud_name.str("");
        cloud_name << "Cloud " << counter;
        pcl::RGB rgb;
        pcl::visualization::getRandomColors(rgb);
        auto cloud_ptr = cloud.makeShared();
        pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> single_color(cloud_ptr, rgb.r, rgb.g, rgb.b);
        viewer->addPointCloud<pcl::PointXYZ>(cloud_ptr, single_color, cloud_name.str());
        viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, cloud_name.str());
    }
    viewer->addCoordinateSystem (1.0);
    viewer->initCameraParameters();
    return (viewer);
}

pcl::visualization::PCLVisualizer::Ptr show_multi_clouds_with_specified_colors(std::vector<pcl::PointCloud<pcl::PointXYZ>>& clouds)
{
    assert(clouds.size()==4);
    std::vector<pcl::RGB> colors = {pcl::RGB(255, 0, 0),
                                    pcl::RGB(0, 255, 0),
                                    pcl::RGB(0, 0, 255),
                                    pcl::RGB(255, 255, 0)};
    pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer ("debug"));
    viewer->setBackgroundColor(0,0,0);
    std::stringstream cloud_name;
    int counter = 0;
    for(auto& cloud:clouds){
        counter ++;
        cloud_name.str("");
        cloud_name << "Cloud " << counter;
        pcl::RGB rgb = colors[counter-1];
        //pcl::visualization::getRandomColors(rgb);
        auto cloud_ptr = cloud.makeShared();
        pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> single_color(cloud_ptr, rgb.r, rgb.g, rgb.b);
        viewer->addPointCloud<pcl::PointXYZ>(cloud_ptr, single_color, cloud_name.str());
        viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, cloud_name.str());
    }
    viewer->addCoordinateSystem (1.0);
    viewer->initCameraParameters();
    return (viewer);
}


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

// void reorder_corners(std::vector<cv::Point3d>& ori_corners, std::vector<cv::Point3d>& reordered_corners)
// {
//     //reorder_corners.clear();
//     double min_y = ori_corners[0].y;
//     int min_pos = 0;
//     for(int i=1;i<4;++i){
//         if(ori_corners[i].y<min_y){
//             min_y = ori_corners[i].y;
//             min_pos = i;
//         }
//     }
//     for(int i=0;i<4;++i){
//         int cur_pos = (i+min_pos) % 4;
//         reordered_corners.push_back(ori_corners[cur_pos]);
//     }
//     if(reordered_corners[0].x>reordered_corners[1].x){
//         reverse(reordered_corners.begin()+1, reordered_corners.end());
//     }

//     return;
// }
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
    sort(data.begin(), data.end());
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


void matrix2Vector(MatrixXd& m, VectorXd& v)
{
    v.resize(6);
    Matrix3d rot = m.block(0,0,3,3);
    Vector3d euler_angles = rot.eulerAngles(0,1,2);
    v(0) = euler_angles(0);
    v(1) = euler_angles(1);
    v(2) = euler_angles(2);
    v(3) = m(0,3);
    v(4) = m(1,3);
    v(5) = m(2,3);
}

void vector2Matrix(VectorXd& v, MatrixXd& m){
    m = Matrix4d::Identity();
    AngleAxisd rollAngle(v(0), Vector3d::UnitZ());
    AngleAxisd yawAngle(v(1), Vector3d::UnitY());
    AngleAxisd pitchAngle(v(2), Vector3d::UnitX());
    Quaternion<double> q = rollAngle * yawAngle * pitchAngle;
    m.block(0,0,3,3) = q.matrix();
    m(0,3) = v(3);
    m(1,3) = v(4);
    m(2,3) = v(5);
}

void  vector2Affine(Eigen::VectorXd& Rt, Eigen::Affine3d& affine_mat)
{
    affine_mat = Eigen::Affine3d::Identity();
    affine_mat.rotate(Eigen::AngleAxis<double>(Rt(0), Eigen::Vector3d::UnitX()));
    affine_mat.rotate(Eigen::AngleAxis<double>(Rt(1), Eigen::Vector3d::UnitY()));
    affine_mat.rotate(Eigen::AngleAxis<double>(Rt(2), Eigen::Vector3d::UnitZ()));
    affine_mat.translate ( Eigen::Vector3d(Rt(3) , Rt(4) , Rt(5)));
    return;            
}

void  affine2Vector( Eigen::Affine3d& affine_mat,Eigen::VectorXd& Rt)
{
    Eigen::Matrix3d rot = affine_mat.matrix().block(0,0,3,3);
    Eigen::Vector3d euler_angles = rot.eulerAngles(0, 1, 2);
    Rt(0) = euler_angles(0);
    Rt(1) = euler_angles(1);
    Rt(2) = euler_angles(2);
    Rt(3) = affine_mat.translation().x();
    Rt(4) = affine_mat.translation().y();
    Rt(5) = affine_mat.translation().z();
    return;
}


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

