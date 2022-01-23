#include"utils.hpp"

using namespace Eigen;
using namespace std;


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

void display_four_corners(vector<pcl::PointXYZ>& corners)
{   
    pcl::visualization::PCLVisualizer::Ptr viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    for(auto& corner:corners){
        cloud->push_back(corner);
    }
    viewer->addPointCloud<pcl::PointXYZ>(cloud, "sample cloud");
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 10, "sample cloud");
    while (!viewer->wasStopped())
    {
        viewer->spinOnce(100);
        //boost::this_thread::sleep(boost::posix_time::microseconds(100000));
    }
    return;
}

void display_multi_clouds(vector<pcl::PointCloud<pcl::PointXYZ>>& clouds)
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
    while (!viewer->wasStopped())
    {
        viewer->spinOnce(100);
        //boost::this_thread::sleep(boost::posix_time::microseconds(100000));
    }
    return;
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
        if(file_path.extension()=="." + ext){
            data.push_back(iter->path().string());
        }
    }
    sort(data.begin(), data.end());
    return;
}

bool loadPointXYZ(string& path, pcl::PointCloud<pcl::PointXYZ>::Ptr& pcd)
{
    pcl::PCDReader reader;
    pcl::PCLPointCloud2::Ptr cloud(new pcl::PCLPointCloud2);
    if(reader.read(path, *cloud)!=0){
        return false;
    }
    pcl::fromPCLPointCloud2(*cloud, *pcd);
    return true;
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

// 文件路径操作
bool isDirExist(const std::string& path_name)
{
  if (boost::filesystem::exists(path_name) && boost::filesystem::is_directory(path_name))
  {
    return true;
  }
  return false;
}



void matrix2Vector(Matrix4d& m, VectorXd& v)
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

void vector2Matrix(VectorXd& v, Matrix4d& m){
    m = Matrix4d::Identity();
    Eigen::Matrix3d R3 = Sophus::SO3d::exp(v.head(3)).matrix();
    m.block(0,0,3,3) = R3;
    m.block(0,3,3,1) = v.tail(3);
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


void calculateInitialRt(vector<vector<Vector3d>>& cloud_3d_corners, vector<vector<Vector3d>>& image_3d_corners, VectorXd& R_t)
{
    assert(cloud_3d_corners.size()== image_3d_corners.size());
    int n = cloud_3d_corners.size() * 4;
    MatrixXd cloud_corners, image_corners;
    cloud_corners.resize(3,n);
    image_corners.resize(3,n);
    for(int i=0;i<cloud_3d_corners.size();++i){
        for(int j=0;j<4;++j){
            cloud_corners.col(i*4+j) = cloud_3d_corners[i][j].transpose();
        }
    }

    for(int i=0;i<image_3d_corners.size();++i){
        for(int j=0;j<4;++j){
            image_corners.col(i*4+j) = image_3d_corners[i][j].transpose();
        }
    }
    
    Vector3d mu_cloud_points, mu_image_points;
    mu_cloud_points = Vector3d::Zero();
    mu_image_points = Vector3d::Zero();
    for(int i = 0;i<n;++i){
        mu_cloud_points += cloud_corners.col(i);
        mu_image_points += image_corners.col(i);
    }
    mu_cloud_points /= n;
    mu_image_points /= n;

    cloud_corners = cloud_corners.colwise() - mu_cloud_points;
    image_corners = image_corners.colwise() - mu_image_points;

    Matrix3d cov = image_corners * cloud_corners.transpose();
    JacobiSVD <MatrixXd> svd(cov, ComputeFullU | ComputeFullV);
    Matrix3d rotation = svd.matrixU() * svd.matrixV().transpose();
    if(rotation.determinant() < 0){
        Vector3d diag;
        diag << 1.0, 1.0, -1.0;
        rotation = svd.matrixU() * diag.asDiagonal() * svd.matrixV().transpose();
    }

    Vector3d translation = mu_image_points - rotation * mu_cloud_points;
    AngleAxisd affine;
    affine.fromRotationMatrix(rotation);
    R_t.resize(6);
    R_t.head(3) = affine.angle() * affine.axis();
    R_t.tail(3) = translation;
    return;
}




void draw_axis(cv::Mat& img, cv::Mat& rvec, cv::Mat& tvec, cv::Mat K, cv::Mat dist)
{
    std::vector<cv::Point3d> objectPoints;
    objectPoints.push_back(cv::Point3d(0,0,0));
    objectPoints.push_back(cv::Point3d(0.5,0,0));
    objectPoints.push_back(cv::Point3d(0,0.5,0));
    objectPoints.push_back(cv::Point3d(0,0,0.5));
    std::vector<cv::Point2d> imagePoints;
    cv::projectPoints(objectPoints, rvec, tvec, K , dist, imagePoints);
    cv::line(img, imagePoints[0], imagePoints[1], cv::Scalar(255, 0, 0),2);
    cv::line(img, imagePoints[0], imagePoints[2], cv::Scalar(0, 255, 0),2);
    cv::line(img, imagePoints[0], imagePoints[3], cv::Scalar(0, 0, 255),2);
}

void draw_corners_in_image(cv::Mat& img, std::vector<cv::Point3d>& corners, cv::Mat& rvec, cv::Mat& tvec, cv::Mat K, cv::Mat dist)
{
    std::vector<cv::Point2d> image_corners;
    cv::projectPoints(corners, rvec, tvec, K , dist, image_corners);
    //debug
    // if(1){
    //     for(auto& corner:image_corners){
    //         std::cout<<corner<<std::endl;
    //     }
    //     std::cout<<std::endl;
    // }
    for(auto& corner: image_corners){
        cv::circle( img, corner, 5, cv::Scalar( 0, 0, 255 ), cv::FILLED,cv::LINE_8 );
    }
}

