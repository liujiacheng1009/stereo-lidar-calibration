#include "extractChessboard.hpp"
#include "extractImageFeature.hpp"
#include "extractLidarFeature.hpp"
#include "optimization.hpp"
#include "utils.hpp"
#include "config.hpp"
#include <pcl/common/geometry.h>
#include <sophus/so3.hpp>

using namespace std;
using namespace Eigen;
using namespace cv;
using namespace pcl;

int main()
{
    Config_stereo_left config_left;
    Config_stereo_right config_right;
    string left_images_dir = config_left.left_images_dataset_path;
    string right_images_dir = config_right.left_images_dataset_path;
    string clouds_dir = config_left.lidar_clouds_dataset_path;
    vector<string> left_images;
    get_data_by_path(left_images, left_images_dir, config_left.image_format);
    vector<string> right_images;
    get_data_by_path(right_images, right_images_dir, config_right.image_format);
    std::vector<std::string> clouds;
    get_data_by_path(clouds, clouds_dir, config_left.cloud_format);
    if(!check_data_by_index(left_images, clouds)){
        cerr <<"图像和点云文件应存在且文件名相同！！"<<endl;
        exit(-1);
    }
    if(!check_data_by_index(right_images, clouds)){
        cerr <<"图像和点云文件应存在且文件名相同！！"<<endl;
        exit(-1);
    }

    ImageResults left_images_features;
    processImage(config_left, left_images, left_images_features);
    cout<< "有效的图像帧有： "<<endl;
    for(auto& id:left_images_features.valid_index){
        cout<< id<<" ";
    }
    cout<<endl;

    ImageResults right_images_features;
    processImage(config_right, right_images, right_images_features);
    cout<< "有效的图像帧有： "<<endl;
    for(auto& id:right_images_features.valid_index){
        cout<< id<<" ";
    }
    cout<<endl;

    CloudResults cloud_features;
    processCloud(config_left, clouds, cloud_features);
    cout<< "有效的点云帧有： "<<endl;
    for(auto& id:cloud_features.valid_index){
        cout<< id<<" ";
    }
    cout<<endl;

    getValidDataSet(left_images_features,right_images_features, cloud_features);
    auto& left_image_3d_corners = left_images_features.corners_3d; // 图像3d角点
    auto& left_image_2d_corners = left_images_features.corners_2d;
    auto& right_image_3d_corners = right_images_features.corners_3d;
    auto& right_image_2d_corners = right_images_features.corners_2d;
    auto& cloud_3d_corners = cloud_features.corners_3d; // 点云3d角点

    VectorXd Rt_l1_c1(6), Rt_l1_c2(6), Rt_c1_c2(6);

    Rt_l1_c1<< 1.5,-1.31589,1.06346,-0.401976,0.301422,-0.130281;
    Rt_l1_c2<<  1.54296,-1.3,1.06319,-0.5,0.303772,-0.131603;
    Rt_c1_c2 << 0.0,0.0,0.0,-0.12,0.0,0.0;

    Optimizer optimizer;
    ceres::Problem problem;

    for(int i=0;i<left_image_3d_corners.size();++i){
        auto& image_corners = left_image_3d_corners[i];
        auto& cloud_corners = cloud_3d_corners[i];
        optimizer.addPointToPointConstriants(problem,cloud_corners,image_corners,Rt_l1_c1);
    }

    for(int i=0;i<right_image_3d_corners.size();++i){
        auto& image_corners = right_image_3d_corners[i];
        auto& cloud_corners = cloud_3d_corners[i];
        optimizer.addPointToPointConstriants(problem,cloud_corners,image_corners,Rt_l1_c2);
    }

    for(int i=0;i<left_image_2d_corners.size();i++){
        auto& left_corners = left_image_2d_corners[i];
        auto& right_corners = right_image_2d_corners[i];
        optimizer.addStereoMatchingConstraints(problem, left_corners, right_corners, Rt_c1_c2);
    }

    for(int i = 0;i<left_image_3d_corners.size();++i){
        optimizer.addClosedLoopConstraints(problem,Rt_l1_c1, Rt_l1_c2, Rt_c1_c2);
    }
    
    ceres::Solver::Options options;
    options.max_num_iterations = 500;
    options.linear_solver_type = ceres::SPARSE_NORMAL_CHOLESKY;
    options.minimizer_progress_to_stdout = false;
    ceres::Solver::Summary summary;
    ceres::Solve(options, &problem, &summary);
    std::cout << summary.BriefReport() << "\n";

    MatrixXd M_Rt_l1_c1 = Matrix<double,4,4>::Identity();
    Eigen::Matrix3d R1 = Sophus::SO3d::exp(Rt_l1_c1.head(3)).matrix();
    M_Rt_l1_c1.block(0,0,3,3) = R1;
    M_Rt_l1_c1.block(0,3,3,1) = Rt_l1_c1.tail(3);
    cout << M_Rt_l1_c1 << std::endl;
    cout << M_Rt_l1_c1-config_left.matlab_tform<<endl;
    cout<<"--------------------------"<<endl;
    MatrixXd M_Rt_l1_c2 = Matrix<double,4,4>::Identity();
    Eigen::Matrix3d R2 = Sophus::SO3d::exp(Rt_l1_c2.head(3)).matrix();
    M_Rt_l1_c2.block(0,0,3,3) = R2;
    M_Rt_l1_c2.block(0,3,3,1) = Rt_l1_c2.tail(3);
    cout << M_Rt_l1_c2 << std::endl;
    cout << M_Rt_l1_c2-config_right.matlab_tform<<endl;
    cout<<"--------------------------"<<endl;

    VectorXd V_Rt_c1_c2 = Rt_c1_c2;
    VectorXd Rt_c1_c2_gt(6);
    Rt_c1_c2_gt<<0.0,0.0,0.0,-0.12,0.0,0.0;
    cout << V_Rt_c1_c2 << std::endl;
    cout << V_Rt_c1_c2-Rt_c1_c2_gt<<endl;
    cout<<"--------------------------"<<endl;
    return 0;
}