#include "optimization.hpp"
#include "utils.hpp"
#include "extractImageFeature.hpp"
#include <bits/stdc++.h>
using namespace Eigen;
using namespace std;

void generate_simulation_points(std::vector<Eigen::Vector3d>& p1, std::vector<Eigen::Vector3d>& p2,int n)
{
    p1.clear();
    p2.clear();
    std::vector<Eigen::Vector3d> modelP;
    double a = 1,b = 2.,c = 3.;//椭球参数
    for(auto i = -1.4;i<=1.4;i+=0.28){
        for(auto j= -3.1;j<=3.1;j+=0.62){
            double x = a*cos(i)*cos(j);
            double y = b*cos(i)*sin(j);
            double z = c*sin(i);
            modelP.push_back(Vector3d(x,y,z));
        }
    }
    
    Vector3d angles = {0.5,0.5,0.5};
    Vector3d translation = {0.5,0.5,0.5};
    std::default_random_engine dre;
    std::normal_distribution<double> noise(0,0.001);
    vector<Vector3d> modelT;
    Matrix3d R;
    R = AngleAxisd(angles(0), Vector3d::UnitX())
        * AngleAxisd(angles(1),  Vector3d::UnitY())
        * AngleAxisd(angles(2), Vector3d::UnitZ());
    // cout<< "original rotation: \n"<< R <<endl;
    // cout<< "original translation: "<< translation[0]<<" "<<translation[1]<<" "<<translation[2]<<endl;
    for(int i=0;i<modelP.size();i++){
        Vector3d xyz = R*modelP[i] + translation;
        modelT.push_back(xyz);
    }
    if(n>modelP.size()){
        n = modelP.size();
    }
    int m = modelP.size();
    int k = m/(n-1);
    for(int i=0;i<n*k;i+=k){
        p1.push_back(modelP[i]);
        p2.push_back(modelT[i]);
    }
    return;
}

typedef std::vector<pcl::PointXYZ> A;
typedef std::vector<A> Cloud;

bool get_sim_data(Cloud&source_cloud, Cloud&target_cloud)
{
    double image_points[96] = {
        -0.7703, -0.5006, 3.1952,
        0.8134, -0.6382, 3.0134,
        0.9016, 0.3578, 3.0281,
        -0.6820, 0.4954, 3.2099,
        -1.0544, 0.0117, 3.2396,
        0.3138, -0.7995, 3.0669,
        0.8201, 0.0621, 3.0308,
        -0.5481, 0.8733, 3.2035,
        -0.3291, 0.0060, 4.3576,
        1.2688, -0.0234, 4.2810,
        1.2882, 0.9762, 4.3029,
        -0.3097, 1.0056, 4.3795,
        -1.6068, 0.0315, 4.4608,
        -0.0089, 0.0021, 4.3855,
        0.0105, 1.0017, 4.4067,
        -1.5875, 1.0310, 4.4820,
        -1.1078, -0.0327, 4.4363,
        0.3516, -0.6833, 4.3538,
        0.7589, 0.2300, 4.3553,
        -0.7006, 0.8806, 4.4379,
        -1.0805, -0.0857, 4.2142,
        0.3869, -0.6525, 4.5066,
        0.7339, 0.2827, 4.5778,
        -0.7336, 0.8494, 4.2854,
        -0.0944, -0.9164, 4.3910,
        1.5043, -0.9354, 4.4535,
        1.5159, 0.0645, 4.4618,
        -0.0828, 0.0835, 4.3993,
        -0.2309, -0.4682, 4.3919,
        1.1616, -1.2548, 4.4398,
        1.6526, -0.3840, 4.4640,
        0.2602, 0.4026, 4.4160};

    for (int i = 0; i < 8; ++i)
    {
        std::vector<pcl::PointXYZ> four_points;
        for (int j = 0; j < 4; ++j)
        {
            four_points.push_back(pcl::PointXYZ(image_points[i * 12 + j * 3], image_points[i * 12 + j * 3 + 1], image_points[i * 12 + j * 3 + 2]));
        }
        target_cloud.push_back(four_points);
    }

  double lidar_points[96] = {
      3.9850, 0.7090, -0.5623,
      3.8054, -0.8685, -0.3142,
      3.6742, -1.0092, -1.3039,
      3.8538, 0.5683, -1.5520,
      3.9541, 0.9596, -1.1076,
      3.9022, -0.3455, -0.1898,
      3.7290, -0.9119, -1.0050,
      3.7810, 0.3932, -1.9228,
      5.0500, 0.2255, -1.2348,
      4.9760, -1.3686, -1.0804,
      4.8473, -1.4587, -2.0723,
      4.9213, 0.1354, -2.2267,
      5.1623, 1.4955, -1.3561,
      5.0882, -0.0995, -1.2087,
      4.9567, -0.1849, -2.1986,
      5.0308, 1.4101, -2.3460,
      5.1521, 0.9968, -1.2602,
      5.1499, -0.4158, -0.5071,
      5.0160, -0.8830, -1.3840,
      5.0182, 0.5296, -2.1370,
      4.9240, 0.9731, -1.1636,
      5.3011, -0.4551, -0.5573,
      5.2443, -0.8593, -1.4741,
      4.8672, 0.5689, -2.0804,
      5.2564, 0.0427, -0.3212,
      5.2953, -1.5545, -0.1969,
      5.1212, -1.6357, -1.1851,
      5.0823, -0.0385, -1.3095,
      5.1529, 0.1451, -0.7607,
      5.3466, -1.1917, 0.0942,
      5.2299, -1.7407, -0.7379,
      5.0362, -0.4039, -1.5927};
    for (int i = 0; i < 8; ++i)
    {
        std::vector<pcl::PointXYZ> four_points;
        for (int j = 0; j < 4; ++j)
        {
            four_points.push_back(pcl::PointXYZ(lidar_points[i * 12 + j * 3], lidar_points[i * 12 + j * 3 + 1], lidar_points[i * 12 + j * 3 + 2]));
        }
        source_cloud.push_back(four_points);
    }
    return true;
}

// void test_point_to_line_error()
// {
//     Eigen::Vector3d line_pt(0,0,0);
//     Eigen::Vector3d line_dir(1,1,0);
//     std::vector<Eigen::Vector3d> pts;
//     std::default_random_engine dre;
//     std::normal_distribution<double> noise(0,0.001);
//     Eigen::VectorXd Rt_gt(6);
//     Rt_gt<< 0.1, 0.1, 0.1, 0.1, 0.1, 0.1;
//     Eigen::Quaternion<double> q;  
//     q = AngleAxis<double>(Rt_gt(0), Eigen::Vector3d::UnitX())* 
//         AngleAxis<double>(Rt_gt(1), Eigen::Vector3d::UnitX())*
//         AngleAxis<double>(Rt_gt(2), Eigen::Vector3d::UnitX());
//     Eigen::Matrix3d rot = q.matrix();
//     for(int i=0;i<10;++i){
//         Eigen::Vector3d pt(i*0.1+noise(dre),i*0.1+noise(dre), noise(dre));
//         pt = rot*pt;
//         pt += Eigen::Vector3d(Rt_gt(3), Rt_gt(4),Rt_gt(5));
//         pts.push_back(pt);
//     }

//     Eigen::VectorXd R_t(6);
//     R_t << 0., 0., 0., 0., 0., 0.;
//     OptimizationLC optimizer_lc(R_t);
//     ceres::Problem problem;
//     ceres::LossFunction *loss_function = NULL;
//     double w = 1 / sqrt((double)pts.size());
//     for(auto& pt:pts){
//         ceres::CostFunction *cost_function = new ceres::AutoDiffCostFunction<PointToLineError, 1, 6>(new PointToLineError(pt, line_pt, line_dir, w));
//         problem.AddResidualBlock(cost_function, loss_function, R_t.data());
//     }

//     ceres::Solver::Options options;
//     options.max_num_iterations = 200;
//     options.linear_solver_type = ceres::SPARSE_NORMAL_CHOLESKY;
//     options.minimizer_progress_to_stdout = false;
//     ceres::Solver::Summary summary;
//     ceres::Solve(options, &problem, &summary);
//     std::cout << summary.BriefReport() << "\n";
//     std::cout<< R_t<<std::endl;
// }

void runs1()
{
    std::string left_image_path = "/home/jc/Documents/stereo-lidar-calibration/exclude_dir/lcc/HDL64/images/0001.png";
    std::string right_image_path = "/home/jc/Documents/stereo-lidar-calibration/exclude_dir/lcc/HDL64/images/0002.png";
    cv::Mat left_image = cv::imread(left_image_path, cv::IMREAD_COLOR);
    cv::Mat right_image = cv::imread(right_image_path, cv::IMREAD_COLOR);
    cv::Mat camera_matrix = cv::Mat::eye(3, 3, CV_64F);
    camera_matrix.at<double>(0,0) = 1.614546232069338e+03;
    camera_matrix.at<double>(0,2) = 6.412276358621397e+02;
    camera_matrix.at<double>(1,1) = 1.614669013419422e+03;
    camera_matrix.at<double>(1,2) = 4.801410561665820e+02;
    cv::Mat dist_coeffs = cv::Mat::zeros(5, 1, CV_64F);
    dist_coeffs.at<double>(0,0) = -0.004497294509341;
    dist_coeffs.at<double>(1,0) = 0.020426051162860;
    double square_size = 0.2;
    ImageFeatureDetector image_feature_detector_left(camera_matrix, dist_coeffs, square_size);
    std::vector<cv::Point2f> left_image_corners;
    if(!image_feature_detector_left.detectImageCorner(left_image, left_image_corners))
        return ;

    ImageFeatureDetector image_feature_detector_right(camera_matrix, dist_coeffs, square_size);
    std::vector<cv::Point2f> right_image_corners;
    if(!image_feature_detector_right.detectImageCorner(right_image, right_image_corners))
        return ;

    Eigen::VectorXd Rt_c1_c2(6);
    Rt_c1_c2 << 0.1, 2, 1, 0.1, 1.0, 0.1;
    OptimizationLCC optimizer_lcc(Rt_c1_c2);
    ceres::Problem problem;
    // std::cout<< left_image_corners.size()<<std::endl;
    // for(auto& corner:left_image_corners){
    //     std::cout<< corner << std::endl;
    // }
    // std::cout<< right_image_corners.size()<<std::endl;
    // for(auto& corner:right_image_corners){
    //     std::cout<< corner << std::endl;
    // }
    optimizer_lcc.addStereoMatchingConstraints(problem, left_image_corners, right_image_corners,camera_matrix);
    ceres::Solver::Options options;
    options.max_num_iterations = 500;
    options.linear_solver_type = ceres::SPARSE_NORMAL_CHOLESKY;
    options.minimizer_progress_to_stdout = false;
    ceres::Solver::Summary summary;
    ceres::Solve(options, &problem, &summary);
    std::cout << summary.BriefReport() << "\n";
    cout << optimizer_lcc.get_Rt_c1_c2() << std::endl;

    // for(auto& corner:image_corners){
    //     std::cout<<corner<<std::endl;
    // }
    // cv::Mat rvec, tvec;
    // image_feature_detector.estimatePose(image_corners, rvec, tvec);
    // std::vector<cv::Point3d> chessboard_3d_corners;
    // image_feature_detector.calculate3DCorners(chessboard_3d_corners, rvec, tvec);
 
    // for(auto& corner:chessboard_3d_corners){
    //     std::cout<< corner<<std::endl;
    //     cv::Point2d p = project(corner,camera_matrix);
    //     cv::circle(img, p, 5, cv::Scalar(0, 255, 0), -1);
    // }
    // cv::Point3d ori_p(tvec);
    // cv::Point2d p = project(ori_p,camera_matrix);
    // cv::circle(img, p, 5, cv::Scalar(0, 255, 0), -1);
    // std::cout<< rvec <<std::endl;
    // std::cout<< tvec <<std::endl;
    return;
}

void runs2()
{
    Eigen::VectorXd Rt1(6);
    Rt1<< 0.1, 0.1,0.1, 0.1, 0.1,0.1;
    Eigen::Affine3d aff1;
    vector2Affine(Rt1,aff1);
    std::cout<< aff1.matrix()<<std::endl;
    affine2Vector(aff1,Rt1);
    std::cout<< Rt1<<std::endl;
    Eigen::VectorXd Rt2(6);
    Rt2<< 0.2, 0.2, 0.2, 0.2, 0.2,0.2;
    Eigen::Affine3d aff2, aff3;
    vector2Affine(Rt2, aff2);
    aff3 = aff2.inverse()* aff1;
    std::cout<<aff3.matrix()<<std::endl;
}

int main()
{
    runs2();
    // // test_point_to_line_error();
    // Eigen::VectorXd R_t(6);
    // R_t << 0., 0., 0., 0., 0., 0.;
    // OptimizationLC optimizer_lc(R_t);
    // ceres::Problem problem;
    // // problem.AddParameterBlock(R_t.data(), 6);
    // // std::vector<Eigen::Vector3d> p1, p2;
    // // generate_simulation_points(p1, p2, 4);
    // Cloud source_cloud;
    // Cloud target_cloud;
    // get_sim_data(source_cloud,target_cloud);
    // for(int i=0;i<8;i++){
    //     auto& p1 = source_cloud[i];
    //     auto& p2 = target_cloud[i];
    //     optimizer_lc.addPointToPointConstriants(problem, p1, p2);
    // }

    // // for(auto& p:p1){
    // //     std::cout<<p<<std::endl;
    // // }
    // // for(auto& p:p2){
    // //     std::cout<<p<<std::endl;
    // // }
    // // optimizer_lc.addPointToPointConstriants(problem, p1, p2);
    // ceres::Solver::Options options;
    // options.max_num_iterations = 200;
    // options.linear_solver_type = ceres::SPARSE_NORMAL_CHOLESKY;
    // options.minimizer_progress_to_stdout = false;
    // ceres::Solver::Summary summary;
    // ceres::Solve(options, &problem, &summary);
    // std::cout << summary.BriefReport() << "\n";
    // R_t = optimizer_lc.get_R_t();

    // cout << optimizer_lc.get_R_t() << std::endl;

    // icp求解
    // pcl::PointCloud<pcl::PointXYZ>::Ptr source(new pcl::PointCloud<pcl::PointXYZ>);
    // pcl::PointCloud<pcl::PointXYZ>::Ptr target(new pcl::PointCloud<pcl::PointXYZ>);
    // Eigen::Matrix4d trans;
    // for(auto& image_corners:target_cloud){
    //     for(auto& corner: image_corners){
    //         target->points.push_back(corner);
    //     }
    // }
    // for(auto& lidar_corners:source_cloud){
    //     for(auto& corner: lidar_corners){
    //         source->points.push_back(corner);
    //     }
    // }
    // if(!computeTransICP(source,target, trans)){
    //     return -1;
    // }

    // std::cout<<trans<<std::endl;
    return 0;
}