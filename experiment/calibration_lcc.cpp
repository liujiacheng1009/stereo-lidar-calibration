#include "extractChessboard.hpp"
#include "extractImageFeature.hpp"
#include "extractLidarFeature.hpp"
#include "optimization.hpp"
#include "utils.hpp"
#include <sophus/so3.hpp>

using namespace std;
using namespace Eigen;
using namespace cv;

int main()
{
    cv::Mat camera_matrix = cv::Mat::eye(3, 3, CV_64F);
    camera_matrix.at<double>(0,0) = 1626.09707260559;
    camera_matrix.at<double>(0,2) = 639.989948488609;
    camera_matrix.at<double>(1,1) = 1626.19155093010;
    camera_matrix.at<double>(1,2) = 480.466809755920;
    cv::Mat dist_coeffs = cv::Mat::zeros(5, 1, CV_64F);
    dist_coeffs.at<double>(0,0) = 0.000201871191235612;
    dist_coeffs.at<double>(1,0) = -0.000726558404685396;
    double square_size = 0.2;
    	
    // 获得对应的左右目、点云路径
    std::vector<std::string> left_images_paths, right_images_paths, lidar_clouds_paths;
    std::string dataset_path = "/home/jc/Documents/stereo-lidar-calibration/exclude_dir/lcc/mydata";
    std::string left_images_dir = dataset_path + "/left_images";
    std::string right_images_dir = dataset_path + "/right_images";
    std::string lidar_clouds_dir = dataset_path + "/pointclouds";
    std::vector<std::string> left_images, right_images;
    get_data_by_path(left_images, left_images_dir, ".png");
    get_data_by_path(right_images, right_images_dir, ".png");
    sort(left_images.begin(), left_images.end());
    sort(right_images.begin(), right_images.end());
    std::vector<std::string> lidar_clouds;
    get_data_by_path(lidar_clouds, lidar_clouds_dir, ".pcd");
    sort(lidar_clouds.begin(), lidar_clouds.end());
    // for(auto& image:left_images){
    //     std::cout<< image <<std::endl;
    // }
    // for(auto& image:right_images){
    //     std::cout<< image <<std::endl;
    // }
    // for(auto& lidar_cloud:lidar_clouds){
    //     std::cout<< lidar_cloud <<std::endl;
    // }
    if(!check_data_by_index(left_images, lidar_clouds)) return -1;
    if(!check_data_by_index(right_images, lidar_clouds)) return -1;


    // 提取左右目及点云特征
    // 左目， 包括四个角点、平面和边缘直线
    ImageFeatureDetector left_image_feature_detector(camera_matrix, dist_coeffs, square_size);
    std::vector<int> valid_left_image_index;
    std::vector<std::vector<cv::Point3d>> left_image_3d_corners; // 图像3d角点
    vector<VectorXd> left_image_planes; // 图像的平面方程
    vector<vector<VectorXd>> left_image_lines; // 图像边缘直线方程
    vector<vector<Point2f>> left_image_corners;
    for (int i = 0;i<left_images.size();++i)
    {
        cv::Mat img = cv::imread(left_images[i], cv::IMREAD_COLOR);
        cv::Mat half_image;
        //cv::resize(img, img , cv::Size(0.5, 0.5), cv::INTER_LINEAR);
        cv::resize(img, half_image, cv::Size(), 0.45, 0.45); // cv::findChessboardCorners在高分辨率图像上有bug
        std::vector<cv::Point2f> image_corners;
        if (!left_image_feature_detector.detectImageCorner(half_image, image_corners)){
            std::cout<< "can not detect corner from image: " << i<<std::endl;
            continue;
        }
        for(auto& image_corner:image_corners){
            image_corner.x *= (1.0/0.45);
            image_corner.y *= (1.0/0.45);
        }
        cv::Mat rvec, tvec;
        left_image_feature_detector.estimatePose(image_corners, rvec, tvec);
        // std::cout<<"tvec: " <<tvec<<std::endl;
        std::vector<cv::Point3d> chessboard_3d_corners, reordered_image_3d_corners;
        left_image_feature_detector.calculate3DCorners(chessboard_3d_corners, rvec, tvec);
        // reorder_corners(chessboard_3d_corners, reordered_image_3d_corners);
        VectorXd plane;
        left_image_feature_detector.calculatePlane1(chessboard_3d_corners, plane);
        vector<VectorXd> lines;
        left_image_feature_detector.calculateLines(chessboard_3d_corners, lines);
        left_image_lines.push_back(lines);
        left_image_planes.push_back(plane);
        left_image_3d_corners.push_back(chessboard_3d_corners);
        left_image_corners.push_back(image_corners);
        valid_left_image_index.push_back(i);
    }
    // 右目， 包括四个角点、平面和边缘直线
    ImageFeatureDetector right_image_feature_detector(camera_matrix, dist_coeffs, square_size);
    std::vector<int> valid_right_image_index;
    std::vector<std::vector<cv::Point3d>> right_image_3d_corners; // 图像3d角点
    vector<VectorXd> right_image_planes; // 图像的平面方程
    vector<vector<VectorXd>> right_image_lines; // 图像边缘直线方程
    vector<vector<Point2f>> right_image_corners; // 
    for (int i = 0;i<right_images.size();++i)
    {
        cv::Mat img = cv::imread(right_images[i], cv::IMREAD_COLOR);
        cv::Mat half_image;
        //cv::resize(img, img , cv::Size(0.5, 0.5), cv::INTER_LINEAR);
        cv::resize(img, half_image, cv::Size(), 0.45, 0.45); // cv::findChessboardCorners在高分辨率图像上有bug
        std::vector<cv::Point2f> image_corners;
        if (!right_image_feature_detector.detectImageCorner(half_image, image_corners)){
            std::cout<< "can not detect corner from image: " << i<<std::endl;
            continue;
        }
        for(auto& image_corner:image_corners){
            image_corner.x *= (1.0/0.45);
            image_corner.y *= (1.0/0.45);
        }
        cv::Mat rvec, tvec;
        right_image_feature_detector.estimatePose(image_corners, rvec, tvec);
        // std::cout<<"tvec: " <<tvec<<std::endl;
        std::vector<cv::Point3d> chessboard_3d_corners, reordered_image_3d_corners;
        right_image_feature_detector.calculate3DCorners(chessboard_3d_corners, rvec, tvec);
        // reorder_corners(chessboard_3d_corners, reordered_image_3d_corners); 不需要
        VectorXd plane;
        right_image_feature_detector.calculatePlane1(chessboard_3d_corners, plane);
        vector<VectorXd> lines;
        right_image_feature_detector.calculateLines(chessboard_3d_corners, lines);
        right_image_lines.push_back(lines);
        right_image_planes.push_back(plane);
        right_image_3d_corners.push_back(chessboard_3d_corners);
        right_image_corners.push_back(image_corners);
        valid_right_image_index.push_back(i);
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
    
    // 点云， checkerboarder分割
    PassFilterParams pass_filter_params(std::vector<double>({-10,10,-10,10,-10,10}));
    ChessboardExtractor extractor;
    std::vector<int> valid_cloud_index, valid_cloud_index1;

    std::vector<pcl::PointCloud<pcl::PointXYZ>> plane_clouds;
    for(int i=0;i<lidar_clouds.size();++i)
    {
        pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud(new pcl::PointCloud<pcl::PointXYZ>);
        if (pcl::io::loadPCDFile<pcl::PointXYZ> (lidar_clouds[i], *input_cloud) == -1) continue;
        extractor.pass_filter(input_cloud, pass_filter_params);
        std::vector<pcl::PointIndices> indices_clusters;
        extractor.pcd_clustering(input_cloud, indices_clusters);
        auto& plane_pcd = input_cloud;
        if(!extractor.fitPlane(plane_pcd, indices_clusters)) continue;
        valid_cloud_index.push_back(i);
        plane_clouds.push_back(*plane_pcd);
       // display_colored_by_depth(plane_pcd);
    }
    

   // 点云，平面、角点、直线特征提取 
    LidarFeatureDetector lidar_feature_detector;
    std::vector<std::vector<pcl::PointXYZ>> cloud_3d_corners;
    std::vector<std::vector<pcl::PointCloud<pcl::PointXYZ>>> lines_pcds;
    for(int i=0;i<plane_clouds.size();++i){
        auto input_cloud = plane_clouds[i].makeShared();
        pcl::PointCloud<pcl::PointXYZ>::Ptr edge_pcd(new pcl::PointCloud<pcl::PointXYZ>);
        lidar_feature_detector.extractEdgeCloud(input_cloud, edge_pcd);
        std::vector<Eigen::VectorXf> lines_params;
        std::vector<pcl::PointCloud<pcl::PointXYZ>> lines_points;
        if (!lidar_feature_detector.extractFourLines(edge_pcd, lines_params, lines_points))
        {
            continue;
        }
        std::vector<pcl::PointXYZ> corners, reordered_corners;
        lidar_feature_detector.estimateFourCorners(lines_params,corners );
        vector<pair<std::pair<pcl::PointXYZ, pcl::PointXYZ> , pcl::PointCloud<pcl::PointXYZ>>> line_corners_to_points;
        for(int i=0;i<4;i++){
            line_corners_to_points.push_back(make_pair(make_pair(corners[i], corners[(i+1)%4]), lines_points[(i+1)%4]));
        }
        reorder_corners(corners, reordered_corners);
        lines_points.clear();
        for(int i=0;i<4;++i){
            auto p1 = make_pair(reordered_corners[i], reordered_corners[(i+1)%4]);
            auto p2 = make_pair(reordered_corners[(i+1)%4], reordered_corners[i]);
            for(int j=0;j<4;++j){
                auto& corners_points = line_corners_to_points[j];
                if(pcl::geometry::distance(corners_points.first.first, p1.first)< 1e-2 && pcl::geometry::distance(corners_points.first.second, p1.second)<1e-2){
                    lines_points.push_back(corners_points.second);
                }
                if(pcl::geometry::distance(corners_points.first.first, p2.first)< 1e-2 && pcl::geometry::distance(corners_points.first.second, p2.second)<1e-2){
                    lines_points.push_back(corners_points.second);
                }
            }
        }
        lines_pcds.push_back(lines_points);
        cloud_3d_corners.push_back(reordered_corners);
        valid_cloud_index1.push_back(valid_cloud_index[i]);
    }

    // 左目与lidar标定
    Eigen::VectorXd Rt_l1_c1(6);
    Rt_l1_c1 << 0., 0., 0., 0., 0., 0.;
    OptimizationLC optimizer_lc1(Rt_l1_c1);
    ceres::Problem problem1;

    // 添加点到点的约束
    
    for(int i=0;i<left_image_3d_corners.size();++i){
        auto& image_corners = left_image_3d_corners[i];
        auto& cloud_corners = cloud_3d_corners[i];
        std::vector<pcl::PointXYZ> p1, p2;
        for(int j=0;j<4;++j){
            p1.push_back(pcl::PointXYZ(image_corners[j].x, image_corners[j].y, image_corners[j].z));
            p2.push_back(cloud_corners[j]);
        }
        optimizer_lc1.addPointToPointConstriants(problem1, p1, p2);
    }

    // 添加点到平面的约束
    for(int i=0;i<plane_clouds.size();++i ){
        auto plane_cloud = plane_clouds[i].makeShared();
        auto& plane = left_image_planes[i];
        Vector3d plane_centroid = plane.head<3>();
        Vector3d plane_normal = plane.tail<3>();
        optimizer_lc1.addPointToPlaneConstraints(problem1, plane_cloud, plane_centroid, plane_normal);
    }

    // 添加点到直线的约束
    for(int i=0;i<lines_pcds.size();++i){
        auto& lines_points = lines_pcds[i];
        auto& lines_params = left_image_lines[i];
        vector<VectorXd> lines_normal;
        for(int j=0;j<4;++j){
            auto& line_params = lines_params[j];
            Vector3d a = line_params.head<3>();
            Vector3d b = line_params.head<3>() + line_params.tail<3>();
            Vector3d line_normal = a.cross(b);
            lines_normal.push_back(line_normal);
        }
        optimizer_lc1.addPointToLineConstriants(problem1, lines_points,lines_normal);
    }

    ceres::Solver::Options options1;
    options1.max_num_iterations = 500;
    options1.linear_solver_type = ceres::SPARSE_NORMAL_CHOLESKY;
    options1.minimizer_progress_to_stdout = false;
    ceres::Solver::Summary summary1;
    ceres::Solve(options1, &problem1, &summary1);
    std::cout << summary1.BriefReport() << "\n";
    // Rt_l1_c1 = optimizer_lc1.get_R_t();
    cout << Rt_l1_c1 << std::endl;
    
    // 右目与lidar标定
    Eigen::VectorXd Rt_l1_c2(6);
    Rt_l1_c2 << 0., 0., 0., 0., 0., 0.;
    OptimizationLC optimizer_lc2(Rt_l1_c2);
    ceres::Problem problem2;

    // 添加点到点的约束
   
    for(int i=0;i<right_image_3d_corners.size();++i){
        auto& image_corners = right_image_3d_corners[i];
        auto& cloud_corners = cloud_3d_corners[i];
        std::vector<pcl::PointXYZ> p1, p2;
        for(int j=0;j<4;++j){
            p1.push_back(pcl::PointXYZ(image_corners[j].x, image_corners[j].y, image_corners[j].z));
            p2.push_back(cloud_corners[j]);
        }
        optimizer_lc2.addPointToPointConstriants(problem2, p1, p2);
    }

    // 添加点到平面的约束
    for(int i=0;i<plane_clouds.size();++i ){
        auto plane_cloud = plane_clouds[i].makeShared();
        auto& plane = right_image_planes[i];
        Vector3d plane_centroid = plane.head<3>();
        Vector3d plane_normal = plane.tail<3>();
        optimizer_lc2.addPointToPlaneConstraints(problem2, plane_cloud, plane_centroid, plane_normal);
    }

    // 添加点到直线的约束
    for(int i=0;i<lines_pcds.size();++i){
        auto& lines_points = lines_pcds[i];
        auto& lines_params = right_image_lines[i];
        vector<VectorXd> lines_normal;
        for(int j=0;j<4;++j){
            auto& line_params = lines_params[j];
            Vector3d a = line_params.head<3>();
            Vector3d b = line_params.head<3>() + line_params.tail<3>();
            Vector3d line_normal = a.cross(b);
            lines_normal.push_back(line_normal);
        }
        optimizer_lc2.addPointToLineConstriants(problem2, lines_points,lines_normal);
    }

    ceres::Solver::Options options2;
    options2.max_num_iterations = 500;
    options2.linear_solver_type = ceres::SPARSE_NORMAL_CHOLESKY;
    options2.minimizer_progress_to_stdout = false;
    ceres::Solver::Summary summary2;
    ceres::Solve(options2, &problem2, &summary2);
    std::cout << summary2.BriefReport() << "\n";
    // Rt_l1_c2 = optimizer_lc2.get_R_t();
    cout << Rt_l1_c2 << std::endl;

    Eigen::VectorXd Rt_c1_c2(6);
    Eigen::Affine3d affine_matrix_cl, affine_matrix_cr;
    vector2Affine(Rt_l1_c1, affine_matrix_cl);
    vector2Affine(Rt_l1_c2, affine_matrix_cr);
    Eigen::Affine3d affine_matrix_lr = affine_matrix_cl.inverse()*affine_matrix_cr;
    affine2Vector(affine_matrix_lr,Rt_c1_c2 );
    std::cout<< Rt_c1_c2 <<std::endl;

    // 联合标定
    OptimizationLCC optimizer_lcc(Rt_l1_c1,Rt_l1_c2,Rt_c1_c2);
    ceres::Problem problem;
    // 添加点到点的约束
    
    for(int i=0;i<left_image_3d_corners.size();++i){
        auto& image_corners = left_image_3d_corners[i];
        auto& cloud_corners = cloud_3d_corners[i];
        std::vector<pcl::PointXYZ> p1, p2;
        for(int j=0;j<4;++j){
            p1.push_back(pcl::PointXYZ(image_corners[j].x, image_corners[j].y, image_corners[j].z));
            p2.push_back(cloud_corners[j]);
        }
        optimizer_lcc.addPointToPointConstriants(problem, p1, p2,Rt_l1_c1);
    }

    // 添加点到平面的约束
    for(int i=0;i<plane_clouds.size();++i ){
        auto plane_cloud = plane_clouds[i].makeShared();
        auto& plane = left_image_planes[i];
        Vector3d plane_centroid = plane.head<3>();
        Vector3d plane_normal = plane.tail<3>();
        optimizer_lcc.addPointToPlaneConstraints(problem, plane_cloud, plane_centroid, plane_normal,Rt_l1_c1);
    }

    // 添加点到直线的约束
    for(int i=0;i<lines_pcds.size();++i){
        auto& lines_points = lines_pcds[i];
        auto& lines_params = left_image_lines[i];
        vector<VectorXd> lines_normal;
        for(int j=0;j<4;++j){
            auto& line_params = lines_params[j];
            Vector3d a = line_params.head<3>();
            Vector3d b = line_params.head<3>() + line_params.tail<3>();
            Vector3d line_normal = a.cross(b);
            lines_normal.push_back(line_normal);
        }
        optimizer_lcc.addPointToLineConstriants(problem, lines_points,lines_normal, Rt_l1_c1);
    }

    // 添加点到点的约束
    
    for(int i=0;i<right_image_3d_corners.size();++i){
        auto& image_corners = right_image_3d_corners[i];
        auto& cloud_corners = cloud_3d_corners[i];
        std::vector<pcl::PointXYZ> p1, p2;
        for(int j=0;j<4;++j){
            p1.push_back(pcl::PointXYZ(image_corners[j].x, image_corners[j].y, image_corners[j].z));
            p2.push_back(cloud_corners[j]);
        }
        optimizer_lcc.addPointToPointConstriants(problem, p1, p2,Rt_l1_c2);
    }

    // 添加点到平面的约束
    for(int i=0;i<plane_clouds.size();++i ){
        auto plane_cloud = plane_clouds[i].makeShared();
        auto& plane = right_image_planes[i];
        Vector3d plane_centroid = plane.head<3>();
        Vector3d plane_normal = plane.tail<3>();
        optimizer_lcc.addPointToPlaneConstraints(problem, plane_cloud, plane_centroid, plane_normal, Rt_l1_c2);
    }

    // 添加点到直线的约束
    for(int i=0;i<lines_pcds.size();++i){
        auto& lines_points = lines_pcds[i];
        auto& lines_params = right_image_lines[i];
        vector<VectorXd> lines_normal;
        for(int j=0;j<4;++j){
            auto& line_params = lines_params[j];
            Vector3d a = line_params.head<3>();
            Vector3d b = line_params.head<3>() + line_params.tail<3>();
            Vector3d line_normal = a.cross(b);
            lines_normal.push_back(line_normal);
        }
        optimizer_lcc.addPointToLineConstriants(problem2, lines_points,lines_normal,Rt_l1_c2);
    }

    for(int i=0;i<left_image_corners.size();i++){
        auto& left_corners = left_image_corners[i];
        auto& right_corners = right_image_corners[i];
        optimizer_lcc.addStereoMatchingConstraints(problem, left_corners, right_corners,camera_matrix);
    }

    ceres::Solver::Options options;
    options.max_num_iterations = 500;
    options.linear_solver_type = ceres::SPARSE_NORMAL_CHOLESKY;
    options.minimizer_progress_to_stdout = false;
    ceres::Solver::Summary summary;
    ceres::Solve(options, &problem, &summary);
    std::cout << summary.BriefReport() << "\n";
    cout << optimizer_lcc.get_Rt_l1_c1() << std::endl;
    cout << optimizer_lcc.get_Rt_l1_c2() << std::endl;
    cout << optimizer_lcc.get_Rt_c1_c2() << std::endl;
    return 0;
}