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

int main()
{
    Config config;
    std::vector<std::string> images_paths, lidar_clouds_paths;
    std::string images_dir = config.left_images_dataset_path;
    std::string clouds_dir = config.lidar_clouds_dataset_path;
    std::vector<std::string> images;
    get_data_by_path(images, images_dir, config.image_format);
    std::vector<std::string> lidar_clouds;
    get_data_by_path(lidar_clouds, clouds_dir, config.cloud_format);
    if(!check_data_by_index(images, lidar_clouds)){
        cerr <<"图像和点云文件应存在且文件名相同！！"<<endl;
        exit(-1);
    }
    ImageFeatureDetector image_feature_detector(config);
    vector<int> valid_image_index;
    std::vector<std::vector<cv::Point3d>> image_3d_corners; // 图像3d角点
    vector<Eigen::VectorXd> image_planes; // 图像的平面方程
    vector<vector<Eigen::VectorXd>> image_lines; // 图像边缘直线方程
    for (int i = 0;i<images.size();++i)
    {
        cv::Mat img = cv::imread(images[i], cv::IMREAD_COLOR);
        cv::Mat half_image;
        //cv::resize(img, img , cv::Size(0.5, 0.5), cv::INTER_LINEAR);
        cv::resize(img, half_image, cv::Size(), 0.45, 0.45); // cv::findChessboardCorners在高分辨率图像上有bug
        std::vector<cv::Point2f> image_corners;
        if (!image_feature_detector.detectImageCorner(half_image, image_corners)){
            std::cout<< "can not detect corner from image: " << i<<std::endl;
            continue;
        }
        for(auto& image_corner:image_corners){
            image_corner.x *= (1.0/0.45);
            image_corner.y *= (1.0/0.45);
        }
        cv::Mat rvec, tvec;
        image_feature_detector.estimatePose(image_corners, rvec, tvec);
        std::vector<Point3d> chessboard_3d_corners, reordered_image_3d_corners;
        image_feature_detector.calculate3DCorners(chessboard_3d_corners, rvec, tvec);
        Eigen::VectorXd plane;
        image_feature_detector.calculatePlane1(chessboard_3d_corners, plane);
        vector<Eigen::VectorXd> lines;
        image_feature_detector.calculateLines(chessboard_3d_corners, lines);
        image_lines.push_back(lines);
        image_planes.push_back(plane);
        image_3d_corners.push_back(chessboard_3d_corners);
        valid_image_index.push_back(i);
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
    // std::cout<<plane_clouds.size()<<std::endl;
    
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
        // // debug
        // {
        //     if(i==3){
        //         for(auto& params:lines_params){
        //             std::cout<<params<<std::endl;
        //             std::cout<<std::endl;
        //         }
        //     }
        // }

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
        // for (auto &corner : reordered_corners)
        // {
        //     std::cout << corner.x << " "
        //               << corner.y << " "
        //               << corner.z << std::endl;
        //     std::cout << std::endl;
        // }
    }
    // auto& lines_pcd = lines_pcds[2];
    // for(auto& line:lines_pcd){
    //     std::cout<<line.width*line.height<<std::endl;
    // }
    // auto viewer = show_multi_clouds_with_specified_colors(lines_pcd);
    // while (!viewer->wasStopped())
    // {
    //     viewer->spinOnce(100);
    // }

    // auto& lines_pcd1 = lines_pcds[3];
    // for(auto& line:lines_pcd1){
    //     std::cout<<line.width*line.height<<std::endl;
    // }
    // auto viewer1 = show_multi_clouds_with_specified_colors(lines_pcd1);
    // while (!viewer1->wasStopped())
    // {
    //     viewer1->spinOnce(100);
    // }

    // auto& corners = cloud_3d_corners[3];
    // for(auto& corner:corners){
    //     std::cout<<corner.x<<" "
    //         <<corner.y<<" "
    //         <<corner.z<<std::endl;
    //     std::cout<<std::endl;
    // }
    // std::cout<<cloud_3d_corners.size()<<std::endl;

    // for(int i=0;i<cloud_3d_corners.size();++i){
    //     auto& corners = cloud_3d_corners[i];
    //     for(auto& corner:corners){
    //         std::cout<<corner<<std::endl;
    //     }
    //     std::cout<<"---------------------"<<std::endl;
    // }

    

    // 显示角点
    // std::cout<< cloud_3d_corners.size()<<std::endl;
    // for (int i=0;i<cloud_3d_corners.size();++i)
    // {   
    //     auto& corners = cloud_3d_corners[i];
    //     auto input_cloud = plane_clouds[i].makeShared();
    //     pcl::PointCloud<pcl::PointXYZ>::Ptr corners_pcd(new pcl::PointCloud<pcl::PointXYZ>);
    //     for (auto &corner : corners)
    //     {
    //         corners_pcd->push_back(corner);
    //         //std::cout<<corner<<std::endl;
    //     }

    //     pcl::visualization::PCLVisualizer viewer("test");
    //     viewer.setBackgroundColor(0, 0, 0);

    //     pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> singleColor1(corners_pcd, 0, 255, 0); //0-255  设置成绿色
    //     pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> singleColor2(input_cloud, 255, 255, 0);
    //     viewer.addPointCloud<pcl::PointXYZ>(corners_pcd, singleColor1, "sample1"); //显示点云，其中fildColor为颜色显示
    //     viewer.addPointCloud<pcl::PointXYZ>(input_cloud, singleColor2, "sample2");
    //     viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, "sample1"); //设置点云大小
    //     viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "sample2");
    //     while (!viewer.wasStopped())
    //     {
    //         viewer.spinOnce();
    //     }
    // }

    // for (int i = 0; i < 4; i++)
    // {
    //     auto edge_pcd_ptr = lines_points[i].makeShared();
    //     display_colored_by_depth(edge_pcd_ptr);
    // }

    Eigen::VectorXd R_t(6);
    R_t << 0., 0., 0., 0., 0., 0.;
    OptimizationLC optimizer_lc(R_t);
    ceres::Problem problem;
    // problem.AddParameterBlock(R_t.data(), 6);
    std::vector<pcl::PointXYZ> p1, p2;
    for(int i=0;i<image_3d_corners.size();++i){
        auto& image_corners = image_3d_corners[i];
        auto& cloud_corners = cloud_3d_corners[i];
        p1.clear();
        p2.clear();
        for(int j=0;j<4;++j){
            p1.push_back(pcl::PointXYZ(image_corners[j].x, image_corners[j].y, image_corners[j].z));
            p2.push_back(cloud_corners[j]);
        }
        optimizer_lc.addPointToPointConstriants(problem, p1, p2);
    }

    // 添加点到平面的约束
    // for(int i=0;i<plane_clouds.size();++i ){
    //     auto plane_cloud = plane_clouds[i].makeShared();
    //     auto& plane = image_planes[i];
    //     Eigen::Vector3d plane_centroid = plane.head<3>();
    //     Eigen::Vector3d plane_normal = plane.tail<3>();
    //     optimizer_lc.addPointToPlaneConstraints(problem, plane_cloud, plane_centroid, plane_normal);
    // }

    // 添加点到直线的约束
    // for(int i=0;i<lines_pcds.size();++i){
    //     auto& lines_points = lines_pcds[i];
    //     auto& lines_params = image_lines[i];
    //     vector<Eigen::VectorXd> lines_normal;
    //     for(int j=0;j<4;++j){
    //         auto& line_params = lines_params[j];
    //         Eigen::Vector3d a = line_params.head<3>();
    //         Eigen::Vector3d b = line_params.head<3>() + line_params.tail<3>();
    //         Eigen::Vector3d line_normal = a.cross(b);
    //         lines_normal.push_back(line_normal);
    //     }
    //     optimizer_lc.addPointToLineConstriants(problem, lines_points,lines_normal);
    // }

    ceres::Solver::Options options;
    options.max_num_iterations = 500;
    options.linear_solver_type = ceres::SPARSE_NORMAL_CHOLESKY;
    options.minimizer_progress_to_stdout = false;
    ceres::Solver::Summary summary;
    ceres::Solve(options, &problem, &summary);
    std::cout << summary.BriefReport() << "\n";
    MatrixXd Rt = Matrix<double,4,4>::Identity();
    Eigen::Matrix3d R = Sophus::SO3d::exp(optimizer_lc.get_R_t().head(3)).matrix();
    Rt.block(0,0,3,3) = R;
    Rt.block(0,3,3,1) = optimizer_lc.get_R_t().tail(3);
    cout << Rt << std::endl;
    cout<< Rt.inverse()<<endl;
    
    // cout<<"-----------------------------------------"<<endl;
    // for(int i=0;i<n_frames;++i){
    //     for(int j = 0;j<n_corners;++j){
    //         auto& camera_point = right_camera_corners_3d[i][j];
    //         auto& lidar_point = lidar_corners_3d[i][j];
    //         Vector4d p1,p2;
    //         p1.head(3) = camera_point;
    //         p1(3) = 1;
    //         p2.head(3) = lidar_point;
    //         p2(3) = 1;
    //         Vector4d p3 = p1 - refined_Rt_l1_c2*p2;
    //         cout<< p1(0)<<" "<<p1(1)<<" "<<p1(2)<<endl<<
    //             p2(0)<<" "<<p2(1)<<" "<<p2(2)<<endl<<
    //             p3(0)<<" "<<p3(1)<<" "<<p3(2)<<endl;
    //         cout<<endl;
    //     }
    // }

    // icp求解
    // pcl::PointCloud<pcl::PointXYZ>::Ptr source_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    // pcl::PointCloud<pcl::PointXYZ>::Ptr target_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    // Eigen::Matrix4d trans;
    // for(auto& image_corners:image_3d_corners){
    //     for(auto& corner: image_corners){
    //         std::cout<<corner<<std::endl;
    //         target_cloud->points.push_back(pcl::PointXYZ(corner.x, corner.y, corner.z));
    //     }
    //     std::cout<<std::endl;
    // }
    // for(auto& lidar_corners:cloud_3d_corners){
    //     for(auto& corner: lidar_corners){
    //         std::cout<<corner<<std::endl;
    //         source_cloud->points.push_back(pcl::PointXYZ(corner.x, corner.y, corner.z));
    //     }
    //     std::cout<<std::endl;
    // }
    // if(!computeTransICP(source_cloud,target_cloud, trans)){
    //     return -1;
    // }
    // std::cout<<trans<<std::endl;

    // generate_simulation_points(p1, p2, 4);
    // for(auto& p:p1){
    //     std::cout<<p<<std::endl;
    // }
    // for(auto& p:p2){
    //     std::cout<<p<<std::endl;
    // }


    return 0;
}