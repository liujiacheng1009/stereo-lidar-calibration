#include <pcl/common/common.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <iostream>

#include "utils.hpp"
#include "extractLidarFeature.hpp"
#include "extractChessboard.hpp"

using namespace std;
using namespace Eigen;

void run1()
{
    std::vector<std::string> lidar_clouds_paths;
    std::string dataset_path = "/home/jc/Documents/stereo-lidar-calibration/exclude_dir/lcc/HDL64";
    std::string lidar_clouds_dir = dataset_path + "/pointCloud";
    std::vector<std::string> lidar_clouds;
    get_data_by_path(lidar_clouds, lidar_clouds_dir, ".pcd");
    sort(lidar_clouds.begin(), lidar_clouds.end());

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
        // for(int i=0;i<4;++i){
        //     line_points_and_params[lines_params[i]] = lines_points[i];
        // }
        std::vector<pcl::PointXYZ> corners, reordered_corners;
        lidar_feature_detector.estimateFourCorners(lines_params,corners);
        vector<pair<std::pair<pcl::PointXYZ, pcl::PointXYZ> , pcl::PointCloud<pcl::PointXYZ>>> line_corners_to_points;
        for(int i=0;i<4;i++){
            line_corners_to_points.push_back(make_pair(make_pair(corners[i], corners[(i+1)%4]), lines_points[(i+1)%4]));
        }
        reorder_corners(corners, reordered_corners); // 角点重排序
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


    // for(auto& corners: cloud_3d_corners){
    //     for (auto &corner : corners)
    //     {
    //         std::cout << corner.x << " "
    //                   << corner.y << " "
    //                   << corner.z << std::endl;
    //     }
    //     std::cout << std::endl;
    // }
    for(auto& lines_points: lines_pcds)
    {
        for(auto& line:lines_points){
            cout<< line.width* line.height<<endl;
        }
        auto viewer = show_multi_clouds_with_specified_colors(lines_points);
        while (!viewer->wasStopped())
        {
            viewer->spinOnce();
        }
    }



    return;
}



int main(){
    run1();

    // pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    // if (pcl::io::loadPCDFile<pcl::PointXYZ>("/home/jc/Documents/stereo-lidar-calibration/exclude_dir/debug_data/chessboard_plane_0002.pcd", *input_cloud) == -1) //* load the file
    // {
    //     PCL_ERROR("Couldn't read file test_pcd.pcd \n");
    //     return (-1);
    // }
    // LidarFeatureDetector lidar_feature_detector;
    // pcl::PointCloud<pcl::PointXYZ>::Ptr edge_pcd(new pcl::PointCloud<pcl::PointXYZ>);
    // lidar_feature_detector.extractEdgeCloud(input_cloud, edge_pcd);
    // std::vector<Eigen::VectorXf> lines_params;
    // std::vector<pcl::PointCloud<pcl::PointXYZ>> lines_points;
    // if (!lidar_feature_detector.extractFourLines(edge_pcd, lines_params, lines_points))
    // {
    //     return -1;
    // }
    // std::vector<pcl::PointXYZ> corners;
    // lidar_feature_detector.estimateFourCorners(lines_params,corners );
    // // for (int i = 0; i < 4; i++)
    // // {
    // //     auto edge_pcd_ptr = lines_points[i].makeShared();
    // //     display_colored_by_depth(edge_pcd_ptr);
    // // }
    // pcl::PointCloud<pcl::PointXYZ>::Ptr corners_pcd(new pcl::PointCloud<pcl::PointXYZ>);
    // for(auto& corner:corners){
    //     corners_pcd->push_back(corner);
    //     // std::cout<<corner<<std::endl;
    // }

    // pcl::visualization::PCLVisualizer viewer("test");
	// viewer.setBackgroundColor(0, 0, 0);

	// pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> singleColor1(corners_pcd, 0,255,0);//0-255  设置成绿色
    // pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> singleColor2(input_cloud, 255,255,0);
	// viewer.addPointCloud<pcl::PointXYZ>(corners_pcd, singleColor1, "sample1");//显示点云，其中fildColor为颜色显示
    // viewer.addPointCloud<pcl::PointXYZ>(input_cloud, singleColor2, "sample2");
	// viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, "sample1");//设置点云大小
    // viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "sample2");
	// while (!viewer.wasStopped())
	// {
	// 	viewer.spinOnce();
	// }


    return 0;
}