#include <pcl/common/common.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <iostream>

#include "utils.hpp"
#include "extractLidarFeature.hpp"


int main(){
    pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    if (pcl::io::loadPCDFile<pcl::PointXYZ>("/home/jc/Documents/stereo-lidar-calibration/exclude_dir/debug_data/chessboard_plane_0002.pcd", *input_cloud) == -1) //* load the file
    {
        PCL_ERROR("Couldn't read file test_pcd.pcd \n");
        return (-1);
    }
    LidarFeatureDetector lidar_feature_detector;
    pcl::PointCloud<pcl::PointXYZ>::Ptr edge_pcd(new pcl::PointCloud<pcl::PointXYZ>);
    lidar_feature_detector.extractEdgeCloud(input_cloud, edge_pcd);
    std::vector<Eigen::VectorXf> lines_params;
    std::vector<pcl::PointCloud<pcl::PointXYZ>> lines_points;
    if (lidar_feature_detector.extractFourLines(edge_pcd, lines_params, lines_points))
    {
        for (int i = 0; i < 4; i++)
        {
            auto edge_pcd_ptr = lines_points[i].makeShared();
            display_colored_by_depth(edge_pcd_ptr);
        }
    }
    return 0;
}