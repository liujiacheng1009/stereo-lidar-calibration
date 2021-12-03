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
    if (!lidar_feature_detector.extractFourLines(edge_pcd, lines_params, lines_points))
    {
        return -1;
    }
    std::vector<pcl::PointXYZ> corners;
    lidar_feature_detector.estimateFourCorners(lines_params,corners );
    // for (int i = 0; i < 4; i++)
    // {
    //     auto edge_pcd_ptr = lines_points[i].makeShared();
    //     display_colored_by_depth(edge_pcd_ptr);
    // }
    pcl::PointCloud<pcl::PointXYZ>::Ptr corners_pcd(new pcl::PointCloud<pcl::PointXYZ>);
    for(auto& corner:corners){
        corners_pcd->push_back(corner);
        // std::cout<<corner<<std::endl;
    }

    pcl::visualization::PCLVisualizer viewer("test");
	viewer.setBackgroundColor(0, 0, 0);

	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> singleColor1(corners_pcd, 0,255,0);//0-255  设置成绿色
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> singleColor2(input_cloud, 255,255,0);
	viewer.addPointCloud<pcl::PointXYZ>(corners_pcd, singleColor1, "sample1");//显示点云，其中fildColor为颜色显示
    viewer.addPointCloud<pcl::PointXYZ>(input_cloud, singleColor2, "sample2");
	viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, "sample1");//设置点云大小
    viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "sample2");
	while (!viewer.wasStopped())
	{
		viewer.spinOnce();
	}


    return 0;
}