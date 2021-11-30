#include <pcl/common/common.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <iostream>


//#include "extractChessboard.hpp"
#include "utils.hpp"

using namespace stereo_lidar_calib;


int main(){
    pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud (new pcl::PointCloud<pcl::PointXYZ>);
    if (pcl::io::loadPCDFile<pcl::PointXYZ> 
        ("/home/jc/Documents/stereo-lidar-calibration/exclude_dir/test_data/test0.pcd", *input_cloud) == -1) //* load the file
    {
        PCL_ERROR ("Couldn't read file test_pcd.pcd \n");
        return (-1);
    }
    std::cout << "Loaded "
        << cloud->width * cloud->height
        << " data points from test_pcd.pcd with the following fields: "
        << std::endl;
    pcl::PointCloud<pcl::PointXYZ>::Ptr chessboard_point_cloud;
    chessboardExtractor extractor;
    extractor.extract(input_cloud, chessboard_point_cloud);
    display_colored_by_depth(chessboard_point_cloud);
    return 0;

}
