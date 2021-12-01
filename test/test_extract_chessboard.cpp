#include <pcl/common/common.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <iostream>


#include "extractChessboard.hpp"
#include "utils.hpp"



int main(){
    pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud (new pcl::PointCloud<pcl::PointXYZ>);
    if (pcl::io::loadPCDFile<pcl::PointXYZ> 
        ("/home/jc/Documents/stereo-lidar-calibration/exclude_dir/lcc/HDL64/pointCloud/0002.pcd", *input_cloud) == -1) //* load the file
    {
        PCL_ERROR ("Couldn't read file test_pcd.pcd \n");
        return (-1);
    }
    PassFilterParams pass_filter_params(std::vector<double>({-5,5,-5,5,-5,5}));
    chessboardExtractor extractor(pass_filter_params);
    
    display_colored_by_depth(input_cloud);
    std::cout<< input_cloud->width* input_cloud->height << "points"<<std::endl;
    extractor.pass_filter(input_cloud);
    std::cout<< input_cloud->width* input_cloud->height << "points"<<std::endl;
    // pcl::PointCloud<pcl::PointXYZIR>::Ptr chessboard_point_cloud;
    // chessboardExtractor extractor;
    // extractor.extract(input_cloud, chessboard_point_cloud);
    display_colored_by_depth(input_cloud);
    return 0;

}
