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
    PassFilterParams pass_filter_params(std::vector<double>({-10,10,-10,10,-10,10}));
    chessboardExtractor extractor(pass_filter_params);
    
    //display_colored_by_depth(input_cloud);
    // std::cout<< input_cloud->width* input_cloud->height << "points"<<std::endl;
    extractor.pass_filter(input_cloud);
    // std::cout<< input_cloud->width* input_cloud->height << "points"<<std::endl;
    std::vector<pcl::PointIndices> indices_clusters;
    extractor.pcd_clustering(input_cloud, indices_clusters);
    if(!extractor.fitPlane(input_cloud, indices_clusters)){
        return -1;
    }
    auto& plane_pcd = input_cloud;

    // for(auto& cluster:pcd_clusters){
    //     std::cout<< cluster.indices.size()<<std::endl;
    // }
    // pcl::ExtractIndices<pcl::PointXYZ> extract;

    // extract.setInputCloud(input_cloud);
    // extract.setIndices(boost::shared_ptr<pcl::PointIndices>(new pcl::PointIndices(pcd_clusters[0])));
    // extract.setNegative(false);
    // extract.filter(*input_cloud);
    
    // pcl::PointCloud<pcl::PointXYZIR>::Ptr chessboard_point_cloud;
    // chessboardExtractor extractor;
    // extractor.extract(input_cloud, chessboard_point_cloud);
    pcl::io::savePCDFileASCII ("/home/jc/Documents/stereo-lidar-calibration/exclude_dir/debug_data/chessboard_plane_0002.pcd", *plane_pcd);
    display_colored_by_depth(plane_pcd);
    return 0;

}
