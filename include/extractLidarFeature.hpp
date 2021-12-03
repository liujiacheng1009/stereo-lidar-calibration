#include <iostream>
#include <memory>   /// std::shared_ptr
#include <pcl/common/common.h>
#include <vector>
#include <Eigen/Core>
#include <vector>
#include <ctime>
#include <boost/thread/thread.hpp>
#include <pcl/io/ply_io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/console/parse.h>
#include <pcl/features/eigen.h>
#include <pcl/features/feature.h>
#include <pcl/features/normal_3d.h>
#include <pcl/impl/point_types.hpp>
#include <pcl/features/boundary.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_line.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <Eigen/Core>


class LidarFeatureDetector{
public:
    LidarFeatureDetector(){}
    // void getCorners(std::vector<Eigen::VectorXf> &line_params, std::vector<pcl::PointXYZ>& chessboard_corners); // 从直线方程中获取四个角点

    // void getEdgePointCloud(std::vector<std::vector<pcl::PointXYZ>>& rings,pcl::PointCloud<pcl::PointXYZ>::Ptr& edge_pcd);// 从rings中提取边缘点云

    // bool extractFourEdges(pcl::PointCloud<pcl::PointXYZ>::Ptr &edge_pcd,
    //                       std::vector<pcl::PointCloud<pcl::PointXYZ>> &line_points,
    //                       std::vector<Eigen::VectorXf> &line_params); // 从边缘点云中拟合四个直线

    
    // void detectPlane(pcl::PointCloud<PointXYZ>::Ptr& plane_pcd);

    void extractEdgeCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr &input_cloud, pcl::PointCloud<pcl::PointXYZ>::Ptr &edge_pcd);
    bool extractFourLines(pcl::PointCloud<pcl::PointXYZ>::Ptr &edge_pcd,
                          std::vector<Eigen::VectorXf> &lines_params,
                          std::vector<pcl::PointCloud<pcl::PointXYZ>> &lines_pcd);

private:
    // void getRings(std::vector<std::vector<pcl::PointXYZ>>& rings); // 将marker board点云按照线束id存储
    void remove_inliers(const pcl::PointCloud<pcl::PointXYZ> &cloud_in,
                        std::vector<int> inliers_indices,
                        pcl::PointCloud<pcl::PointXYZ> &cloud_out); 
    pcl::PointCloud<pcl::PointXYZ>::Ptr m_marker_board_pcd;
    int m_number_of_rings;

};
