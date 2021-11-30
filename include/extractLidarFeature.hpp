#include <iostream>
#include <memory>   /// std::shared_ptr
#include <pcl/common/common.h>
#include <vector>
#include <Eigen/Core>


class LidarFeatureDetector{
public:
    void getCorners(std::vector<Eigen::VectorXf> &line_params, vector<pcl::PointXYZ>& chessboard_corners); // 从直线方程中获取四个角点

    void getEdgePointCloud(std::vector<std::vector<PointXYZIr>>& rings,pcl::PointCloud<pcl::PointXYZIr>::Ptr& edge_pcd);// 从rings中提取边缘点云

    bool extractFourEdges(pcl::PointCloud<pcl::PointXYZIr>::Ptr &edge_pcd,
                          std::vector<pcl::PointCloud<pcl::PointXYZI>> &line_points,
                          std::vector<Eigen::VectorXf> &line_params); // 从边缘点云中拟合四个直线

    
    void detectPlane(pcl::PointCloud<PointXYZIr>::Ptr& plane_pcd);

private:
    void getRings(std::vector<std::vector<PointXYZIr>>& rings); // 将marker board点云按照线束id存储

    pcl::PointCloud<pcl::PointXYZIr>::Ptr m_marker_board_pcd;
    int m_number_of_rings;

};
