#ifndef STEREO_LIDAR_CALIBRATION_EXTRACTLIDARFEATURE_HPP
#define STEREO_LIDAR_CALIBRATION_EXTRACTLIDARFEATURE_HPP
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
#include <pcl/filters/filter.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/project_inliers.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/common/intersections.h>
#include <Eigen/Core>
#include <pcl/common/geometry.h>
#include <algorithm>
#include <unordered_map>
#include <map>
#include "extractChessboard.hpp"
#include "optimization.hpp"
#include "utils.hpp"


using namespace Eigen;
using namespace std;



bool loadPointXYZ(string& path, pcl::PointCloud<pcl::PointXYZ>::Ptr& pcd);
void getValidDataSet(ImageResults& images_features, CloudResults& cloud_features);
void getValidDataSet(ImageResults& images_features1,ImageResults& images_features2, CloudResults& cloud_features);




void processCloud(vector<string>& cloud_paths, CloudResults& cloud_features);


class LidarFeatureDetector{
public:
    LidarFeatureDetector() : m_chessboard_extractor(ChessboardExtractor()),
                                    m_checkerboard_grid_size(Config::checkerboardGridSize()),
                                     m_checkerboard_square_size(Config::checkerboardSquareSize()),
                                     m_checkerboard_padding(Config::checkerboardPadding()) {}
    // void getCorners(std::vector<Eigen::VectorXf> &line_params, std::vector<pcl::PointXYZ>& chessboard_corners); // 从直线方程中获取四个角点

    // void getEdgePointCloud(std::vector<std::vector<pcl::PointXYZ>>& rings,pcl::PointCloud<pcl::PointXYZ>::Ptr& edge_pcd);// 从rings中提取边缘点云

    // bool extractFourEdges(pcl::PointCloud<pcl::PointXYZ>::Ptr &edge_pcd,
    //                       std::vector<pcl::PointCloud<pcl::PointXYZ>> &line_points,
    //                       std::vector<Eigen::VectorXf> &line_params); // 从边缘点云中拟合四个直线

    
    // void detectPlane(pcl::PointCloud<PointXYZ>::Ptr& plane_pcd);
    bool extractPlaneCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr &input_cloud, pcl::PointCloud<pcl::PointXYZ>::Ptr &plane_pcd);
    bool extractPlaneCloud1(pcl::PointCloud<pcl::PointXYZ>::Ptr &input_cloud,
                            pcl::PointCloud<pcl::PointXYZ>::Ptr &plane_pcd,
                            Eigen::Affine3f &board_pose,
                            Eigen::Vector3f &board_size);
    void extractEdgeCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr &input_cloud, pcl::PointCloud<pcl::PointXYZ>::Ptr &edge_pcd);
    bool extractFourLines(pcl::PointCloud<pcl::PointXYZ>::Ptr &edge_pcd,
                          std::vector<Eigen::VectorXf> &lines_params,
                          std::vector<pcl::PointCloud<pcl::PointXYZ>> &lines_points);
    bool estimateFourCorners(std::vector<Eigen::VectorXf> &line_params, std::vector<pcl::PointXYZ>& corners); // 从直线方程中获取四个角点

    // 调整lidar points 的顺序
    void reorder_corners(std::vector<pcl::PointXYZ>& ori_corners, std::vector<pcl::PointXYZ>& reordered_corners);

    // 调整边缘直线上的点云顺序
    void reorder_line_points(std::vector<pcl::PointXYZ>  &reordered_corners,
                            vector<pcl::PointCloud<pcl::PointXYZ>> &reordered_lines_points,
                            vector<pair<pair<pcl::PointXYZ, pcl::PointXYZ>, pcl::PointCloud<pcl::PointXYZ>>> &line_corners_to_points);
    // 利用边界线点云优化角点
    void refineCorners(vector<Vector3d>& corners_3d, vector<pcl::PointCloud<pcl::PointXYZ>>& lines_points_3d);

    void estimateCornersFromBoardParams(Eigen::Affine3f& board_pose, Eigen::Vector3f& board_size, vector<pcl::PointXYZ>& corners);

private:
    // void getRings(std::vector<std::vector<pcl::PointXYZ>>& rings); // 将marker board点云按照线束id存储
    void remove_inliers(const pcl::PointCloud<pcl::PointXYZ> &cloud_in,
                        std::vector<int> inliers_indices,
                        pcl::PointCloud<pcl::PointXYZ> &cloud_out);

    void reorder_lines(std::vector<Eigen::VectorXf> &lines_params,
                       std::vector<pcl::PointCloud<pcl::PointXYZ>> &lines_points);

    void reorder_lines1(std::vector<Eigen::VectorXf> &lines_params,
                        std::vector<pcl::PointCloud<pcl::PointXYZ>> &lines_points);

    bool isValidCorners(vector<pcl::PointXYZ>& corners);



    pcl::PointCloud<pcl::PointXYZ>::Ptr m_marker_board_pcd;
    int m_number_of_rings;

private:
    ChessboardExtractor m_chessboard_extractor;
    const double m_checkerboard_square_size;
    const cv::Size m_checkerboard_grid_size;    
    const vector<double> m_checkerboard_padding; 
};






#endif