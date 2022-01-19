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
using namespace cv;
using namespace std;
using namespace pcl;

class CloudResults;

bool loadPointXYZ(string& path, PointCloud<PointXYZ>::Ptr& pcd);
void getValidDataSet(ImageResults& images_features, CloudResults& cloud_features);
void getValidDataSet(ImageResults& images_features1,ImageResults& images_features2, CloudResults& cloud_features);


class CloudResults{
public:
    vector<int> valid_index; // 有效的点云index
    vector<vector<Vector3d>> corners_3d; // 点云3d角点
    vector<PointCloud<PointXYZ>> plane_points_3d; 
    //vector<vector<Vector3d>> plane_points_3d;
    vector<vector<VectorXd>> lines_3d; // 点云边缘直线方程
    vector<vector<PointCloud<PointXYZ>>> lines_points_3d ;//直线包含的点云
};




class LidarFeatureDetector{
public:
    template<typename T>
    LidarFeatureDetector(T &config) : m_chessboard_extractor(ChessboardExtractor(config)),
                                    m_checkerboard_grid_size(config.checkerboard_grid_size),
                                     m_checkerboard_square_size(config.checkerboard_square_size),
                                     m_checkerboard_padding(config.checkerboard_padding) {}
    // void getCorners(std::vector<Eigen::VectorXf> &line_params, std::vector<pcl::PointXYZ>& chessboard_corners); // 从直线方程中获取四个角点

    // void getEdgePointCloud(std::vector<std::vector<pcl::PointXYZ>>& rings,pcl::PointCloud<pcl::PointXYZ>::Ptr& edge_pcd);// 从rings中提取边缘点云

    // bool extractFourEdges(pcl::PointCloud<pcl::PointXYZ>::Ptr &edge_pcd,
    //                       std::vector<pcl::PointCloud<pcl::PointXYZ>> &line_points,
    //                       std::vector<Eigen::VectorXf> &line_params); // 从边缘点云中拟合四个直线

    
    // void detectPlane(pcl::PointCloud<PointXYZ>::Ptr& plane_pcd);
    bool extractPlaneCloud(PointCloud<PointXYZ>::Ptr &input_cloud, PointCloud<PointXYZ>::Ptr &plane_pcd);
    bool extractPlaneCloud1(PointCloud<PointXYZ>::Ptr &input_cloud,
                            PointCloud<PointXYZ>::Ptr &plane_pcd,
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
                            vector<PointCloud<PointXYZ>> &reordered_lines_points,
                            vector<pair<pair<PointXYZ, PointXYZ>, PointCloud<PointXYZ>>> &line_corners_to_points);
    // 利用边界线点云优化角点
    void refineCorners(vector<Vector3d>& corners_3d, vector<PointCloud<PointXYZ>>& lines_points_3d);

    void estimateCornersFromBoardParams(Eigen::Affine3f& board_pose, Eigen::Vector3f& board_size, vector<PointXYZ>& corners);

private:
    // void getRings(std::vector<std::vector<pcl::PointXYZ>>& rings); // 将marker board点云按照线束id存储
    void remove_inliers(const pcl::PointCloud<pcl::PointXYZ> &cloud_in,
                        std::vector<int> inliers_indices,
                        pcl::PointCloud<pcl::PointXYZ> &cloud_out);

    void reorder_lines(std::vector<Eigen::VectorXf> &lines_params,
                       std::vector<pcl::PointCloud<pcl::PointXYZ>> &lines_points);

    void reorder_lines1(std::vector<Eigen::VectorXf> &lines_params,
                        std::vector<pcl::PointCloud<pcl::PointXYZ>> &lines_points);

    bool isValidCorners(vector<PointXYZ>& corners);



    pcl::PointCloud<pcl::PointXYZ>::Ptr m_marker_board_pcd;
    int m_number_of_rings;

private:
    ChessboardExtractor m_chessboard_extractor;
    const double m_checkerboard_square_size;
    const Size m_checkerboard_grid_size;    
    const vector<double> m_checkerboard_padding; 
};




template<typename T>
void processCloud(T& config, vector<string>& cloud_paths, CloudResults& cloud_features)
{
    LidarFeatureDetector lidar_feature_detector(config);
    
    for(int i=0;i<cloud_paths.size();++i)
    {
        pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud(new pcl::PointCloud<pcl::PointXYZ>);
        if (!loadPointXYZ(cloud_paths[i], input_cloud)) continue;
        // cout<< input_cloud->points.size()<<endl;
        // display_colored_by_depth(input_cloud);
        PointCloud<PointXYZ>::Ptr plane_cloud(new pcl::PointCloud<pcl::PointXYZ>);
        Eigen::Affine3f board_pose;
        Eigen::Vector3f board_size;
        //if(!lidar_feature_detector.extractPlaneCloud(input_cloud,plane_cloud)) continue;
        if(!lidar_feature_detector.extractPlaneCloud1(input_cloud, plane_cloud, board_pose, board_size)) continue;
        pcl::PointCloud<pcl::PointXYZ>::Ptr edge_cloud(new pcl::PointCloud<pcl::PointXYZ>);
        lidar_feature_detector.extractEdgeCloud(plane_cloud, edge_cloud);// 提取边缘点云
        std::vector<Eigen::VectorXf> lines_params;
        std::vector<pcl::PointCloud<pcl::PointXYZ>> lines_points;
        if (!lidar_feature_detector.extractFourLines(edge_cloud, lines_params, lines_points)) continue;// 提取四条边界线
        std::vector<pcl::PointXYZ> corners, reordered_corners;
        if (lidar_feature_detector.estimateFourCorners(lines_params, corners))
        {
            
            vector<pair<std::pair<pcl::PointXYZ, pcl::PointXYZ>, pcl::PointCloud<pcl::PointXYZ>>> line_corners_to_points;
            for (int i = 0; i < 4; i++)
            {
                line_corners_to_points.push_back(make_pair(make_pair(corners[i], corners[(i + 1) % 4]), lines_points[(i + 1) % 4]));
            }                                                                   // 建立角点和边缘直线上点云的对应关系
            lidar_feature_detector.reorder_corners(corners, reordered_corners); // 角点重排序，与camera corner 保持对应关系
            std::vector<pcl::PointCloud<pcl::PointXYZ>> reordered_lines_points;
            lidar_feature_detector.reorder_line_points(reordered_corners, reordered_lines_points, line_corners_to_points); // 重排序边缘直线上的点
            // debug
            // display_multi_clouds(reordered_lines_points);
            vector<Vector3d> corners_temp;
            for (auto &corner : reordered_corners)
            {
                corners_temp.push_back(Vector3d(corner.x, corner.y, corner.z));
            }
            lidar_feature_detector.refineCorners(corners_temp, reordered_lines_points);
            cloud_features.lines_points_3d.push_back(reordered_lines_points);
            cloud_features.corners_3d.push_back(corners_temp);
        }
        else
        {
            lidar_feature_detector.estimateCornersFromBoardParams(board_pose, board_size, corners);
            lidar_feature_detector.reorder_corners(corners, reordered_corners); 
            vector<Vector3d> corners_temp;
            for (auto &corner : reordered_corners)
            {
                corners_temp.push_back(Vector3d(corner.x, corner.y, corner.z));
            }
            cloud_features.corners_3d.push_back(corners_temp);
        }
        // debug
        // display_four_corners(corners);
        cloud_features.plane_points_3d.push_back(*plane_cloud);
        cloud_features.valid_index.push_back(i);
    }
    return;
}



#endif