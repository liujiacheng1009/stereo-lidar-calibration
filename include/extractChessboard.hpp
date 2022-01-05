#ifndef STEREO_LIDAR_CALIBRATION_EXTRACTCHESSBOARD_HPP
#define STEREO_LIDAR_CALIBRATION_EXTRACTCHESSBOARD_HPP

#include <pcl/common/common.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/eigen.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/passthrough.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_plane.h>
#include <pcl/features/normal_3d.h>
#include <pcl/search/kdtree.h>
#include <pcl/search/search.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/segmentation/region_growing.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/project_inliers.h>
#include <pcl/filters/crop_box.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/io/pcd_io.h>
#include <string>
#include "config.hpp"
#include "utils.hpp"

using namespace cv;
using namespace Eigen;
using namespace std;
using namespace pcl;

struct PassFilterParams{
    double min_x,max_x;
    double min_y,max_y;
    double min_z, max_z;
    PassFilterParams(std::vector<double> params){
        assert(params.size()==6);
        min_x = params[0];
        max_x = params[1];
        min_y = params[2];
        max_y = params[3];
        min_z = params[4];
        max_z = params[5];
    }
};
template<typename T>
class ChessboardExtractor{
public:
    ChessboardExtractor(T &config) : m_pass_filter_params(PassFilterParams(config.pass_filter_params)) {}

public:
    // 主函数， 从环境点云中提取marker board 点云
    // bool extract(std::string& lidarPath, pcl::PointCloud<pcl::PointXYZ>::Ptr& plane_cloud);
    // 点云聚类， 获取可能的点云簇
    void pcd_clustering(pcl::PointCloud<pcl::PointXYZ>::Ptr& input_pcd,
        std::vector<pcl::PointIndices>& pcd_clusters);
    // 对上面的点云簇进行平面拟合 
    bool fitPlane(pcl::PointCloud<pcl::PointXYZ>::Ptr& input_pcd,
        std::vector<pcl::PointIndices>& indices_clusters, pcl::PointCloud<pcl::PointXYZ>::Ptr& plane_pcd);

    bool extractChessboard(pcl::PointCloud<pcl::PointXYZ>::Ptr& input_pcd,
        std::vector<pcl::PointIndices>& indices_clusters, pcl::PointCloud<pcl::PointXYZ>::Ptr& chessboard_pcd);

    // 将原始点云投影到估计的平面上
    void projPlaneFilter(pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud,
                         pcl::PointCloud<pcl::PointXYZ>::Ptr &proj_cloud,
                         pcl::ModelCoefficients::Ptr &coeff);

    void pass_filter(pcl::PointCloud<pcl::PointXYZ>::Ptr& input_pcd);

private:
    // 预处理， 滤出固定范围外的点云
    void getPointCloudInROI();
    void extract_pcd_by_indices(pcl::PointIndices &indices, pcl::PointCloud<pcl::PointXYZ>::Ptr &input_pcd,
                                pcl::PointCloud<pcl::PointXYZ>::Ptr &output_pcd);
    PassFilterParams m_pass_filter_params;
};

template<typename T>
void ChessboardExtractor<T>::pass_filter(pcl::PointCloud<pcl::PointXYZ>::Ptr& input_pcd)
{
    auto &pcd_in_roi = input_pcd;
    PassFilterParams& params = m_pass_filter_params;
    pcl::PassThrough<pcl::PointXYZ> filter;
    filter.setInputCloud(pcd_in_roi);
    filter.setFilterFieldName("z");
    filter.setFilterLimits(params.min_z, params.max_z);
    filter.filter(*pcd_in_roi);

    filter.setInputCloud(pcd_in_roi);
    filter.setFilterFieldName("y");
    filter.setFilterLimits(params.min_y, params.max_y);
    filter.filter(*pcd_in_roi);

    filter.setInputCloud(pcd_in_roi);
    filter.setFilterFieldName("x");
    filter.setFilterLimits(params.min_x, params.max_x);
    filter.filter(*pcd_in_roi);
    return;
}

template<typename T>
void ChessboardExtractor<T>::pcd_clustering(pcl::PointCloud<pcl::PointXYZ>::Ptr& input_pcd,
                                              std::vector<pcl::PointIndices>& pcd_clusters)
{
    pcl::search::Search<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
    pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
    pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> normal_estimator;
    normal_estimator.setSearchMethod(tree);
    normal_estimator.setInputCloud(input_pcd);
    normal_estimator.setKSearch(50);
    normal_estimator.compute(*normals);

    pcl::RegionGrowing<pcl::PointXYZ, pcl::Normal> reg;
    reg.setMinClusterSize(400);
    reg.setMaxClusterSize(4000);
    reg.setSearchMethod(tree);
    reg.setNumberOfNeighbours(60);
    reg.setInputCloud(input_pcd);
    reg.setInputNormals(normals);
    reg.setSmoothnessThreshold(3.0 / 180.0 * M_PI);
    reg.setCurvatureThreshold(0.3);

    reg.extract(pcd_clusters);
    // dubug
    // pcl::ExtractIndices<pcl::PointXYZ> extract;
    // extract.setInputCloud(input_pcd);
    // for(auto& cluster:pcd_clusters){
    //     pcl::PointCloud<pcl::PointXYZ>::Ptr pcd_cluster(new pcl::PointCloud<pcl::PointXYZ>);
    //     extract.setIndices(boost::shared_ptr<pcl::PointIndices>(new pcl::PointIndices(cluster)));
    //     extract.setNegative(false);
    //     extract.filter(*pcd_cluster);
    //     display_colored_by_depth(pcd_cluster);
    //     //cout<< cluster.size()<<endl;
    // }
    // exit(17);
    return;
}

template<typename T>
bool ChessboardExtractor<T>::fitPlane(pcl::PointCloud<pcl::PointXYZ>::Ptr &input_pcd,
                                   std::vector<pcl::PointIndices> &indices_clusters,
                                       PointCloud<PointXYZ>::Ptr &plane_pcd)
{
    pcl::ExtractIndices<pcl::PointXYZ> extract;
    extract.setInputCloud(input_pcd);
    for (auto &cluster : indices_clusters)
    {
        pcl::PointCloud<pcl::PointXYZ>::Ptr pcd_cluster(new pcl::PointCloud<pcl::PointXYZ>);
        extract.setIndices(boost::shared_ptr<pcl::PointIndices>(new pcl::PointIndices(cluster)));
        extract.setNegative(false);
        extract.filter(*pcd_cluster);
        pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
        pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
        pcl::SACSegmentation<pcl::PointXYZ> seg;
        seg.setOptimizeCoefficients(true);
        seg.setModelType(pcl::SACMODEL_PLANE);
        seg.setMethodType(pcl::SAC_RANSAC);
        seg.setDistanceThreshold(0.04);
        seg.setInputCloud(pcd_cluster);
        seg.segment(*inliers, *coefficients);
        double inliers_percentage = double(inliers->indices.size()) / pcd_cluster->points.size();
        if (inliers_percentage > 0.9)
        {
            plane_pcd = pcd_cluster;
            return true;
        }
    }
    
    return false;
}

template<typename T>
bool ChessboardExtractor<T>::extractChessboard(pcl::PointCloud<pcl::PointXYZ>::Ptr &input_pcd,
                                   std::vector<pcl::PointIndices> &indices_clusters,
                                       PointCloud<PointXYZ>::Ptr &chessboard_pcd)
{
    pcl::ExtractIndices<pcl::PointXYZ> extract;
    extract.setInputCloud(input_pcd);
    for (auto &cluster : indices_clusters)
    {
        pcl::PointCloud<pcl::PointXYZ>::Ptr pcd_cluster(new pcl::PointCloud<pcl::PointXYZ>);
        extract.setIndices(boost::shared_ptr<pcl::PointIndices>(new pcl::PointIndices(cluster)));
        extract.setNegative(false);
        extract.filter(*pcd_cluster);

        pcl::SampleConsensusModelPlane<pcl::PointXYZ>::Ptr plane_model(
            new pcl::SampleConsensusModelPlane<pcl::PointXYZ>(pcd_cluster));
        pcl::RandomSampleConsensus<pcl::PointXYZ> ransac(plane_model);
        ransac.setDistanceThreshold(0.04);
        ransac.setMaxIterations(100);
        if (!ransac.computeModel()) continue;
        vector<int> inliers;
        ransac.getInliers(inliers);
        double inliers_percentage = double(inliers.size()) / pcd_cluster->points.size();
        if (inliers_percentage > 0.9)
        {
            // debug
            display_colored_by_depth(pcd_cluster);
            exit(17);
            pcl::copyPointCloud(*input_pcd, inliers, *chessboard_pcd);
            return true;
        }
    }
    return false;
}



#endif