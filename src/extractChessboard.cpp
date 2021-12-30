#include "extractChessboard.hpp"

using namespace std;
using namespace cv;
using namespace Eigen;
using namespace pcl;
// bool ChessboardExtractor::extract(std::string& lidarPath, pcl::PointCloud<pcl::PointXYZ>::Ptr& plane_cloud)
// {
//     pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud (new pcl::PointCloud<pcl::PointXYZ>);
//     if (pcl::io::loadPCDFile<pcl::PointXYZ> (lidarPath, *input_cloud) == -1) 
//     {
//         PCL_ERROR ("Couldn't read file \n");
//         return false;
//     }
//     // PassFilterParams pass_filter_params(std::vector<double>({-10,10,-10,10,-10,10}));
//     this->pass_filter(input_cloud);
//     std::vector<pcl::PointIndices> indices_clusters;
//     this->pcd_clustering(input_cloud, indices_clusters);
//     if(!this->fitPlane(input_cloud, indices_clusters)){
//         return false;
//     }
//     plane_cloud = input_cloud;
//     return true;
// }


// void ChessboardExtractor::extract(pcl::PointCloud<pcl::PointXYZ>::Ptr input_lidar_pcd,
//                                   pcl::PointCloud<pcl::PointXYZ>::Ptr chessboard_pcd)
// {
//     getPointCloudInROI();
//     std::vector<pcl::PointIndices> pcd_clusters;
//     get_pcds_by_cluster(input_lidar_pcd, pcd_clusters);
//     for (int i = 0; i < pcd_clusters.size(); ++i)
//     {
//         pcl::PointIndices &indices = pcd_clusters[i];
//         pcl::PointCloud<pcl::PointXYZ>::Ptr pcd_cluster(new pcl::PointCloud<pcl::PointXYZ>);
//         extract_pcd_by_indices(indices, input_lidar_pcd, pcd_cluster);
//         pcd::PointCloud<pcl::PointXYZ>::Ptr plane_pcd(new pcd::PointCloud<pcl::PointXYZ>);
//         if (fitPlane(pcd_cluster, plane_pcd))
//         {
//             chessboard_pcd = plane_pcd;
//             break;
//         }
//     }
//     return;
// }

// void ChessboardExtractor::extract_pcd_by_indices(pcl::PointIndices &indices,
//                                                  pcl::PointCloud<pcl::PointXYZ>::Ptr &input_pcd,
//                                                      pcl::PointCloud<pcl::PointXYZ>::Ptr &output_pcd)
// {
//     pcl::ExtractIndices<pcl::PointXYZ> extract;
//     extract.setInputCloud(input_pcd);
//     extract.setIndices(boost::shared_ptr<pcl::PointIndices>(new pcl::PointIndices(indices)));
//     extract.setNegative(false);
//     extract.filter(*output_pcd);
//     return;
// }

// void ChessboardExtractor::getPointCloudInROI()
// {
//     pcl::PassThrough<pcl::PointXYZ> filter;
//     filter.setInputCloud(pcd_in_roi_);
//     filter.setFilterFieldName("z");
//     filter.setFilterLimits(min_z_, max_z_);
//     filter.filter(*pcd_in_roi_);
//     filter.setInputCloud(pcd_in_roi_);
//     filter.setFilterFieldName("y");
//     filter.setFilterLimits(min_y_, max_y_);
//     filter.filter(*pcd_in_roi_);
//     filter.setInputCloud(pcd_in_roi_);
//     filter.setFilterFieldName("x");
//     filter.setFilterLimits(min_x_, max_x_);
//     filter.filter(*pcd_in_roi_);
//     return;
// }

void ChessboardExtractor::pass_filter(pcl::PointCloud<pcl::PointXYZ>::Ptr& input_pcd)
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

void ChessboardExtractor::pcd_clustering(pcl::PointCloud<pcl::PointXYZ>::Ptr& input_pcd,
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
    reg.setMaxClusterSize(3500);
    reg.setSearchMethod(tree);
    reg.setNumberOfNeighbours(60);
    reg.setInputCloud(input_pcd);
    reg.setInputNormals(normals);
    reg.setSmoothnessThreshold(3.0 / 180.0 * M_PI);
    reg.setCurvatureThreshold(0.3);

    reg.extract(pcd_clusters);
    return;
}

bool ChessboardExtractor::fitPlane(pcl::PointCloud<pcl::PointXYZ>::Ptr &input_pcd,
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
