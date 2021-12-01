#include "extractChessboard.hpp"

using namespace std;


// void chessboardExtractor::extract(pcl::PointCloud<pcl::PointXYZ>::Ptr input_lidar_pcd,
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

// void chessboardExtractor::extract_pcd_by_indices(pcl::PointIndices &indices,
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

// void chessboardExtractor::getPointCloudInROI()
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

void chessboardExtractor::pass_filter(pcl::PointCloud<pcl::PointXYZ>::Ptr& input_pcd)
{
    auto &pcd_in_roi = input_pcd;
    pcl::PassThrough<pcl::PointXYZ> filter;
    filter.setInputCloud(pcd_in_roi);
    filter.setFilterFieldName("z");
    filter.setFilterLimits(m_pass_filter_params.min_z, m_pass_filter_params.max_z);
    filter.filter(*pcd_in_roi);

    filter.setInputCloud(pcd_in_roi);
    filter.setFilterFieldName("y");
    filter.setFilterLimits(m_pass_filter_params.min_y, m_pass_filter_params.max_y);
    filter.filter(*pcd_in_roi);

    filter.setInputCloud(pcd_in_roi);
    filter.setFilterFieldName("x");
    filter.setFilterLimits(m_pass_filter_params.min_x, m_pass_filter_params.max_x);
    filter.filter(*pcd_in_roi);
    return;
}

// void chessboardExtractor::get_pcds_by_cluster(pcl::PointCloud<pcl::PointXYZ>::Ptr input_pcd,
//                                               std::vector<pcl::PointIndices> pcd_clusters)
// {
//     pcl::search::Search<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
//     pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
//     pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> normal_estimator;
//     normal_estimator.setSearchMethod(tree);
//     normal_estimator.setInputCloud(cloud);
//     normal_estimator.setKSearch(50);
//     normal_estimator.compute(*normals);

//     // Region growing to create potentatial planes
//     pcl::RegionGrowing<pcl::PointXYZ, pcl::Normal> reg;
//     reg.setMinClusterSize(150);
//     reg.setMaxClusterSize(6000);
//     reg.setSearchMethod(tree);
//     reg.setNumberOfNeighbours(60);
//     reg.setInputCloud(cloud);
//     reg.setInputNormals(normals);
//     reg.setSmoothnessThreshold(3.0 / 180.0 * M_PI);
//     reg.setCurvatureThreshold(0.3);

//     std::vector<pcl::PointIndices> clusters;
//     reg.extract(clusters);
//     return;
// }

// bool chessboardExtractor::fitPlane(pcl::PointCloud<pcl::PointXYZ>::Ptr input_pcd,
//     pcl::PointCloud<pcl::PointXYZ>::Ptr plane_pcd)
// {

//     pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
//     pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
//     pcl::SACSegmentation<pcl::PointXYZ> seg;
//     seg.setOptimizeCoefficients(true);
//     seg.setModelType(pcl::SACMODEL_PLANE);
//     seg.setMethodType(pcl::SAC_RANSAC);
//     seg.setDistanceThreshold(0.04);
//     seg.setInputCloud(input_pcd);
//     seg.segment(*inliers, *coefficients);
//     double inliers_percentage = double(inliers->indices.size()) / input_pcd->size();
//     if(inliers_percentage>0.9){
//         extract_pcd_by_indices(inliers, input_pcd, plane_pcd);
//         return true;
//     } 
//     return false;
// }

