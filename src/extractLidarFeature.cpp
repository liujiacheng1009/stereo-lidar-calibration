#include "extractLidarFeature.hpp"

// void LidarFeatureDetector::getRings(std::vector<std::vector<pcl::PointXYZIr>>& rings)
// {
//     std::vector<std::vector<pcl::PointXYZIr> > rings(m_number_of_rings);
//     for(int i = 0; i < m_marker_board_pcd->points.size(); i++) {
//         m_marker_board_pcd->points[i].yaw = atan2(m_marker_board_pcd->points[i].y, m_marker_board_pcd->points[i].x);
//         rings[m_marker_board_pcd->points[i].ring].push_back(m_marker_board_pcd->points[i]);
//     }
//     return;
// }

// void LidarFeatureDetector::projPlaneFilter(pcl::PointCloud<pcl::PointXYZIR>::Ptr &cloud_seg,
//                         pcl::PointCloud<pcl::PointXYZIR>::Ptr &cloud_projected,
//                         pcl::ModelCoefficients::Ptr &coefficients)
// {
//     pcl::ProjectInliers<pcl::PointXYZIR> proj;
//     proj.setModelType(pcl::SACMODEL_PLANE);
//     proj.setInputCloud(cloud_seg);
//     proj.setModelCoefficients(coefficients);
//     proj.filter(*cloud_projected);
//     return;
// }

// void LidarFeatureDetector::getEdgePointCloud(std::vector<std::vector<pcl::PointXYZIr>>& rings,pcl::PointCloud<pcl::PointXYZIr>::Ptr& edge_pcd)
// {

//     for(int i = 0; i < rings.size(); i++) {
//         if(rings[i].size() > 0) {
//             std::vector<float> yaw_values;
//             for(int j = 0; j < rings[i].size(); j++) {
//                 pcl::PointXYZIr pt = rings[i][j];
//                 yaw_values.push_back(pt.yaw);
//             }
//             long maxElementIndex = std::max_element(yaw_values.begin(), yaw_values.end()) - yaw_values.begin();
//             float maxElement = *std::max_element(yaw_values.begin(), yaw_values.end());

//             long minElementIndex = std::min_element(yaw_values.begin(), yaw_values.end()) - yaw_values.begin();
//             float minElement = *std::min_element(yaw_values.begin(), yaw_values.end());

//             edge_pcd->points.push_back(rings[i][maxElementIndex]);
//             edge_pcd->points.push_back(rings[i][minElementIndex]);
//         }
//     }
//     return;
// }

bool LidarFeatureDetector::extractFourLines(pcl::PointCloud<pcl::PointXYZ>::Ptr &edge_pcd,
                                            std::vector<Eigen::VectorXf> &lines_params,
                                            std::vector<pcl::PointCloud<pcl::PointXYZ>> &lines_points)
{
    lines_params.clear();
    lines_points.clear();
    for (int i = 0; i < 4; i++)
    {
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_ptr(edge_pcd);
        pcl::SampleConsensusModelLine<pcl::PointXYZ>::Ptr model_l(new pcl::SampleConsensusModelLine<pcl::PointXYZ>(cloud_ptr));
        pcl::RandomSampleConsensus<pcl::PointXYZ> ransac_l(model_l);
        ransac_l.setDistanceThreshold(0.03);
        ransac_l.computeModel();
        std::vector<int> line_inliers;
        ransac_l.getInliers(line_inliers);
        if (!line_inliers.empty())
        {
            pcl::PointCloud<pcl::PointXYZ> line_i;
            pcl::copyPointCloud<pcl::PointXYZ>(*edge_pcd,
                                               line_inliers,
                                               line_i);
            lines_points.push_back(line_i);                       // 边界点云
            lines_params.push_back(ransac_l.model_coefficients_); // 边界线的参数
        }
        else
        {
            break;
        }
        pcl::PointCloud<pcl::PointXYZ> plane_no_line;
        remove_inliers(*cloud_ptr, line_inliers, plane_no_line);
        *edge_pcd = plane_no_line;
    }
    if (lines_points.size() == 4)
    {
        return true;
    }
    return false;
}

void LidarFeatureDetector::remove_inliers(const pcl::PointCloud<pcl::PointXYZ> &cloud_in,
                                          std::vector<int> inliers_indices,
                                          pcl::PointCloud<pcl::PointXYZ> &cloud_out)
{
    std::vector<int> outliers_indicies;
    for (size_t i = 0; i < cloud_in.size(); i++)
    {
        if (find(inliers_indices.begin(), inliers_indices.end(), i) == inliers_indices.end())
        {
            outliers_indicies.push_back(i);
        }
    }
    pcl::copyPointCloud<pcl::PointXYZ>(cloud_in, outliers_indicies, cloud_out);
}
// bool LidarFeatureDetector::extractFourEdges(pcl::PointCloud<pcl::PointXYZIr>::Ptr &edge_pcd,
//                                             std::vector<pcl::PointCloud<pcl::PointXYZI>> &line_points,
//                                             std::vector<Eigen::VectorXf> &line_params)
// {
//     for (int i = 0; i < 4; i++)
//     {
//         pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_ptr(edge_pcd);
//         pcl::SampleConsensusModelLine<pcl::PointXYZI>::Ptr model_l(new pcl::SampleConsensusModelLine<pcl::PointXYZI>(cloud_ptr));
//         pcl::RandomSampleConsensus<pcl::PointXYZI> ransac_l(model_l);
//         ransac_l.setDistanceThreshold(0.02);
//         ransac_l.computeModel();
//         std::vector<int> line_inliers;
//         ransac_l.getInliers(line_inliers);
//         if (!line_inliers.empty())
//         {
//             pcl::PointCloud<pcl::PointXYZI> line_i;
//             pcl::copyPointCloud<pcl::PointXYZI>(*edge_pcd, line_inliers, line_i);
//             line_points.push_back(line_i);                       // 边界点云
//             line_params.push_back(ransac_l.model_coefficients_); // 边界线的参数
//         }
//         else
//         {
//             break;
//         }
//         pcl::PointCloud<pcl::PointXYZI> plane_no_line;
//         remove_inliers(*cloud_ptr, line_inliers, plane_no_line);
//         *cloud_msg_pcl = plane_no_line;
//     }
//     if (line_points.size() == 4)
//     {
//         return true;
//     }
//     return false;
// }

// void LidarFeatureDetector::detectPlane(pcl::PointCloud<PointXYZIr>::Ptr& plane_pcd)
// {
//     pcl::PointCloud<PointXYZIr>::Ptr cloud_in_ptr(new pcl::PointCloud<PointXYZIr>);
//     *cloud_in_ptr = m_marker_board_pcd;
//     pcl::SampleConsensusModelPlane<pcl::PointXYZIr>::Ptr model_p(new pcl::SampleConsensusModelPlane<pcl::PointXYZIr>(cloud_in_ptr));
//     pcl::RandomSampleConsensus<PointXYZIr> ransac(model_p);
//     ransac.setDistanceThreshold(0.05);
//     bool model_computed = ransac.computeModel();
//     std::vector<int> inlier_indices;
//     if (model_computed) {
//         ransac.getInliers(inlier_indices);
//         pcl::copyPointCloud<PointXYZIr>(*cloud_in_ptr,inlier_indices,plane_pcd);
//     }
//     return;
// }

// void LidarFeatureDetector::getCorners(std::vector<Eigen::VectorXf> &line_params,
//                                      vector<pcl::PointXYZ> &chessboard_corners)
// {
//     Eigen::Vector4f p1, p2, p_intersect;
//     for(int i=0;i<4;++i){
//         pcl::lineToLineSegment(line_params[i],line_params[(i+1)%4], p1,p2);
//         for (int j = 0; j < 4; j++) {
//             p_intersect(j) = (p1(j) + p2(j)) / 2.0;
//         }
//         chessboard_corners->push_back(pcl::PointXYZ(p_intersect(0), p_intersect(1), p_intersect(2)));
//     }
//     return;
// }

void LidarFeatureDetector::extractEdgeCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr &input_cloud, pcl::PointCloud<pcl::PointXYZ>::Ptr &edge_pcd)
{
    edge_pcd->clear();
    pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> n; 
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
    n.setInputCloud(input_cloud);
    n.setSearchMethod(tree);
    n.setRadiusSearch(1);
    pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
    n.compute(*normals);

    pcl::BoundaryEstimation<pcl::PointXYZ, pcl::Normal, pcl::Boundary> boundEst; 
    boundEst.setInputCloud(input_cloud);
    boundEst.setInputNormals(normals);
    boundEst.setRadiusSearch(1);
    boundEst.setAngleThreshold(M_PI / 4); 

    boundEst.setSearchMethod(tree);
    pcl::PointCloud<pcl::Boundary> boundaries;
    boundEst.compute(boundaries);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_boundary(new pcl::PointCloud<pcl::PointXYZ>);
    for (int i = 0; i < input_cloud->points.size(); i++)
    {

        if (boundaries[i].boundary_point > 0)
        {
            edge_pcd->push_back(input_cloud->points[i]);
        }
    }
    return;
}