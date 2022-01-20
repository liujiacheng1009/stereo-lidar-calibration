#include "extractChessboard.hpp"

using namespace std;
using namespace Eigen;


bool ChessboardExtractor::check_board_size(pcl::PointCloud<pcl::PointXYZ>::Ptr board_pcd)
{
    double max_padding = *max_element(m_checkerboard_padding.begin(), m_checkerboard_padding.end());
    max_padding *= 2.0;
    double max_len = sqrt(((m_checkerboard_grid_size.width+1)*m_checkerboard_square_size+max_padding)*
            ((m_checkerboard_grid_size.width+1)*m_checkerboard_square_size+max_padding) + 
            ((m_checkerboard_grid_size.width+1)*m_checkerboard_square_size+max_padding)*
            ((m_checkerboard_grid_size.width+1)*m_checkerboard_square_size+max_padding)); // 对角线长
    max_len *= 1.2; // 裕度
    pcl::PointXYZ minPt, maxPt;
    pcl::getMinMax3D (*board_pcd, minPt, maxPt);

    // cout<< max_len<<endl;
    // cout<< maxPt.x-minPt.x<<endl;
    // cout<< maxPt.y-minPt.y<<endl;
    // cout<< maxPt.z-minPt.z<<endl;
    // cout<<"------------------------"<<endl;
    if(maxPt.x-minPt.x>max_len) return false;
    if(maxPt.y-minPt.y>max_len) return false;
    if(maxPt.z-minPt.z>max_len) return false;
    return true;
}


bool ChessboardExtractor::check_board_size(pcl::PointCloud<pcl::PointXYZ>::Ptr board_pcd, Eigen::VectorXf& coeff)
{
    double max_padding = *max_element(m_checkerboard_padding.begin(), m_checkerboard_padding.end());
    max_padding *= 2.0;
    double len_x = (m_checkerboard_grid_size.width+1)*m_checkerboard_square_size+max_padding;
    double len_y = (m_checkerboard_grid_size.height+1)*m_checkerboard_square_size+max_padding;
    double len_z = 0.03;
    vector<double> lens = {len_x, len_y, len_z};
    sort(lens.begin(), lens.end());
    Eigen::Affine3f pose;
    Eigen::Vector3f size;
    fitMaximumContactBoundingBoxConstrained<pcl::PointXYZ>(board_pcd,coeff,pose, size, (float)0.1);
    vector<double> lens2 = {(double)size(0), (double)size(1), (double)size(2)};
    sort(lens2.begin(), lens2.end());

    // cout<< board_pcd->points.size()<<endl;
    // cout<< coeff <<endl;
    // cout<< "----------------------------"<<endl;
    // cout<< lens[0]<<" "<<lens[1]<<" "<<lens[2]<<endl;
    // cout<< size<<endl;
    // cout<< pose.matrix()<<endl;
    // cout<< "----------------------------"<<endl;
    if(abs(lens2[0]-lens[0])>m_checkerboard_square_size) return false;
    if(abs(lens2[1]-lens[1])>m_checkerboard_square_size) return false;
    if(abs(lens2[2]-lens[2])>0.03) return false;
    return true;
}


bool ChessboardExtractor::check_board_size(pcl::PointCloud<pcl::PointXYZ>::Ptr board_pcd,
                                              Eigen::VectorXf &coeff,
                                              Eigen::Affine3f &pose,
                                              Eigen::Vector3f &size)
{
    // double max_padding = *max_element(m_checkerboard_padding.begin(), m_checkerboard_padding.end());
    // max_padding *= 2.0;
    double len_x = (m_checkerboard_grid_size.width + 1) * m_checkerboard_square_size + m_checkerboard_padding[0]+ m_checkerboard_padding[2];
    double len_y = (m_checkerboard_grid_size.height + 1) * m_checkerboard_square_size +  m_checkerboard_padding[1]+ m_checkerboard_padding[3];
    double len_z = 0.03;
    vector<double> lens = {len_x, len_y, len_z};
    sort(lens.begin(), lens.end());

    fitMaximumContactBoundingBoxConstrained<pcl::PointXYZ>(board_pcd, coeff, pose, size, (float)0.1);
    vector<double> lens2 = {(double)size(0), (double)size(1), (double)size(2)};
    sort(lens2.begin(), lens2.end());
    // debug
    // {
    //     if(board_pcd->size()==317){
    //         std::cout<< abs(lens2[0] - lens[0])<<std::endl;
    //         std::cout<< abs(lens2[1] - lens[1])<<std::endl;
    //         std::cout<< abs(lens2[2] - lens[2])<<std::endl;
    //         std::cout<< m_checkerboard_square_size<<std::endl;
    //     }
    // }

    // debug
    // cout<< board_pcd->points.size()<<endl;
    // cout<< coeff <<endl;
    // cout<< "----------------------------"<<endl;
    // cout<< lens[0]<<" "<<lens[1]<<" "<<lens[2]<<endl;
    // cout<< size<<endl;
    // cout<< pose.matrix()<<endl;
    // cout<< "----------------------------"<<endl;
    if (abs(lens2[0] - lens[0]) > m_checkerboard_square_size)
        return false;
    if (abs(lens2[1] - lens[1]) > m_checkerboard_square_size)
        return false;
    if (abs(lens2[2] - lens[2]) > 0.03)
        return false;
    return true;
}


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
    reg.setMinClusterSize(150);
    reg.setMaxClusterSize(4000);
    // reg.setMinClusterSize(280);
    // reg.setMaxClusterSize(520);
    reg.setSearchMethod(tree);
    reg.setNumberOfNeighbours(60);
    reg.setInputCloud(input_pcd);
    reg.setInputNormals(normals);
    //reg.setSmoothnessThreshold(3.0 / 180.0 * M_PI);
    reg.setSmoothnessThreshold(5.0 / 180.0 * M_PI);
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
    //     // check_board_size(pcd_cluster);
    //     cout<< pcd_cluster->points.size() <<endl;
    //     // display_colored_by_depth(pcd_cluster);
    // }
    // exit(17);
    // cout<< "-------------------------"<<endl;
    return;
}


bool ChessboardExtractor::fitPlane(pcl::PointCloud<pcl::PointXYZ>::Ptr &input_pcd,
                                   std::vector<pcl::PointIndices> &indices_clusters,
                                       pcl::PointCloud<pcl::PointXYZ>::Ptr &plane_pcd)
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


bool ChessboardExtractor::extractChessboard(pcl::PointCloud<pcl::PointXYZ>::Ptr &input_pcd,
                                   std::vector<pcl::PointIndices> &indices_clusters,
                                       pcl::PointCloud<pcl::PointXYZ>::Ptr &chessboard_pcd)
{
    pcl::ExtractIndices<pcl::PointXYZ> extract;
    extract.setInputCloud(input_pcd);
    for (auto &cluster : indices_clusters)
    {
        pcl::PointCloud<pcl::PointXYZ>::Ptr pcd_cluster(new pcl::PointCloud<pcl::PointXYZ>);
        extract.setIndices(boost::shared_ptr<pcl::PointIndices>(new pcl::PointIndices(cluster)));
        extract.setNegative(false);
        extract.filter(*pcd_cluster);
        //debug
        // if(pcd_cluster->points.size()!=368) continue;
        // pcl::io::savePCDFileASCII ("/home/jc/Documents/stereo-lidar-calibration/exclude_dir/debug_data/chessboard_plane_0001.pcd", *pcd_cluster);

        pcl::SampleConsensusModelPlane<pcl::PointXYZ>::Ptr plane_model(
            new pcl::SampleConsensusModelPlane<pcl::PointXYZ>(pcd_cluster));
        pcl::RandomSampleConsensus<pcl::PointXYZ> ransac(plane_model);
        ransac.setDistanceThreshold(0.04);
        ransac.setMaxIterations(100);
        if (!ransac.computeModel()) continue;
        vector<int> inliers;
        ransac.getInliers(inliers);
        Eigen::VectorXf coeff(4);
        ransac.getModelCoefficients(coeff);
        double inliers_percentage = double(inliers.size()) / pcd_cluster->points.size();
        if (inliers_percentage > 0.9)
        {
            pcl::copyPointCloud(*pcd_cluster, inliers, *chessboard_pcd);
            
            if(!check_board_size(chessboard_pcd, coeff)) continue;
            // debug
            // display_colored_by_depth(chessboard_pcd);
            // exit(17);
            return true;
        }
    }
    // debug
    // exit(17);
    return false;
}



bool ChessboardExtractor::fitBoardCubic(pcl::PointCloud<pcl::PointXYZ>::Ptr &input_pcd,
                                   std::vector<pcl::PointIndices> &indices_clusters,
                                       pcl::PointCloud<pcl::PointXYZ>::Ptr &chessboard_pcd,
                                       Eigen::Affine3f& board_pose,
                                       Eigen::Vector3f& board_size)
{
    pcl::ExtractIndices<pcl::PointXYZ> extract;
    extract.setInputCloud(input_pcd);
    for (auto &cluster : indices_clusters)
    {
        pcl::PointCloud<pcl::PointXYZ>::Ptr pcd_cluster(new pcl::PointCloud<pcl::PointXYZ>);
        extract.setIndices(boost::shared_ptr<pcl::PointIndices>(new pcl::PointIndices(cluster)));
        extract.setNegative(false);
        extract.filter(*pcd_cluster);
        //debug
        // if(pcd_cluster->points.size()!=368) continue;
        // pcl::io::savePCDFileASCII ("/home/jc/Documents/stereo-lidar-calibration/exclude_dir/debug_data/chessboard_plane_0001.pcd", *pcd_cluster);


        pcl::SampleConsensusModelPlane<pcl::PointXYZ>::Ptr plane_model(
            new pcl::SampleConsensusModelPlane<pcl::PointXYZ>(pcd_cluster));
        pcl::RandomSampleConsensus<pcl::PointXYZ> ransac(plane_model);
        ransac.setDistanceThreshold(0.04);
        ransac.setMaxIterations(100);
        if (!ransac.computeModel()) continue;
        vector<int> inliers;
        ransac.getInliers(inliers);
        Eigen::VectorXf coeff(4);
        ransac.getModelCoefficients(coeff);
        double inliers_percentage = double(inliers.size()) / pcd_cluster->points.size();
        //debug
        // {
        //     if(input_pcd->size()==17448){
        //         std::cout<< pcd_cluster->size()<<std::endl;
        //         std::cout<< inliers_percentage <<std::endl;
        //         display_colored_by_depth(pcd_cluster);
        //     }
        // }

        if (inliers_percentage > 0.9)
        {
            pcl::copyPointCloud(*pcd_cluster, inliers, *chessboard_pcd);

            if(!check_board_size(chessboard_pcd, coeff, board_pose, board_size)) continue;
            // debug
            // display_colored_by_depth(chessboard_pcd);
            // exit(17);
            return true;
        }
    }
    // debug
    // exit(17);
    return false;
}
