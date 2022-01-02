#include "extractLidarFeature.hpp"
#include "utils.hpp"

bool loadPointXYZ(string& path, PointCloud<PointXYZ>::Ptr& pcd)
{
    PCDReader reader;
    PCLPointCloud2::Ptr cloud(new PCLPointCloud2);
    if(reader.read(path, *cloud)!=0){
        return false;
    }
    pcl::fromPCLPointCloud2(*cloud, *pcd);
    return true;
}

void getValidDataSet(ImageResults& images_features, CloudResults& cloud_features)
{
    auto& valid_image_index = images_features.valid_index;
    auto& valid_cloud_index = cloud_features.valid_index;
    set<int> valid_index;
    for(auto& id:valid_image_index){ // 取交集
        if(find(valid_cloud_index.begin(),valid_cloud_index.end(), id)!= valid_cloud_index.end()){
            valid_index.insert(id);
        }
    }
    vector<vector<Vector3d>> valid_image_corners_3d;
    vector<vector<Vector3d>> valid_cloud_corners_3d;
    for(int i = 0;i<valid_image_index.size();++i){
        if(valid_index.count(valid_image_index[i])){
            valid_image_corners_3d.push_back(images_features.corners_3d[i]);
        }
    }
    for(int i = 0;i<valid_cloud_index.size();++i){
        if(valid_index.count(valid_cloud_index[i])){
            valid_cloud_corners_3d.push_back(cloud_features.corners_3d[i]);
        }
    }
    images_features.corners_3d = valid_image_corners_3d;
    cloud_features.corners_3d = valid_cloud_corners_3d;
    return;
}

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
// template<typename T>
// bool LidarFeatureDetector<T>::extractFourLines(pcl::PointCloud<pcl::PointXYZ>::Ptr &edge_pcd,
//                                             std::vector<Eigen::VectorXf> &lines_params,
//                                             std::vector<pcl::PointCloud<pcl::PointXYZ>> &lines_points)
// {
//     lines_params.clear();
//     lines_points.clear();
//     for (int i = 0; i < 4; i++)
//     {
//         pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_ptr(edge_pcd);
//         pcl::SampleConsensusModelLine<pcl::PointXYZ>::Ptr model_l(new pcl::SampleConsensusModelLine<pcl::PointXYZ>(cloud_ptr));
//         pcl::RandomSampleConsensus<pcl::PointXYZ> ransac_l(model_l);
//         ransac_l.setDistanceThreshold(0.03);
//         ransac_l.computeModel();
//         std::vector<int> line_inliers;
//         ransac_l.getInliers(line_inliers);
//         if (!line_inliers.empty())
//         {
//             pcl::PointCloud<pcl::PointXYZ> line_i;
//             pcl::copyPointCloud<pcl::PointXYZ>(*edge_pcd,
//                                                line_inliers,
//                                                line_i);
//             lines_points.push_back(line_i);                       // 边界点云
//             lines_params.push_back(ransac_l.model_coefficients_); // 边界线的参数
//         }
//         else
//         {
//             break;
//         }
//         pcl::PointCloud<pcl::PointXYZ> plane_no_line;
//         remove_inliers(*cloud_ptr, line_inliers, plane_no_line);
//         *edge_pcd = plane_no_line;
//     }
//     if (lines_points.size() == 4)
//     {
//         reorder_lines1(lines_params,lines_points );
//         return true;
//     }
//     return false;
// }
// template<typename T>
// void LidarFeatureDetector<T>::reorder_lines(std::vector<Eigen::VectorXf> &v_line_coeff,
//                                          std::vector<pcl::PointCloud<pcl::PointXYZ>> &v_line_cloud)
// {
//     int line_num = v_line_coeff.size();
//     std::vector<std::pair<Eigen::VectorXf, pcl::PointCloud<pcl::PointXYZ>>> coeff_cloud_v;
//     for (size_t i = 0; i < line_num; ++i)
//     {
//         coeff_cloud_v.emplace_back(std::make_pair(v_line_coeff[i], v_line_cloud[i]));
//     }
//     std::sort(coeff_cloud_v.begin(), coeff_cloud_v.end(), [](std::pair<Eigen::VectorXf, pcl::PointCloud<pcl::PointXYZ>> &lhs, std::pair<Eigen::VectorXf, pcl::PointCloud<pcl::PointXYZ>> &rhs)
//               { return lhs.first(2) > rhs.first(2); });
//     // sort by descending order
//     // The six coefficients of the line are given by a point on the line and the direction of the line as:
//     // [point_on_line.x point_on_line.y point_on_line.z line_direction.x line_direction.y line_direction.z]
//     // line order in clockwise: top-left, top-right, bottom-right, bottom-left
//     if (coeff_cloud_v[0].first(1) < coeff_cloud_v[1].first(1))
//     {
//         std::swap(coeff_cloud_v[0], coeff_cloud_v[1]);
//     }
//     if (coeff_cloud_v[2].first(1) > coeff_cloud_v[3].first(1))
//     {
//         std::swap(coeff_cloud_v[2], coeff_cloud_v[3]);
//     }

//     v_line_coeff.clear();
//     v_line_cloud.clear();
//     for (size_t i = 0; i < line_num; ++i)
//     {
//         // std::cout << "coeff: " << coeff_cloud_v[i].first.size() << std::endl;
//         // std::cout << "cloud: " << coeff_cloud_v[i].second.size() << std::endl;
//         v_line_coeff.emplace_back(coeff_cloud_v[i].first);
//         v_line_cloud.emplace_back(coeff_cloud_v[i].second);
//     }
//     return;
// }
// template<typename T>
// void LidarFeatureDetector<T>::reorder_lines1(std::vector<Eigen::VectorXf> &v_line_coeff,
//                                          std::vector<pcl::PointCloud<pcl::PointXYZ>> &v_line_cloud)
// {
//     assert(v_line_coeff.size()==v_line_cloud.size()&& v_line_cloud.size()==4);
//     Eigen::Vector3d line0,line1;
//     line0 << v_line_coeff[0](3) , v_line_coeff[0](4), v_line_coeff[0](5);
//     line1<< v_line_coeff[1](3) , v_line_coeff[1](4), v_line_coeff[1](5);    
//     double rad =line0.dot(line1) /(line0.norm()*line1.norm());
//     if(abs(rad)>0.2){
//         Eigen::VectorXf line = v_line_coeff[1];
//         pcl::PointCloud<pcl::PointXYZ> cloud = v_line_cloud[1];
//         v_line_coeff[1] = v_line_coeff[2];
//         v_line_cloud[1] = v_line_cloud[2];
//         v_line_cloud[2] = cloud;
//         v_line_coeff[2] = line;
//         // swap(v_line_cloud[1], v_line_cloud[2]);
//     }
//     return;
// }
// template<typename T>
// void LidarFeatureDetector<T>::remove_inliers(const pcl::PointCloud<pcl::PointXYZ> &cloud_in,
//                                           std::vector<int> inliers_indices,
//                                           pcl::PointCloud<pcl::PointXYZ> &cloud_out)
// {
//     std::vector<int> outliers_indicies;
//     for (size_t i = 0; i < cloud_in.size(); i++)
//     {
//         if (find(inliers_indices.begin(), inliers_indices.end(), i) == inliers_indices.end())
//         {
//             outliers_indicies.push_back(i);
//         }
//     }
//     pcl::copyPointCloud<pcl::PointXYZ>(cloud_in, outliers_indicies, cloud_out);
// }
// // bool LidarFeatureDetector::extractFourEdges(pcl::PointCloud<pcl::PointXYZIr>::Ptr &edge_pcd,
// //                                             std::vector<pcl::PointCloud<pcl::PointXYZI>> &line_points,
// //                                             std::vector<Eigen::VectorXf> &line_params)
// // {
// //     for (int i = 0; i < 4; i++)
// //     {
// //         pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_ptr(edge_pcd);
// //         pcl::SampleConsensusModelLine<pcl::PointXYZI>::Ptr model_l(new pcl::SampleConsensusModelLine<pcl::PointXYZI>(cloud_ptr));
// //         pcl::RandomSampleConsensus<pcl::PointXYZI> ransac_l(model_l);
// //         ransac_l.setDistanceThreshold(0.02);
// //         ransac_l.computeModel();
// //         std::vector<int> line_inliers;
// //         ransac_l.getInliers(line_inliers);
// //         if (!line_inliers.empty())
// //         {
// //             pcl::PointCloud<pcl::PointXYZI> line_i;
// //             pcl::copyPointCloud<pcl::PointXYZI>(*edge_pcd, line_inliers, line_i);
// //             line_points.push_back(line_i);                       // 边界点云
// //             line_params.push_back(ransac_l.model_coefficients_); // 边界线的参数
// //         }
// //         else
// //         {
// //             break;
// //         }
// //         pcl::PointCloud<pcl::PointXYZI> plane_no_line;
// //         remove_inliers(*cloud_ptr, line_inliers, plane_no_line);
// //         *cloud_msg_pcl = plane_no_line;
// //     }
// //     if (line_points.size() == 4)
// //     {
// //         return true;
// //     }
// //     return false;
// // }

// // void LidarFeatureDetector::detectPlane(pcl::PointCloud<PointXYZIr>::Ptr& plane_pcd)
// // {
// //     pcl::PointCloud<PointXYZIr>::Ptr cloud_in_ptr(new pcl::PointCloud<PointXYZIr>);
// //     *cloud_in_ptr = m_marker_board_pcd;
// //     pcl::SampleConsensusModelPlane<pcl::PointXYZIr>::Ptr model_p(new pcl::SampleConsensusModelPlane<pcl::PointXYZIr>(cloud_in_ptr));
// //     pcl::RandomSampleConsensus<PointXYZIr> ransac(model_p);
// //     ransac.setDistanceThreshold(0.05);
// //     bool model_computed = ransac.computeModel();
// //     std::vector<int> inlier_indices;
// //     if (model_computed) {
// //         ransac.getInliers(inlier_indices);
// //         pcl::copyPointCloud<PointXYZIr>(*cloud_in_ptr,inlier_indices,plane_pcd);
// //     }
// //     return;
// // }

// // void LidarFeatureDetector::getCorners(std::vector<Eigen::VectorXf> &line_params,
// //                                      vector<pcl::PointXYZ> &chessboard_corners)
// // {
// //     Eigen::Vector4f p1, p2, p_intersect;
// //     for(int i=0;i<4;++i){
// //         pcl::lineToLineSegment(line_params[i],line_params[(i+1)%4], p1,p2);
// //         for (int j = 0; j < 4; j++) {
// //             p_intersect(j) = (p1(j) + p2(j)) / 2.0;
// //         }
// //         chessboard_corners->push_back(pcl::PointXYZ(p_intersect(0), p_intersect(1), p_intersect(2)));
// //     }
// //     return;
// // }
// template<typename T>
// void LidarFeatureDetector<T>::estimateFourCorners(std::vector<Eigen::VectorXf> &line_params, std::vector<pcl::PointXYZ> &corners)
// {
//     Eigen::Vector4f p1, p2, p_intersect;
//     for (int i = 0; i < 4; ++i)
//     {
//         pcl::lineToLineSegment(line_params[i], line_params[(i + 1) % 4], p1, p2);
//         for (int j = 0; j < 4; j++)
//         {
//             p_intersect(j) = (p1(j) + p2(j)) / 2.0;
//         }
//         corners.push_back(pcl::PointXYZ(p_intersect(0), p_intersect(1), p_intersect(2)));
//     }
//     return;
// }
// template<typename T>
// void LidarFeatureDetector<T>::extractEdgeCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr &input_cloud, pcl::PointCloud<pcl::PointXYZ>::Ptr &edge_pcd)
// {
//     edge_pcd->clear();
//     pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> n; 
//     pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
//     n.setInputCloud(input_cloud);
//     n.setSearchMethod(tree);
//     n.setRadiusSearch(1);
//     pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
//     n.compute(*normals);

//     pcl::BoundaryEstimation<pcl::PointXYZ, pcl::Normal, pcl::Boundary> boundEst; 
//     boundEst.setInputCloud(input_cloud);
//     boundEst.setInputNormals(normals);
//     boundEst.setRadiusSearch(1);
//     boundEst.setAngleThreshold(M_PI / 4); 

//     boundEst.setSearchMethod(tree);
//     pcl::PointCloud<pcl::Boundary> boundaries;
//     boundEst.compute(boundaries);
//     for (int i = 0; i < input_cloud->points.size(); i++)
//     {

//         if (boundaries[i].boundary_point > 0)
//         {
//             edge_pcd->push_back(input_cloud->points[i]);
//         }
//     }
//     return;
// }

// // 判断四个角点是否有效
// // bool ArucoCalib::isValidCorners(const pcl::PointCloud<pcl::PointXYZ>::Ptr corners_ptr)
// // {
// //   int corner_num = corners_ptr->size();
// //   if (corner_num != 4)
// //   {
// //     return false;
// //   }

// //   const float board_length = laser_proc_ptr_->board_length_;  // meter
// //   const float board_angle = M_PI / 2.0;

// //   // 计算距离评分和角度评分
// //   pcl::PointXYZ pre_corner, cur_corner, next_corner;
// //   float length_diff = 0.0, angle_diff = 0.0;
// //   for (size_t i = 0; i < corner_num; ++i)
// //   {
// //     pre_corner = corners_ptr->points[(i + 3) % 4];
// //     cur_corner = corners_ptr->points[i];
// //     next_corner = corners_ptr->points[(i + 1) % 4];
// //     float dist = pcl::euclideanDistance(cur_corner, next_corner);
// //     length_diff += std::abs(dist - board_length);

// //     Eigen::Vector3f a, b;
// //     a << (cur_corner.x - pre_corner.x), (cur_corner.y - pre_corner.y), (cur_corner.z - pre_corner.z);
// //     b << (cur_corner.x - next_corner.x), (cur_corner.y - next_corner.y), (cur_corner.z - next_corner.z);
// //     float angle = std::acos(a.dot(b) / (a.norm() * b.norm()));
// //     angle_diff += std::abs(angle - board_angle);
// //   }
// //   length_diff /= corner_num;
// //   angle_diff /= corner_num;

// //   float length_core = 1 - length_diff / board_length;
// //   float angle_core = 1 - angle_diff / board_angle;

// // #if LCDEBUG
// //   INFO << "length core is: " << length_core * 100 << REND;
// //   INFO << "angle core is: " << angle_core * 100 << REND;
// // #endif

// //   if (length_core < 0.95 || angle_core < 0.95)
// //   {
// //     return false;
// //   }
// //   return true;
// // }

// template<typename T>
// bool LidarFeatureDetector<T>::extractPlaneCloud(PointCloud<PointXYZ>::Ptr &input_cloud, PointCloud<PointXYZ>::Ptr &plane_pcd)
// {
//     m_chessboard_extractor.pass_filter(input_cloud); // 带通滤波
//     vector<PointIndices> indices_clusters;
//     m_chessboard_extractor.pcd_clustering(input_cloud, indices_clusters); // 聚类
//     if(m_chessboard_extractor.fitPlane(input_cloud, indices_clusters,plane_pcd)){
//         //Todo 保留平面上的点
//         return true;
//     }
//          // 查找平面
//     return false;
// }

// // 调整lidar points 的顺序
// template<typename T>
// void LidarFeatureDetector<T>::reorder_corners(std::vector<pcl::PointXYZ>& ori_corners, std::vector<pcl::PointXYZ>& reordered_corners)
// {
//     reordered_corners.clear();
//     float d1 = pcl::geometry::distance(ori_corners[0], ori_corners[0]);
//     float d2 = pcl::geometry::distance(ori_corners[1], ori_corners[2]);
//     if(d1<d2){
//         std::reverse(ori_corners.begin()+1, ori_corners.end());
//     }
//     if(std::max(ori_corners[0].z, ori_corners[1].z)< std::max(ori_corners[2].z, ori_corners[3].z)){
//         reordered_corners.push_back(ori_corners[2]);
//         reordered_corners.push_back(ori_corners[3]);
//         reordered_corners.push_back(ori_corners[0]);
//         reordered_corners.push_back(ori_corners[1]);
//     }else{
//         reordered_corners = ori_corners;
//     }
//     if(reordered_corners[0].y<reordered_corners[1].y){
//         std::reverse(reordered_corners.begin(), reordered_corners.begin()+2);
//         std::reverse(reordered_corners.begin()+2, reordered_corners.end());
//     }
//     return;
// }

// // 调整边缘直线上的点云顺序
// template<typename T>
// void LidarFeatureDetector<T>::reorder_line_points(std::vector<pcl::PointXYZ>  &reordered_corners,
//                          vector<PointCloud<PointXYZ>> &reordered_lines_points,
//                          vector<pair<pair<PointXYZ, PointXYZ>, PointCloud<PointXYZ>>> &line_corners_to_points)
// {
//     reordered_lines_points.clear();
//     for(int i=0;i<4;++i){
//         auto p1 = make_pair(reordered_corners[i], reordered_corners[(i+1)%4]);
//         auto p2 = make_pair(reordered_corners[(i+1)%4], reordered_corners[i]);
//         for(int j=0;j<4;++j){
//             auto& corners_points = line_corners_to_points[j];
//             if(pcl::geometry::distance(corners_points.first.first, p1.first)< 1e-2 && pcl::geometry::distance(corners_points.first.second, p1.second)<1e-2){
//                 reordered_lines_points.push_back(corners_points.second);
//             }
//             if(pcl::geometry::distance(corners_points.first.first, p2.first)< 1e-2 && pcl::geometry::distance(corners_points.first.second, p2.second)<1e-2){
//                 reordered_lines_points.push_back(corners_points.second);
//             }
//         }
//     }
//     return;
// }
