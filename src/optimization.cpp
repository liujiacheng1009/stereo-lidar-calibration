#include "optimization.hpp"

void OptimizationLC::addPointToPlaneConstraints(ceres::Problem &problem,
                                                pcl::PointCloud<pcl::PointXYZ>::Ptr &plane_pcd,
                                                Eigen::Vector3d &plane_normal,
                                                Eigen::Vector3d &plane_centroid)
{
    ceres::LossFunction *loss_function = NULL;
    double w = 1 / sqrt((double)plane_pcd->points.size());
    for (int i = 0; i < plane_pcd->points.size(); ++i)
    {
        Eigen::Vector3d point_3d(plane_pcd->points[i].x,
                                 plane_pcd->points[i].y,
                                 plane_pcd->points[i].z);
        ceres::CostFunction *cost_function =
            new ceres::AutoDiffCostFunction<PointToPlaneError, 1, 6>(
                new PointToPlaneError(point_3d, plane_normal, plane_centroid, w));
        problem.AddResidualBlock(cost_function, loss_function, m_R_t.data());
    }
    return;
}

void OptimizationLC::addPointToLineConstriants(ceres::Problem &problem,
                                               std::vector<pcl::PointCloud<pcl::PointXYZ>> &lines_pcd,
                                               std::vector<Eigen::VectorXd> &line_normals)
{
    assert(lines_pcd.size() == 4);
    for (int i = 0; i < 4; ++i)
    {
        auto& line_pcd = lines_pcd[i];
        auto &normal = line_normals[i];
        ceres::LossFunction *loss_function = NULL;
        double w = 1 / sqrt((double)line_pcd.points.size());
        for (int j = 0; j < line_pcd.points.size(); j++)
        {
            Eigen::Vector3d point_3d(line_pcd.points[i].x,
                                     line_pcd.points[i].y,
                                     line_pcd.points[i].z);
            ceres::CostFunction *cost_function = new ceres::AutoDiffCostFunction<PointToLineError1, 1, 6>(new PointToLineError1(point_3d, normal, w));
            problem.AddResidualBlock(cost_function, loss_function, m_R_t.data());
        }
    }
    return;
}

void OptimizationLC::addPointToPointConstriants(ceres::Problem &problem,
                                                vector<Vector3d> &lidar_corners_3d,
                                                vector<Vector3d> &image_corners_3d)
{
    assert(image_corners_3d.size() == lidar_corners_3d.size());
    int n = image_corners_3d.size();
    double w = 1 / sqrt((double)n);
    for (int i = 0; i < n; ++i)
    {
        Eigen::Vector3d lidar_point = lidar_corners_3d[i];
        Eigen::Vector3d image_point = image_corners_3d[i];
        ceres::LossFunction *loss_function = NULL;
        ceres::CostFunction *cost_function = new ceres::AutoDiffCostFunction<PointToPointError1, 3, 6>(new PointToPointError1(lidar_point,image_point, w));
        problem.AddResidualBlock(cost_function, loss_function, m_R_t.data());
    }
    return;
}

// ceres::Problem OptimizationLC::constructProblem()
// {
//     ceres::Problem problem;
//     problem.AddParameterBlock(m_R_t.data(), 6);
//     return problem;
// }

// void OptimizationLC::solveProblem(ceres::Problem& problem){
//     ceres::Solver::Options options;
//     options.max_num_iterations = 200;
//     options.linear_solver_type = ceres::SPARSE_NORMAL_CHOLESKY;
//     options.minimizer_progress_to_stdout = false;
//     ceres::Solver::Summary summary;
//     ceres::Solve(options, &problem, &summary);
// }

// pcl 接口求解 ICP
// bool ArucoCalib::computeTransICP(const pcl::PointCloud<pcl::PointXYZ>::Ptr source_cloud,
//                                  const pcl::PointCloud<pcl::PointXYZ>::Ptr target_cloud, Eigen::Matrix4d& trans)
// {
//   // using pcl icp use ransac to solve non-linear optimation
//   pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
//   icp.setInputSource(source_cloud);
//   icp.setInputTarget(target_cloud);
//   icp.setTransformationEpsilon(1e-8);
//   pcl::PointCloud<pcl::PointXYZ> Final;
//   icp.align(Final);
//   if (!icp.hasConverged())
//   {
// #if LCDEBUG
//     WARN << "icp has unconverged!" << REND;
// #endif
//     return false;
//   }

//   Eigen::Matrix4f temp_trans = icp.getFinalTransformation();  // colomn major
//   trans = temp_trans.cast<double>();
//   return true;
// }



// SVD 分解计算 trans
// bool ArucoCalib::computeTransSVD(const pcl::PointCloud<pcl::PointXYZ>::Ptr source_cloud,
//                                  const pcl::PointCloud<pcl::PointXYZ>::Ptr target_cloud, Eigen::Matrix4d& trans)
// {
//   // turn into cv::Point3f
//   int corner_size = source_cloud->size();
//   std::vector<cv::Point3f> pts1(corner_size), pts2(corner_size);
//   for (size_t i = 0; i < corner_size; ++i)
//   {
//     pcl::PointXYZ cloud_point = source_cloud->points[i], camera_point = target_cloud->points[i];
//     pts1[i] = cv::Point3f(camera_point.x, camera_point.y, camera_point.z);
//     pts2[i] = cv::Point3f(cloud_point.x, cloud_point.y, cloud_point.z);
//   }
//   // compute center of mass
//   cv::Point3f p1(0, 0, 0), p2(0, 0, 0);
//   for (size_t i = 0; i < corner_size; ++i)
//   {
//     p1 += pts1[i];
//     p2 += pts2[i];
//   }
//   p1 /= corner_size;
//   p2 /= corner_size;

//   // remove centroid
//   std::vector<cv::Point3f> qts1(corner_size), qts2(corner_size);
//   for (size_t i = 0; i < corner_size; ++i)
//   {
//     qts1[i] = pts1[i] - p1;
//     qts2[i] = pts2[i] - p2;
//   }

//   // compute qts1 *  qts2^T
//   Eigen::Matrix3d w = Eigen::Matrix3d::Zero();
//   for (size_t i = 0; i < corner_size; ++i)
//   {
//     w +=
//         Eigen::Vector3d(qts1[i].x, qts1[i].y, qts1[i].z) * Eigen::Vector3d(qts2[i].x, qts2[i].y, qts2[i].z).transpose();
//   }

//   // svd on w
//   Eigen::JacobiSVD<Eigen::Matrix3d> svd(w, Eigen::ComputeFullU | Eigen::ComputeFullV);
//   Eigen::Matrix3d u = svd.matrixU();
//   Eigen::Matrix3d v = svd.matrixV();

//   // get r and t
//   Eigen::Matrix3d r = u * (v.transpose());
//   Eigen::Vector3d t = Eigen::Vector3d(p1.x, p1.y, p1.z) - r * Eigen::Vector3d(p2.x, p2.y, p2.z);

//   // convert to cv::Mat
//   trans = Eigen::Matrix4d::Identity();
//   trans.block<3, 3>(0, 0) = r;
//   trans.block<3, 1>(0, 3) = t;
//   return true;
// }

void OptimizationLCC::addPointToPlaneConstraints(ceres::Problem &problem,
                                                pcl::PointCloud<pcl::PointXYZ>::Ptr &plane_pcd,
                                                Eigen::Vector3d &plane_normal,
                                                Eigen::Vector3d &plane_centroid,
                                                Eigen::VectorXd &params)
{
    ceres::LossFunction *loss_function = NULL;
    double w = 1 / sqrt((double)plane_pcd->points.size());
    for (int i = 0; i < plane_pcd->points.size(); ++i)
    {
        Eigen::Vector3d point_3d(plane_pcd->points[i].x,
                                 plane_pcd->points[i].y,
                                 plane_pcd->points[i].z);
        ceres::CostFunction *cost_function =
            new ceres::AutoDiffCostFunction<PointToPlaneError, 1, 6>(
                new PointToPlaneError(point_3d, plane_normal, plane_centroid, w));
        problem.AddResidualBlock(cost_function, loss_function, params.data());
    }
    return;
}

void OptimizationLCC::addPointToLineConstriants(ceres::Problem &problem,
                                               std::vector<pcl::PointCloud<pcl::PointXYZ>> &lines_pcd,
                                               std::vector<Eigen::VectorXd> &line_normals,
                                               Eigen::VectorXd &params)
{
    assert(lines_pcd.size() == 4);
    for (int i = 0; i < 4; ++i)
    {
        auto& line_pcd = lines_pcd[i];
        auto &normal = line_normals[i];
        ceres::LossFunction *loss_function = NULL;
        double w = 1 / sqrt((double)line_pcd.points.size());
        for (int j = 0; j < line_pcd.points.size(); j++)
        {
            Eigen::Vector3d point_3d(line_pcd.points[i].x,
                                     line_pcd.points[i].y,
                                     line_pcd.points[i].z);
            ceres::CostFunction *cost_function = new ceres::AutoDiffCostFunction<PointToLineError1, 1, 6>(new PointToLineError1(point_3d, normal, w));
            problem.AddResidualBlock(cost_function, loss_function, params.data());
        }
    }
    return;
}

void OptimizationLCC::addPointToPointConstriants(ceres::Problem &problem,
                                                vector<Vector3d> &lidar_corners_3d,
                                                vector<Vector3d> &image_corners_3d,
                                                VectorXd &params)
{
    assert(image_corners_3d.size() == lidar_corners_3d.size());
    int n = image_corners_3d.size();
    double w = 1 / sqrt((double)n);
    for (int i = 0; i < 4; ++i)
    {
        Vector3d& lidar_point = lidar_corners_3d[i];
        Vector3d& image_point = image_corners_3d[i];
        ceres::LossFunction *loss_function = NULL;
        ceres::CostFunction *cost_function = new ceres::AutoDiffCostFunction<PointToPointError, 1, 6>(new PointToPointError(lidar_point,image_point, w));
        problem.AddResidualBlock(cost_function, loss_function, params.data());
    }
    return;
}


void OptimizationLCC::addStereoMatchingConstraints(ceres::Problem &problem,
                                std::vector<Vector2d> &left_image_corners,
                                std::vector<Vector2d> &right_image_corners)
{
    assert(left_image_corners.size() == right_image_corners.size());
    // Eigen::MatrixXd camera_mat;
    // camera_mat.resize(3,3);
    // camera_mat << camera_matrix.at<double>(0, 0), camera_matrix.at<double>(0, 1), camera_matrix.at<double>(0, 2),
    //     camera_matrix.at<double>(1, 0), camera_matrix.at<double>(1, 1), camera_matrix.at<double>(1, 2),
    //     camera_matrix.at<double>(2, 0), camera_matrix.at<double>(2, 1), camera_matrix.at<double>(2, 2);
    double w = 1 / sqrt((double)left_image_corners.size());
    for (int i = 0; i < left_image_corners.size(); ++i)
    {
        Eigen::Vector2d& left_image_corner = left_image_corners[i] ;
        Eigen::Vector2d& right_image_corner = right_image_corners[i];
        ceres::LossFunction *loss_function = NULL;
        ceres::CostFunction *cost_function = new ceres::AutoDiffCostFunction<StereoMatchingError, 1, 6>(
            new StereoMatchingError(left_image_corner,right_image_corner, w));
        problem.AddResidualBlock(cost_function, loss_function, m_Rt_c1_c2.data());
    }
    return;

}

void Optimizer::addStereoMatchingConstraints(ceres::Problem &problem,
                                std::vector<Vector2d> &left_image_corners,
                                std::vector<Vector2d> &right_image_corners,
                                VectorXd& params)
{
    assert(left_image_corners.size() == right_image_corners.size());
    // Eigen::MatrixXd camera_mat;
    // camera_mat.resize(3,3);
    // camera_mat << camera_matrix.at<double>(0, 0), camera_matrix.at<double>(0, 1), camera_matrix.at<double>(0, 2),
    //     camera_matrix.at<double>(1, 0), camera_matrix.at<double>(1, 1), camera_matrix.at<double>(1, 2),
    //     camera_matrix.at<double>(2, 0), camera_matrix.at<double>(2, 1), camera_matrix.at<double>(2, 2);
    double w = 1 / sqrt((double)left_image_corners.size());
    for (int i = 0; i < left_image_corners.size(); ++i)
    {
        Eigen::Vector2d& left_image_corner = left_image_corners[i] ;
        Eigen::Vector2d& right_image_corner = right_image_corners[i];
        ceres::LossFunction *loss_function = NULL;
        ceres::CostFunction *cost_function = new ceres::AutoDiffCostFunction<StereoMatchingError, 1, 6>(
            new StereoMatchingError(left_image_corner,right_image_corner, w));
        problem.AddResidualBlock(cost_function, loss_function, params.data());
    }
    return;
}

void Optimizer::addStereoMatchingConstraints(ceres::Problem &problem,
                                             vector<Vector3d> &P1,
                                             vector<Vector2d> &p1,
                                             vector<Vector3d> &P2,
                                             vector<Vector2d> &p2,
                                             VectorXd &params)
{
    assert(P1.size() == P2.size());
    assert(p1.size() == p2.size());
    double w = 1 / sqrt((double)P1.size());
    for (int i = 0; i < P1.size(); ++i)
    {
        Vector3d &observed_3d_points_in_cam1 = P1[i];
        Vector2d &observed_2d_points_in_cam1 = p1[i];
        Vector3d &observed_3d_points_in_cam2 = P2[i];
        Vector2d &observed_2d_points_in_cam2 = p2[i];
        ceres::LossFunction *loss_function = NULL;
        ceres::CostFunction *cost_function = new ceres::AutoDiffCostFunction<StereoReprojectionError, 4, 6>(
            new StereoReprojectionError(observed_3d_points_in_cam1, observed_2d_points_in_cam1,
                                    observed_3d_points_in_cam2, observed_2d_points_in_cam2, w));
        problem.AddResidualBlock(cost_function, loss_function, params.data());
    }
    return;
}

void Optimizer::addPointToPointConstriants(ceres::Problem &problem,
                                                vector<Vector3d> &lidar_corners_3d,
                                                vector<Vector3d> &image_corners_3d,
                                                VectorXd &params)
{
    assert(image_corners_3d.size() == lidar_corners_3d.size());
    int n = image_corners_3d.size();
    double w = 1 / sqrt((double)n);
    for (int i = 0; i < 4; ++i)
    {
        Vector3d& lidar_point = lidar_corners_3d[i];
        Vector3d& image_point = image_corners_3d[i];
        ceres::LossFunction *loss_function = NULL;
        ceres::CostFunction *cost_function = new ceres::AutoDiffCostFunction<PointToPointError, 1, 6>(new PointToPointError(lidar_point,image_point, w));
        problem.AddResidualBlock(cost_function, loss_function, params.data());
    }
    return;
}

void Optimizer::addClosedLoopConstraints(ceres::Problem &problem,
                                    VectorXd& params1,
                                    VectorXd& params2,
                                    VectorXd& params3)
{
    double w = 1.0;
    ceres::LossFunction *loss_function = NULL;
    ceres::CostFunction *cost_function = new ceres::AutoDiffCostFunction<ClosedupError,2,6,6,6>(new ClosedupError(w));
    problem.AddResidualBlock(cost_function, loss_function, params1.data(),params2.data(), params3.data());
}