#include "optimization.hpp"

void OptimizationLC::addPointToPlaneConstraints(ceres::Problem& problem, pcl::PointCloud<pcl::PointXYZIr>::Ptr& plane_pcd)
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
                new PointToPlaneError(point_3d, m_plane_normal, m_plane_centroid, w));
        problem.AddResidualBlock(cost_function, loss_function, m_R_t.data());
    }
}


void OptimizationLC::addPointToLineConstriants(ceres::Problem& problem, std::vector<pcl::PointCloud<pcl::PointXYZIr>::Ptr>& lines_pcd)
{
    for(int i=0;i<4;++i){
        auto& line_pcd = lines_pcd[i];
        auto& normal = m_line_normal[i];
        ceres::LossFunction *loss_function = NULL;
        double w = 1/sqrt((double)line_pcd->points.size());
        for(int j = 0; j < line_pcd->points.size(); j++){
            Eigen::Vector3d point_3d(line_pcd->points[i].x,
                                    line_pcd->points[i].y,
                                    line_pcd->points[i].z);
            // Add residual here
            ceres::CostFunction *cost_function = new
                        ceres::AutoDiffCostFunction<PointToLineError, 1, 6>
                        (new PointToLineError(point_3d, normal, w));
            problem.AddResidualBlock(cost_function, loss_function, m_R_t.data());
        }
    }
    return;
}


ceres::Problem OptimizationLC::constructProblemForL1C1()
{
    ceres::Problem problem;
    problem.AddParameterBlock(m_R_t.data(), 6);
    return problem;
}

void OptimizationLC::solveProblem(ceres::Problem& problem){
    ceres::Solver::Options options;
    options.max_num_iterations = 200;
    options.linear_solver_type = ceres::SPARSE_NORMAL_CHOLESKY;
    options.minimizer_progress_to_stdout = false;
    ceres::Solver::Summary summary;
    ceres::Solve(options, &problem, &summary);
}

