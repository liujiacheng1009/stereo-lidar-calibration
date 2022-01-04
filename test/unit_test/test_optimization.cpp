#include "extractChessboard.hpp"
#include "extractImageFeature.hpp"
#include "extractLidarFeature.hpp"
#include "optimization.hpp"
#include "utils.hpp"
#include "config.hpp"
#include <pcl/common/geometry.h>
#include <sophus/so3.hpp>
#include "gtest/gtest.h"
using namespace std;
using namespace Eigen;
using namespace cv;
using namespace pcl;


TEST(OptimizationTest, ClosedupError){
    VectorXd Rt_l1_c1(6), Rt_l1_c2(6), Rt_c1_c2(6);

    Rt_l1_c1<< 0.0,0.0,0.0,0.02,0.0,0.0;
    Rt_l1_c2<<  0.0,0.0,0.0,-0.10,0.0,0.0;
    Rt_c1_c2 << -0.0,-0.0,0.0,-0.12,0.0,0.0;
    Optimizer optimizer;
    ceres::Problem problem;
    optimizer.addClosedLoopConstraints(problem,Rt_l1_c1, Rt_l1_c2, Rt_c1_c2);
    ceres::Solver::Options options;
    options.max_num_iterations = 500;
    options.linear_solver_type = ceres::SPARSE_NORMAL_CHOLESKY;
    options.minimizer_progress_to_stdout = false;
    ceres::Solver::Summary summary;
    ceres::Solve(options, &problem, &summary);
    EXPECT_NEAR(0, summary.initial_cost, 1e-4);
}