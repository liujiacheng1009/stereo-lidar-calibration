#include "optimization.hpp"
#include "utils.hpp"
#include "matlab_data.hpp"


using namespace std;
using namespace Eigen;
using namespace cv;


void addPointCorrespondenceConstraints(
    ceres::Problem &problem,
    VectorXd& params,
    Vector3d& lidar_point,
    Vector3d& image_point,
    double& w)
{
    ceres::LossFunction *loss_function = NULL;
    ceres::CostFunction *cost_function = new ceres::AutoDiffCostFunction<PointToPointError, 1, 6>(new PointToPointError(lidar_point,image_point, w));
    problem.AddResidualBlock(cost_function, loss_function, params.data());
}

void addStereoMatchingConstraints(
    ceres::Problem& problem,
    VectorXd& params,
    Vector2d& camera1_point,
    Vector2d& camera2_point,
    MatrixXd& camera_matrix,
    double& w)
{
    ceres::LossFunction *loss_function = NULL;
    ceres::CostFunction *cost_function = new ceres::AutoDiffCostFunction<StereoMatchingError, 1, 6>(new StereoMatchingError(camera_matrix,
            camera1_point,camera2_point, w));
    problem.AddResidualBlock(cost_function, loss_function, params.data());
}

int main()
{
    matlab_data1 m1("left");
    matlab_data1 m2("right");

    MatrixXd left_camera_matrix = m1.get_camera_intrinsic_matrix();
    MatrixXd right_camera_matrix = m2.get_camera_intrinsic_matrix();

    MatrixXd left_camera_lidar_tform = m1.get_camera_lidar_tform();
    MatrixXd right_camera_lidar_tform = m2.get_camera_lidar_tform();

    VectorXd Rt_l1_c1, Rt_l1_c2, Rt_c1_c2;
    matrix2Vector(left_camera_lidar_tform, Rt_l1_c1);
    matrix2Vector(right_camera_lidar_tform, Rt_l1_c2);
    cout<< "left_camera_lidar_tform: "<<endl;
    cout<< left_camera_lidar_tform<<endl;
    cout<<"right_camera_lidar_tform:  "<<endl;
    cout<< right_camera_lidar_tform<<endl;
    Rt_c1_c2.resize(6);
    Rt_c1_c2<< 0,0,0,-0.12,0,0;
    ceres::Problem problem;

    // 读入3d corners(lidar and camera)
    vector<vector<Vector3d>> left_camera_corners_3d = m1.get_image_corners_3d();
    vector<vector<Vector3d>> right_camera_corners_3d = m2.get_image_corners_3d();
    vector<vector<Vector3d>> lidar_corners_3d = m1.get_lidar_corners_3d();// 是一样的

    int n_frames = left_camera_corners_3d.size();
    int n_corners = left_camera_corners_3d[0].size();

    cout<<"-----------------------------------------"<<endl;
    for(int i=0;i<n_frames;++i){
        for(int j = 0;j<n_corners;++j){
            auto& camera_point = left_camera_corners_3d[i][j];
            auto& lidar_point = lidar_corners_3d[i][j];
            Vector4d p1,p2;
            p1.head(3) = camera_point;
            p1(3) = 1;
            p2.head(3) = lidar_point;
            p2(3) = 1;
            Vector4d p3 = p1 - left_camera_lidar_tform*p2;
            cout<< p1(0)<<" "<<p1(1)<<" "<<p1(2)<<endl<<
                p2(0)<<" "<<p2(1)<<" "<<p2(2)<<endl<<
                p3(0)<<" "<<p3(1)<<" "<<p3(2)<<endl;
            cout<<endl;
        }
    }

    cout<<"-----------------------------------------"<<endl;
    for(int i=0;i<n_frames;++i){
        for(int j = 0;j<n_corners;++j){
            auto& camera_point = right_camera_corners_3d[i][j];
            auto& lidar_point = lidar_corners_3d[i][j];
            Vector4d p1,p2;
            p1.head(3) = camera_point;
            p1(3) = 1;
            p2.head(3) = lidar_point;
            p2(3) = 1;
            Vector4d p3 = p1 - right_camera_lidar_tform*p2;
            cout<< p1(0)<<" "<<p1(1)<<" "<<p1(2)<<endl<<
                p2(0)<<" "<<p2(1)<<" "<<p2(2)<<endl<<
                p3(0)<<" "<<p3(1)<<" "<<p3(2)<<endl;
            cout<<endl;
        }
    }

    exit(0);
    double w1 = 1/ sqrt((double)n_frames*(double)n_corners);
    for(int i=0;i<n_frames;++i){
        for(int j = 0;j<n_corners;++j){
            auto& camera_point = left_camera_corners_3d[i][j];
            auto& lidar_point = lidar_corners_3d[i][j];
            addPointCorrespondenceConstraints(problem, Rt_l1_c1,lidar_point, camera_point,w1);
        }
    }

    for(int i=0;i<n_frames;++i){
        for(int j = 0;j<n_corners;++j){
            auto& camera_point = right_camera_corners_3d[i][j];
            auto& lidar_point = lidar_corners_3d[i][j];
            addPointCorrespondenceConstraints(problem, Rt_l1_c2,lidar_point, camera_point,w1);
        }
    }

    // 读入图像角点
    // vector<vector<Vector2d>> left_camera_points_2d = m1.get_image_points();
    // vector<vector<Vector2d>> right_camera_points_2d = m2.get_image_points();
    // int n_image_point_2d = left_camera_points_2d[0].size();
    // double w2 = 1/sqrt((double)n_frames*(double)n_image_point_2d);
    // for(int i=0;i<n_frames;++i){
    //     for(int j = 0;j<n_image_point_2d;++j){
    //         auto& left_camera_point_2d = left_camera_points_2d[i][j];
    //         auto& right_camera_point_2d = right_camera_points_2d[i][j];
    //         addStereoMatchingConstraints(problem, Rt_c1_c2,left_camera_point_2d, right_camera_point_2d,left_camera_matrix,w2);
    //     }
    // }

    ceres::Solver::Options options;
    options.max_num_iterations = 500;
    options.linear_solver_type = ceres::SPARSE_NORMAL_CHOLESKY;
    options.minimizer_progress_to_stdout = false;
    ceres::Solver::Summary summary;
    ceres::Solve(options, &problem, &summary);
    std::cout << summary.BriefReport() << "\n";

    MatrixXd refined_Rt_l1_c1, refined_Rt_l1_c2;
    vector2Matrix(Rt_l1_c1, refined_Rt_l1_c1);
    vector2Matrix(Rt_l1_c2, refined_Rt_l1_c2);
    cout << refined_Rt_l1_c1 << std::endl;
    cout << refined_Rt_l1_c2 << std::endl;
    cout << Rt_c1_c2 << std::endl;

    cout<<"-----------------------------------------"<<endl;
    for(int i=0;i<n_frames;++i){
        for(int j = 0;j<n_corners;++j){
            auto& camera_point = left_camera_corners_3d[i][j];
            auto& lidar_point = lidar_corners_3d[i][j];
            Vector4d p1,p2;
            p1.head(3) = camera_point;
            p1(3) = 1;
            p2.head(3) = lidar_point;
            p2(3) = 1;
            Vector4d p3 = p1 - refined_Rt_l1_c1*p2;
            cout<< p1(0)<<" "<<p1(1)<<" "<<p1(2)<<endl<<
                p2(0)<<" "<<p2(1)<<" "<<p2(2)<<endl<<
                p3(0)<<" "<<p3(1)<<" "<<p3(2)<<endl;
            cout<<endl;
        }
    }

    cout<<"-----------------------------------------"<<endl;
    for(int i=0;i<n_frames;++i){
        for(int j = 0;j<n_corners;++j){
            auto& camera_point = right_camera_corners_3d[i][j];
            auto& lidar_point = lidar_corners_3d[i][j];
            Vector4d p1,p2;
            p1.head(3) = camera_point;
            p1(3) = 1;
            p2.head(3) = lidar_point;
            p2(3) = 1;
            Vector4d p3 = p1 - refined_Rt_l1_c2*p2;
            cout<< p1(0)<<" "<<p1(1)<<" "<<p1(2)<<endl<<
                p2(0)<<" "<<p2(1)<<" "<<p2(2)<<endl<<
                p3(0)<<" "<<p3(1)<<" "<<p3(2)<<endl;
            cout<<endl;
        }
    }


    return 0;
}