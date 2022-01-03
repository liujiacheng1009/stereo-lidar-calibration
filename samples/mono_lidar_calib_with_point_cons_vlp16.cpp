#include "extractChessboard.hpp"
#include "extractImageFeature.hpp"
#include "extractLidarFeature.hpp"
#include "optimization.hpp"
#include "utils.hpp"
#include "config.hpp"
#include <pcl/common/geometry.h>
#include <sophus/so3.hpp>
using namespace std;
using namespace Eigen;
using namespace cv;

int main()
{
    Config_vlp16 config;
    std::vector<std::string> images_paths, lidar_clouds_paths;
    std::string images_dir = config.left_images_dataset_path;
    std::string clouds_dir = config.lidar_clouds_dataset_path;
    std::vector<std::string> images;
    get_data_by_path(images, images_dir, config.image_format);
    std::vector<std::string> clouds;
    get_data_by_path(clouds, clouds_dir, config.cloud_format);
    if(!check_data_by_index(images, clouds)){
        cerr <<"图像和点云文件应存在且文件名相同！！"<<endl;
        exit(-1);
    }
    ImageResults images_features;
    processImage(config, images, images_features);

    CloudResults cloud_features;
    processCloud(config, clouds, cloud_features );

    getValidDataSet(images_features, cloud_features);
    auto& image_3d_corners = images_features.corners_3d; // 图像3d角点
    auto& cloud_3d_corners = cloud_features.corners_3d; // 点云3d角点

    cout<< images_features.valid_index.size()<<endl;
    cout<< cloud_features.valid_index.size()<<endl;
    cout<< image_3d_corners.size()<<endl;

    Eigen::VectorXd R_t(6);
    R_t << 0., 0., 0., 0., 0., 0.;
    OptimizationLC optimizer_lc(R_t);
    ceres::Problem problem;
    std::vector<Vector3d> p1, p2;
    for(int i=0;i<image_3d_corners.size();++i){
        auto& image_corners = image_3d_corners[i];
        auto& cloud_corners = cloud_3d_corners[i];
        optimizer_lc.addPointToPointConstriants(problem,cloud_corners,image_corners);
    }
    ceres::Solver::Options options;
    options.max_num_iterations = 500;
    options.linear_solver_type = ceres::SPARSE_NORMAL_CHOLESKY;
    options.minimizer_progress_to_stdout = false;
    ceres::Solver::Summary summary;
    ceres::Solve(options, &problem, &summary);
    std::cout << summary.BriefReport() << "\n";
    MatrixXd Rt = Matrix<double,4,4>::Identity();
    Eigen::Matrix3d R = Sophus::SO3d::exp(optimizer_lc.get_R_t().head(3)).matrix();
    Rt.block(0,0,3,3) = R;
    Rt.block(0,3,3,1) = optimizer_lc.get_R_t().tail(3);
    cout << Rt << std::endl;
    cout << Rt-config.matlab_tform<<endl;
    return 0;
}