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
    cout<< "有效的图像帧有： "<<endl;
    for(auto& id:images_features.valid_index){
        cout<< id<<" ";
    }
    cout<<endl;

    CloudResults cloud_features;
    processCloud(config, clouds, cloud_features );
    cout<< "有效的点云帧有： "<<endl;
    for(auto& id:cloud_features.valid_index){
        cout<< id<<" ";
    }
    cout<<endl;

    getValidDataSet(images_features, cloud_features);
    auto& image_3d_corners = images_features.corners_3d; // 图像3d角点
    auto& image_planes_3d = images_features.planes_3d;
    auto& cloud_3d_corners = cloud_features.corners_3d; // 点云3d角点
    auto& plane_clouds_3d  = cloud_features.plane_points_3d;

    // cout<< images_features.valid_index.size()<<endl;
    // cout<< cloud_features.valid_index.size()<<endl;
    // cout<< image_3d_corners.size()<<endl;

    // debug
    // cout<<"----------------------------"<<endl;
    // for(int i = 0;i<cloud_3d_corners.size();++i){
    //     vector<PointCloud<PointXYZ>> cloud;
    //     PointCloud<PointXYZ> corners;
    //     for(auto& corner:image_3d_corners[i]){
    //         cout<< corner.transpose()<<endl;
    //         corners.push_back(PointXYZ(corner(0), corner(1), corner(2)));
    //     }
    //     cloud.push_back(plane_clouds_3d[i]);
    //     cloud.push_back(corners);
    //     // display_multi_clouds(cloud);
    //     cout<<endl;
    // }
    // cout<<"----------------------------"<<endl;


    Eigen::VectorXd R_t(6);
    calculateInitialRt(cloud_3d_corners, image_3d_corners, R_t);
    cout<< "initial Rt \n"<< R_t<<endl; 
    //R_t << 0., 0., 0., 0., 0., 0.;
    Optimizer optimizer_lc;
    ceres::Problem problem;
    std::vector<Vector3d> p1, p2;
    for(int i=0;i<image_3d_corners.size();++i){
        auto& image_corners = image_3d_corners[i];
        auto& cloud_corners = cloud_3d_corners[i];
        optimizer_lc.addPointToPointConstriants(problem,cloud_corners,image_corners,R_t);
    }

    assert(image_planes_3d.size()==plane_clouds_3d.size());
    cout<< "use " << image_planes_3d.size() << " planes"<<endl;
    for(int i=0;i<plane_clouds_3d.size();++i){
        auto& image_plane = image_planes_3d[i];
        auto& plane_cloud = plane_clouds_3d[i];
        // cout<< image_plane <<endl;
        // cout<< plane_cloud.size() <<endl;
        // display_colored_by_depth(plane_cloud.makeShared());
        optimizer_lc.addPointToPlaneConstriants(problem,plane_cloud, image_plane,R_t );
    }

    ceres::Solver::Options options;
    options.max_num_iterations = 500;
    options.linear_solver_type = ceres::SPARSE_NORMAL_CHOLESKY;
    options.minimizer_progress_to_stdout = false;
    ceres::Solver::Summary summary;
    ceres::Solve(options, &problem, &summary);
    std::cout << summary.BriefReport() << "\n";
    MatrixXd Rt = Matrix<double,4,4>::Identity();
    cout<< "Rt \n"<< R_t<<endl; 
    Eigen::Matrix3d R = Sophus::SO3d::exp(R_t.head(3)).matrix();
    Rt.block(0,0,3,3) = R;
    Rt.block(0,3,3,1) = R_t.tail(3);
    cout << Rt << std::endl;
    cout << Rt-config.matlab_tform<<endl;
    return 0;
}