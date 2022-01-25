#include "extractChessboard.hpp"
#include "extractImageFeature.hpp"
#include "extractLidarFeature.hpp"
#include <pcl/common/geometry.h>
#include <sophus/so3.hpp>
#include <boost/program_options.hpp>
#include <opencv2/core/eigen.hpp>
#include "optimization.hpp"
#include "utils.hpp"
#include "config.hpp"
#include "evaluation.hpp"
using namespace std;
using namespace Eigen;

int main(int argc, char** argv)
{
    string config_file;
    boost::program_options::options_description desc("Allowed options");
    desc.add_options()
        ("help", "produce help message")
        ("config,c", boost::program_options::value<std::string>(&config_file)->default_value("config.yaml"), "config file.");
    boost::program_options::variables_map vm;
    boost::program_options::store(boost::program_options::parse_command_line(argc, argv, desc), vm);
    boost::program_options::notify(vm);
    if (vm.count("help"))
    {
        std::cout << desc << std::endl;
        return 1;
    }
    Config::loadFromFile(config_file); // 
    if(Config::calibType()!= "stereo_lidar"){
        std::cout<< "配置文件应是stereo_lidar类型 ！！"<<std::endl;
        exit(-1);
    }
    std::string left_images_dir = Config::leftImageDatasetPath();
    std::string right_images_dir = Config::rightImageDatasetPath();
    std::string clouds_dir = Config::lidarCloudDatasetPath();
    std::vector<std::string> left_images, right_images;
    get_data_by_path(left_images, left_images_dir, Config::imageFormat());
    get_data_by_path(right_images, right_images_dir, Config::imageFormat());
    std::vector<std::string> clouds;
    get_data_by_path(clouds, clouds_dir, Config::cloudFormat());
    if(!check_data_by_index(left_images, clouds)){
        cerr <<"图像和点云文件应存在且文件名相同！！"<<endl;
        exit(-1);
    }
    if(!check_data_by_index(right_images, clouds)){
        cerr <<"图像和点云文件应存在且文件名相同！！"<<endl;
        exit(-1);
    }

    ImageResults left_images_features, right_images_features;
    processImage(left_images, left_images_features, Config::leftCameraMatrix(), Config::leftCameraDistCoeffs());
    cout<< "有效的左目图像帧有： "<<endl;
    for(auto& id:left_images_features.valid_index){
        cout<< id<<" ";
    }
    cout<<endl;

    processImage(right_images, right_images_features, Config::rightCameraMatrix(), Config::rightCameraDistCoeffs());
    cout<< "有效的右目图像帧有： "<<endl;
    for(auto& id:right_images_features.valid_index){
        cout<< id<<" ";
    }
    cout<<endl;

    CloudResults cloud_features;
    processCloud(clouds, cloud_features);
    cout<< "有效的点云帧有： "<<endl;
    for(auto& id:cloud_features.valid_index){
        cout<< id<<" ";
    }
    cout<<endl;
    getValidDataSet(left_images_features,right_images_features, cloud_features);
    auto& left_image_3d_corners = left_images_features.corners_3d; // 图像3d角点
    auto& left_chessboard_points_2d = left_images_features.chessboard_points_2d;
    auto& left_chessboard_points_3d = left_images_features.chessboard_points_3d;
    auto& right_image_3d_corners = right_images_features.corners_3d;
    auto& right_chessboard_points_2d = right_images_features.chessboard_points_2d;
    auto& right_chessboard_points_3d = right_images_features.chessboard_points_3d;
    auto& cloud_3d_corners = cloud_features.corners_3d; // 点云3d角点
    auto& plane_clouds_3d  = cloud_features.plane_points_3d;

    VectorXd Rt_l1_c1(6), Rt_l1_c2(6), Rt_c1_c2(6);
    calculateInitialRt(cloud_3d_corners, left_image_3d_corners, Rt_l1_c1);
    calculateInitialRt(cloud_3d_corners, right_image_3d_corners, Rt_l1_c2);
    Eigen::Matrix4d mat_l1_c1,mat_l1_c2,mat_c1_c2;
    vector2Matrix(Rt_l1_c1, mat_l1_c1);
    vector2Matrix(Rt_l1_c2, mat_l1_c2);
    mat_c1_c2 =  mat_l1_c2* mat_l1_c1.inverse();
    matrix2Vector(mat_c1_c2, Rt_c1_c2);

    std::cout<< "initial Rt_l1_c1 \n"<< Rt_l1_c1<<std::endl; 
    std::cout<< "initial Rt_l1_c2 \n"<< Rt_l1_c2<<std::endl; 
    std::cout<< "initial Rt_c1_c2 \n"<< Rt_c1_c2<<std::endl; 

    std::cout<< "initial mat_l1_c1 \n"<< mat_l1_c1<<std::endl; 
    std::cout<< "initial mat_l1_c2 \n"<< mat_l1_c2<<std::endl; 
    std::cout<< "initial mat_c1_c2 \n"<< mat_c1_c2<<std::endl; 

    // debug 
    // {
    //     for(auto& corners: left_image_3d_corners){
    //         for(auto& corner: corners)
    //         std::cout<< corner.transpose()<<std::endl;
    //     }
    //     exit(17);
    // }
    // debug
    {
        std::cout<<"close up : \n"<< Config::tformL1C1()*Config::tformL1C2().inverse()*Config::tformC1C2() << std::endl;
    }

    // debug
    {
        Rt_c1_c2<<0,0,0,0,0,0;
    }
    
    Optimizer optimizer_stereo;
    ceres::Problem problem_stereo;
    int n_samples = left_image_3d_corners.size();
    for(int i=0;i<n_samples;i++){
        auto& P1 = left_chessboard_points_3d[i];
        auto& p1  = left_chessboard_points_2d[i];
        auto& P2 = right_chessboard_points_3d[i];
        auto& p2  = right_chessboard_points_2d[i];
        optimizer_stereo.addStereoMatchingConstraints(problem_stereo, P1,p1,P2,p2,Rt_c1_c2);
    }
    ceres::Solver::Options options_stereo;
    options_stereo.max_num_iterations = 100;
    // options.linear_solver_type = ceres::SPARSE_NORMAL_CHOLESKY;
    // options.minimizer_progress_to_stdout = false;
    ceres::Solver::Summary summary_stereo;
    ceres::Solve(options_stereo, &problem_stereo, &summary_stereo);
    std::cout << summary_stereo.BriefReport() << "\n";
    // debug
    {
        vector2Matrix(Rt_c1_c2, mat_c1_c2);
        std::cout<<"mat_c1_c2: \n"<< mat_c1_c2<< std::endl;
    }

    // debug
    // {
    //     Rt_c1_c2<<0,0,0,-0.77,0,0;
    // }

    Optimizer optimizer;
    ceres::Problem problem;

    for(int i=0;i<n_samples;++i){
        auto& image_corners = left_image_3d_corners[i];
        auto& cloud_corners = cloud_3d_corners[i];
        optimizer.addPointToPointConstriants(problem,cloud_corners,image_corners,Rt_l1_c1);
    }

    for(int i=0;i<n_samples;++i){
        auto& image_corners = right_image_3d_corners[i];
        auto& cloud_corners = cloud_3d_corners[i];
        optimizer.addPointToPointConstriants(problem,cloud_corners,image_corners,Rt_l1_c2);
    }

    for(int i=0;i<n_samples;i++){
        auto& P1 = left_chessboard_points_3d[i];
        auto& p1  = left_chessboard_points_2d[i];
        auto& P2 = right_chessboard_points_3d[i];
        auto& p2  = right_chessboard_points_2d[i];
        optimizer.addStereoMatchingConstraints(problem, P1,p1,P2,p2,Rt_c1_c2);
    }

    for(int i = 0;i<n_samples;++i){
        optimizer.addClosedLoopConstraints(problem,Rt_l1_c1, Rt_l1_c2, Rt_c1_c2);
    }

    ceres::Solver::Options options;
    options.max_num_iterations = 500;
    options.linear_solver_type = ceres::SPARSE_NORMAL_CHOLESKY;
    options.minimizer_progress_to_stdout = false;
    ceres::Solver::Summary summary;
    ceres::Solve(options, &problem, &summary);
    std::cout << summary.BriefReport() << "\n";

    std::cout<< "optimized Rt_l1_c1 \n"<< Rt_l1_c1<<std::endl; 
    std::cout<< "optimized Rt_l1_c2 \n"<< Rt_l1_c2<<std::endl; 
    std::cout<< "optimized Rt_c1_c2 \n"<< Rt_c1_c2<<std::endl; 


    std::cout<<"----------------------------"<<std::endl;
    mat_l1_c1.block(0,0,3,3) = Sophus::SO3d::exp(Rt_l1_c1.head(3)).matrix();
    mat_l1_c1.block(0,3,3,1) = Rt_l1_c1.tail(3);
    std::cout << mat_l1_c1 << std::endl;
    std::cout << Config::tformL1C1() << std::endl;
    std::cout << mat_l1_c1-Config::tformL1C1()<<endl;
    std::cout<<"----------------------------"<<std::endl;

    std::cout<<"----------------------------"<<std::endl;
    mat_l1_c2.block(0,0,3,3) = Sophus::SO3d::exp(Rt_l1_c2.head(3)).matrix();
    mat_l1_c2.block(0,3,3,1) = Rt_l1_c2.tail(3);
    std::cout << mat_l1_c2 << std::endl;
    std::cout << Config::tformL1C2() << std::endl;
    std::cout << mat_l1_c2-Config::tformL1C2()<<endl;
    std::cout<<"----------------------------"<<std::endl;

    std::cout<<"----------------------------"<<std::endl;
    mat_c1_c2.block(0,0,3,3) = Sophus::SO3d::exp(Rt_c1_c2.head(3)).matrix();
    mat_c1_c2.block(0,3,3,1) = Rt_c1_c2.tail(3);
    std::cout << mat_c1_c2 << std::endl;
    std::cout << Config::tformC1C2()<<endl;
    std::cout << mat_c1_c2-Config::tformC1C2()<<endl;
    std::cout<<"----------------------------"<<std::endl;

    // debug 
    {
        std::cout<<"close up : \n"<< mat_l1_c1*mat_l1_c2.inverse()*mat_c1_c2 << std::endl;
    }

    bool eval = false;
    if(eval){
        // debug
        // {
        //     auto mat = Config::tformL1C1();
        //     Eigen::Matrix3d mat_R = mat.block(0,0,3,3);
        //     Eigen::AngleAxisd aa;
        //     aa.fromRotationMatrix(mat_R);
        //     Rt_l1_c1.head(3) = aa.angle() * aa.axis();
        //     Rt_l1_c1.tail(3) = mat.block(0,3,3,1);
        // }

        cv::Mat rvec = cv::Mat::zeros(cv::Size(1,3), CV_64FC1);
        rvec.at<double>(0,0) = Rt_l1_c1(0);
        rvec.at<double>(1,0) = Rt_l1_c1(1);
        rvec.at<double>(2,0) = Rt_l1_c1(2);
        cv::Mat tvec = cv::Mat::zeros(cv::Size(1,3), CV_64FC1);
        tvec.at<double>(0,0) = Rt_l1_c1(3);
        tvec.at<double>(1,0) = Rt_l1_c1(4);
        tvec.at<double>(2,0) = Rt_l1_c1(5);

        auto& valid_index = cloud_features.valid_index;

        for(int i = 0;i<valid_index.size();++i)
        {
            int k = valid_index[i];
            cv::Mat img = cv::imread(left_images[k], cv::IMREAD_COLOR);
            
            // display on images
            // projectLidarOnImage(img, plane_clouds_3d[i] , rvec, tvec, Config::leftCameraMatrix(), Config::leftCameraDistCoeffs());
            // cv::imshow("eval", img);
            // cv::waitKey(0);

            // display on cloud 
            pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
            if (!loadPointXYZ(clouds[k], cloud)) continue;
            // debug 
            // {
            //     std::vector<pcl::PointCloud<pcl::PointXYZ>> show_cloud;
            //     show_cloud.push_back(*cloud);
            //     show_cloud.push_back(plane_clouds_3d[i]);
            //     display_multi_clouds(show_cloud);
            // }

            auto color_pcd = colorizeCloud(img, *cloud , rvec, tvec, Config::leftCameraMatrix(), Config::leftCameraDistCoeffs());
            boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
            pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(color_pcd);
            viewer->addPointCloud<pcl::PointXYZRGB> (color_pcd, "sample cloud");
            viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "sample cloud"); // 设置点云大小
            viewer->addCoordinateSystem (1.0);
            viewer->initCameraParameters ();
            while (!viewer->wasStopped())
            {
                viewer->spinOnce(100);
                sleep(0.1);
            }
        }
    }

    bool compute_error = true;
    if(compute_error){
        auto translation_error = computeTranslationError(left_image_3d_corners, cloud_3d_corners,mat_l1_c1 );
        std::cout<< " translation_error:\n ";
        for(auto& error:translation_error){
            std::cout<< error <<" ";
        }
        std::cout<<std::endl;
        auto rotation_error = computeRotationError(left_image_3d_corners, cloud_3d_corners,mat_l1_c1 );
        std::cout<< " rotation_error:\n ";
        for(auto& error:rotation_error){
            std::cout<< error <<" ";
        }
        std::cout<<std::endl;
        auto reprojection_error = computeReprojectionError(left_image_3d_corners, cloud_3d_corners,mat_l1_c1 );
        std::cout<< " reprojection_error:\n ";
        for(auto& error:reprojection_error){
            std::cout<< error <<" ";
        }
        std::cout<<std::endl;
    }
    
    return 0;
}