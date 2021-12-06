#include "extractChessboard.hpp"
#include "extractImageFeature.hpp"
#include "extractLidarFeature.hpp"
#include "optimization.hpp"
#include "utils.hpp"





int main()
{
    cv::Mat camera_matrix = cv::Mat::eye(3, 3, CV_64F);
    camera_matrix.at<double>(0,0) = 1.614546232069338e+03;
    camera_matrix.at<double>(0,2) = 6.412276358621397e+02;
    camera_matrix.at<double>(1,1) = 1.614669013419422e+03;
    camera_matrix.at<double>(1,2) = 4.801410561665820e+02;
    cv::Mat dist_coeffs = cv::Mat::zeros(5, 1, CV_64F);
    dist_coeffs.at<double>(0,0) = -0.004497294509341;
    dist_coeffs.at<double>(1,0) = 0.020426051162860;
    double square_size = 0.2;

    std::vector<std::string> images_paths, lidar_clouds_paths;
    std::string dataset_path = "/home/jc/Documents/stereo-lidar-calibration/exclude_dir/lcc/HDL64";
    std::string images_dir = dataset_path + "/images";
    std::string lidar_clouds_dir = dataset_path + "/pointCloud";
    std::vector<std::string> images;
    get_data_by_path(images, images_dir, ".png");
    sort(images.begin(), images.end());
    std::vector<std::string> lidar_clouds;
    get_data_by_path(lidar_clouds, lidar_clouds_dir, ".pcd");
    sort(lidar_clouds.begin(), lidar_clouds.end());
    if(!check_data_by_index(images, lidar_clouds)) return -1;
    // for(auto& image:images){
    //     std::cout<< image <<std::endl;
    // }
    // for(auto& lidar_cloud:lidar_clouds){
    //     std::cout<< lidar_cloud <<std::endl;
    // }
    ImageFeatureDetector image_feature_detector(camera_matrix, dist_coeffs, square_size);
    std::vector<int> valid_image_index;
    std::vector<std::vector<cv::Point3d>> image_3d_corners;
    for (int i = 0;i<images.size();++i)
    {
        cv::Mat img = cv::imread(images[i], cv::IMREAD_COLOR);
        cv::Mat half_image;
        //cv::resize(img, img , cv::Size(0.5, 0.5), cv::INTER_LINEAR);
        cv::resize(img, half_image, cv::Size(), 0.45, 0.45); // cv::findChessboardCorners在高分辨率图像上有bug
        std::vector<cv::Point2f> image_corners;
        if (!image_feature_detector.detectImageCorner(half_image, image_corners)){
            std::cout<< "can not detect corner from image: " << i<<std::endl;
            continue;
        }
        for(auto& image_corner:image_corners){
            image_corner.x *= (1.0/0.45);
            image_corner.y *= (1.0/0.45);
        }
        cv::Mat rvec, tvec;
        image_feature_detector.estimatePose(image_corners, rvec, tvec);
        std::cout<<"tvec: " <<tvec<<std::endl;
        std::vector<cv::Point3d> chessboard_3d_corners;
        image_feature_detector.calculate3DCorners(chessboard_3d_corners, rvec, tvec);
        image_3d_corners.push_back(chessboard_3d_corners);
        valid_image_index.push_back(i);
    }
    std::cout<<image_3d_corners.size()<<std::endl;
    std::cout<<valid_image_index.size()<<std::endl;
    return 0;
}