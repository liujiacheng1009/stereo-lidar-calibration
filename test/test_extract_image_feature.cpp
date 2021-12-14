#include "extractImageFeature.hpp"
#include "utils.hpp"


void runs1()
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

    std::vector<std::string> images;
    get_data_by_path(images, images_dir, ".png");
    sort(images.begin(), images.end());

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
        // std::cout<<"tvec: " <<tvec<<std::endl;
        std::vector<cv::Point3d> chessboard_3d_corners, reordered_image_3d_corners;
        image_feature_detector.calculate3DCorners(chessboard_3d_corners, rvec, tvec);
        // reorder_corners(chessboard_3d_corners, reordered_image_3d_corners);
        image_3d_corners.push_back(chessboard_3d_corners);
        valid_image_index.push_back(i);
    }

    for (int i = 0; i < image_3d_corners.size(); i++)
    {
        // cv::Mat image = cv::imread(images[valid_image_index[i]], cv::IMREAD_COLOR);
        std::vector<cv::Point3d> &corners = image_3d_corners[i];
        for (auto &corner : corners)
        {
            // cv::Point2d p = project(corner, camera_matrix);
            // cv::circle(image, p, 5, cv::Scalar(0, 255, 0), -1);
            std::cout<<corner<<std::endl;
        }
        std::cout<<std::endl;
        // cv::imwrite("/home/jc/Documents/stereo-lidar-calibration/exclude_dir/test_data/" 
        //     + to_string(i) + ".png", image);
    }
    return;
}

void runs2(){
    std::vector<cv::Point3d> corners = {
        cv::Point3d(-0.7703, -0.5006, 3.1952),
        cv::Point3d(0.8134, -0.6382, 3.0134),
        cv::Point3d(0.9016, 0.3578, 3.0281),
        cv::Point3d(-0.6820, 0.4954, 3.2099)
    };
    cv::Mat camera_matrix = cv::Mat::eye(3, 3, CV_64F);
    camera_matrix.at<double>(0,0) = 1.614546232069338e+03;
    camera_matrix.at<double>(0,2) = 6.412276358621397e+02;
    camera_matrix.at<double>(1,1) = 1.614669013419422e+03;
    camera_matrix.at<double>(1,2) = 4.801410561665820e+02;
    cv::Mat dist_coeffs = cv::Mat::zeros(5, 1, CV_64F);
    dist_coeffs.at<double>(0,0) = -0.004497294509341;
    dist_coeffs.at<double>(1,0) = 0.020426051162860;
    double square_size = 0.2;

    ImageFeatureDetector image_feature_detector(camera_matrix, dist_coeffs, square_size);
    std::vector<Eigen::VectorXd> lines;
    Eigen::VectorXd plane;
    image_feature_detector.calculateLines(corners, lines);
    std::cout<<lines.size()<<std::endl;
    for(auto& line:lines){
        std::cout<<line<<std::endl;
    }
    image_feature_detector.calculatePlane(corners, plane);
    std::cout<<plane<<std::endl;
    return;
}


int main()
{
    runs2();
    // std::string image_path = "/home/jc/Documents/stereo-lidar-calibration/exclude_dir/lcc/HDL64/images/0002.png";
    // cv::Mat img = cv::imread(image_path, cv::IMREAD_COLOR);
    // cv::Mat camera_matrix = cv::Mat::eye(3, 3, CV_64F);
    // camera_matrix.at<double>(0,0) = 1.614546232069338e+03;
    // camera_matrix.at<double>(0,2) = 6.412276358621397e+02;
    // camera_matrix.at<double>(1,1) = 1.614669013419422e+03;
    // camera_matrix.at<double>(1,2) = 4.801410561665820e+02;
    // cv::Mat dist_coeffs = cv::Mat::zeros(5, 1, CV_64F);
    // dist_coeffs.at<double>(0,0) = -0.004497294509341;
    // dist_coeffs.at<double>(1,0) = 0.020426051162860;
    // double square_size = 0.2;
    // ImageFeatureDetector image_feature_detector(camera_matrix, dist_coeffs, square_size);
    // // std::cout<<image_feature_detector.get_camera_matrix()<<std::endl;
    // // std::cout<< image_feature_detector.get_dist_coeffs() <<std::endl;
    // std::vector<cv::Point2f> image_corners;
    // if(!image_feature_detector.detectImageCorner(img, image_corners))
    //     return -1;
    // for(auto& corner:image_corners){
    //     std::cout<<corner<<std::endl;
    // }
    // cv::Mat rvec, tvec;
    // image_feature_detector.estimatePose(image_corners, rvec, tvec);
    // std::vector<cv::Point3d> chessboard_3d_corners;
    // image_feature_detector.calculate3DCorners(chessboard_3d_corners, rvec, tvec);
 
    // for(auto& corner:chessboard_3d_corners){
    //     std::cout<< corner<<std::endl;
    //     cv::Point2d p = project(corner,camera_matrix);
    //     cv::circle(img, p, 5, cv::Scalar(0, 255, 0), -1);
    // }
    // cv::Point3d ori_p(tvec);
    // cv::Point2d p = project(ori_p,camera_matrix);
    // cv::circle(img, p, 5, cv::Scalar(0, 255, 0), -1);
    // std::cout<< rvec <<std::endl;
    // std::cout<< tvec <<std::endl;
    
	// cv::imshow("chessboard corners", img);
	// cv::waitKey(0);

    return 0;
}