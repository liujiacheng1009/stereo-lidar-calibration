#include "extractImageFeature.hpp"


int main()
{
    std::string image_path = "/home/jc/Documents/stereo-lidar-calibration/exclude_dir/lcc/HDL64/images/0002.png";
    cv::Mat img = cv::imread(image_path, cv::IMREAD_COLOR);
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
    // std::cout<<image_feature_detector.get_camera_matrix()<<std::endl;
    // std::cout<< image_feature_detector.get_dist_coeffs() <<std::endl;
    std::vector<cv::Point2f> image_corners;
    if(!image_feature_detector.detectImageCorner(img, image_corners))
        return -1;
    cv::Mat rvec, tvec;
    image_feature_detector.estimatePose(image_corners, rvec, tvec);
    // std::cout<< rvec <<std::endl;
    // std::cout<< tvec <<std::endl;
    
	// cv::imshow("chessboard corners", img);
	// cv::waitKey(0);

    return 0;
}