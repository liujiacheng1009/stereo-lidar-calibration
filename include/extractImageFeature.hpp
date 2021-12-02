#ifndef STEREO_LIDAR_CALIBRATION_EXTRACTIMAGEFEATURE_HPP
#define STEREO_LIDAR_CALIBRATION_EXTRACTIMAGEFEATURE_HPP


#include <iostream>
#include <opencv2/core.hpp>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>

#include <memory>   /// std::shared_ptr
#include <vector>
#include <fstream>
#include <string>

using namespace std;

class ImageFeatureDetector{
public:
    ImageFeatureDetector(cv::Mat& camera_matrix, cv::Mat& dist_coeffs, double& square_size ):
        m_camera_matrix(camera_matrix), m_dist_coeffs(dist_coeffs), m_square_size(square_size){}

    void detectImageCorner(cv::Mat& input_image, vector<cv::Point2d>& image_corners); // 从图像检测角点
    void undistort_corners(vector<cv::Point2f>& input_corners, cv::Mat &rectified_corners); // 角点畸变校正
    void estimate_RT(vector<cv::Point2d>& chessboard_corners, cv::Mat& rvec, cv::Mat&tvec); // 从角点pnp恢复位姿

    const cv::Mat get_camera_matrix(){
        return m_camera_matrix;
    }

    const cv::Mat get_dist_coeffs(){
        return m_dist_coeffs;
    }
    
private:
    void calcBoardCornerPositions(); // 计算board corners 在世界坐标系的坐标
    cv::Size m_board_size;  // chessboard 规格大小
    double m_square_size; // 格子边长(m)
    cv::Mat m_camera_matrix, m_dist_coeffs; // 相机的内参和畸变矩阵
    std::vector<cv::Point3f> m_3d_corners; // 世界坐标系下的角点坐标
};

#endif
