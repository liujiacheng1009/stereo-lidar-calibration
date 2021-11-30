#include <iostream>
#include <opencv2/core.hpp>
#include <Eigen/Core>
#include <Eigen/Geometry>

#include <memory>   /// std::shared_ptr
#include <vector>
#include <fstream>

using namespace std;

class ImageFeatureDetector{
public:
    void detectImageCorner(cv::Mat& input_image, vector<cv::Point2d>& image_corners); // 从图像检测角点
    void undistort_corners(vector<cv::Point2f>& input_corners, cv::Mat &rectified_corners); // 角点畸变校正
    void estimate_RT(vector<cv::Point2d>& chessboard_corners, cv::Mat& rvec, cv::Mat&tvec); // 从角点pnp恢复位姿

    
private:
    void calcBoardCornerPositions(); // 计算board corners 在世界坐标系的坐标
    cv::Size m_board_size;  // chessboard 规格大小
    double m_square_size; // 格子边长(m)
    cv::Mat m_camera_matrix, m_dist_coeffs; // 相机的内参和畸变矩阵
    std::vector<cv::Point3f> m_3d_corners; // 世界坐标系下的角点坐标
};