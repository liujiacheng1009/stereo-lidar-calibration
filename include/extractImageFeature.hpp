#ifndef STEREO_LIDAR_CALIBRATION_EXTRACTIMAGEFEATURE_HPP
#define STEREO_LIDAR_CALIBRATION_EXTRACTIMAGEFEATURE_HPP
#include <opencv2/core.hpp>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/core/utility.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/videoio.hpp>
#include "config.hpp"
#include "libcbdetect/config.h"
#include "libcbdetect/find_corners.h"
#include "libcbdetect/boards_from_corners.h"
// #include "utils.hpp"
#include <memory>   
#include <vector>
#include <fstream>
#include <string>
#include <sstream>
#include <ctime>
#include <cstdio>
#include <iostream>

using namespace std;
using namespace Eigen;
using namespace cv;
// using namespace pcl;

bool fitBoard(vector<vector<int>> &board, vector<Point2d>& corners, Size &size);

class ImageResults{
public:
    vector<int> valid_index; // 有效的图像index
    vector<vector<Vector3d>> corners_3d; // 图像3d角点
    vector<vector<Vector2d>> chessboard_points_2d;
    vector<vector<Vector3d>> chessboard_points_3d;
    vector<VectorXd> planes_3d; // 图像的平面方程
    vector<vector<VectorXd>> lines_3d; // 图像边缘直线方程
};

void processImage(vector<string>& image_paths, ImageResults& images_features);

class ImageFeatureDetector{
public:
    ImageFeatureDetector() : m_camera_matrix(Config::leftCameraMatrix()),
                                           m_dist_coeffs(Config::leftCameraDistCoeffs()),
                                           m_square_size(Config::checkerboardSquareSize()),
                                           m_board_size(Config::checkerboardGridSize()),
                                           m_padding(Config::checkerboardPadding()) {}

    bool detectImageCorner(cv::Mat& input_image, vector<cv::Point2f>& image_corners); // 从图像检测角点
    bool detectImageCorner1(cv::Mat &input_image, vector<cv::Point2f> &image_corners);
    void undistort_corners(vector<cv::Point2f>& input_corners, cv::Mat &rectified_corners); // 角点畸变校正
    void estimatePose(vector<cv::Point2f>& chessboard_corners,vector<Vector3d>& chessboard_points_3d, cv::Mat& rvec, cv::Mat&tvec); // 从角点pnp恢复位姿
    void estimatePose1(vector<cv::Point2f>& chessboard_corners,vector<Vector3d>& chessboard_points_3d, cv::Mat& rvec, cv::Mat&tvec); // 从角点pnp恢复位姿
    void calculate3DCorners(vector<cv::Point3d>& chessboard_3d_corners, cv::Mat& rvec, cv::Mat& tvec); // 计算marker板的3D角点位置，需考虑偏移
    void calculateLines(std::vector<cv::Point3d>& corners, std::vector<Eigen::VectorXd>& lines); // 通过四个角点计算四条边的方程
    void calculatePlane(std::vector<cv::Point3d>& corners, Eigen::VectorXd& plane); // 通过四个角点计算平面的方程
    void calculatePlane1(std::vector<cv::Point3d>& corners, Eigen::VectorXd& plane); // 通过四个角点计算平面的方程
    void transform_to_normalized_plane(vector<Point2f>& corners1, vector<Vector2d>& corners2);
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
    cv::Mat m_camera_matrix = Mat(3, 3, CV_64F);
    cv::Mat m_dist_coeffs = Mat(5,1, CV_64F); // 相机的内参和畸变矩阵
    //std::vector<cv::Point3f> m_3d_corners; // 世界坐标系下的角点坐标
    std::vector<double> m_padding;
};





#endif
