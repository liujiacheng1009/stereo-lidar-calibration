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



class ImageFeatureDetector{
public:
    template<typename T>
    ImageFeatureDetector(T &config) : m_camera_matrix(config.left_camera_matrix),
                                           m_dist_coeffs(config.left_camera_dist_coeffs),
                                           m_square_size(config.checkerboard_square_size),
                                           m_board_size(config.checkerboard_grid_size),
                                           m_padding(config.checkerboard_padding) {}

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




template<typename T>
void processImage(T& config, vector<string>& image_paths, ImageResults& images_features)
{
    ImageFeatureDetector image_feature_detector(config);
    auto& valid_image_index = images_features.valid_index;
    auto& image_3d_corners = images_features.corners_3d; // 图像3d角点
    auto& chessboard_2d_points = images_features.chessboard_points_2d;
    auto& chessboard_3d_points = images_features.chessboard_points_3d;
    auto& image_planes = images_features.planes_3d; // 图像的平面方程
    auto& image_lines = images_features.lines_3d;

    for (int i = 0;i<image_paths.size();++i)
    {
        cv::Mat img = cv::imread(image_paths[i], cv::IMREAD_COLOR);
        // cv::Mat half_image;
        // double s1 = (double)640 / (double)img.size().width;
        // double s2 = (double)480 / (double)img.size().height;
        // cv::resize(img, half_image, cv::Size(), s1, s2); // cv::findChessboardCorners在高分辨率图像上有bug, 放缩dao

        std::vector<cv::Point2f> image_corners;
        // if (!image_feature_detector.detectImageCorner(half_image, image_corners)){
        //     // std::cout<< "can not detect corner from image: " << i<<std::endl;
        //     continue;
        // }
        // for(auto& image_corner:image_corners){
        //     image_corner.x *= (1.0/s1);
        //     image_corner.y *= (1.0/s2);
        // }

        if (!image_feature_detector.detectImageCorner1(img, image_corners)) continue;
        cv::Mat rvec, tvec;
        vector<Vector3d> chessboard_points_3d;
        image_feature_detector.estimatePose1(image_corners,chessboard_points_3d, rvec, tvec);
        vector<Vector2d> chessboard_points_2d;
        image_feature_detector.transform_to_normalized_plane(image_corners, chessboard_points_2d);
        std::vector<Point3d> chessboard_3d_corners, reordered_image_3d_corners;
        image_feature_detector.calculate3DCorners(chessboard_3d_corners, rvec, tvec);
        
        Eigen::VectorXd plane;
        image_feature_detector.calculatePlane1(chessboard_3d_corners, plane);
        vector<Eigen::VectorXd> lines;
        image_feature_detector.calculateLines(chessboard_3d_corners, lines);
        image_lines.push_back(lines);
        image_planes.push_back(plane);
        vector<Vector3d> corners_3d; //
        for(auto& corners: chessboard_3d_corners){
            corners_3d.push_back(Vector3d(corners.x, corners.y, corners.z));
        }
        // vector<Vector2d> chessboard_points_2d;
        // image_feature_detector.transform_to_normalized_plane(image_corners, co)
        // for(auto& corner: image_corners){
        //     chessboard_points_2d.push_back(Vector2d(corner.x, corner.y));
        // }
        chessboard_3d_points.push_back(chessboard_points_3d);
        chessboard_2d_points.push_back(chessboard_points_2d);
        image_3d_corners.push_back(corners_3d);
        valid_image_index.push_back(i);
    }
    return;
}

#endif
