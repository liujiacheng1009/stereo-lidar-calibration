#include "extractImageFeature.hpp"

using namespace std;
using namespace Eigen;
using namespace cv;

bool ImageFeatureDetector::detectImageCorner(cv::Mat &input_image, vector<cv::Point2f> &image_corners)
{
    image_corners.clear();
    cv::Mat gray_img;
    cv::cvtColor(input_image, gray_img, cv::COLOR_BGR2GRAY);
    std::vector<cv::Point2f> points;
    const int chessBoardFlags = cv::CALIB_CB_ADAPTIVE_THRESH | cv::CALIB_CB_NORMALIZE_IMAGE;
    bool found = cv::findChessboardCorners(gray_img, m_board_size, points, chessBoardFlags);

    if (found)
    {
        cv::cornerSubPix(gray_img, points, cv::Size(11, 11), cv::Size(-1, -1), cv::TermCriteria(cv::TermCriteria::EPS + cv::TermCriteria::COUNT, 30, 0.1));
        image_corners = points;
        // cv::drawChessboardCorners(input_image, m_board_size, image_corners, found);
        // cv::imshow("debug", input_image);
        // cv::waitKey(0);
        return true;
    }
    return false;
}

//  void ImageFeatureDetector::calcBoardCornerPositions()
//  {
//      m_3d_corners.clear();
//      for (int i = 0; i < m_board_size.height; ++i)
//          for (int j = 0; j < m_board_size.width; ++j)
//              m_3d_corners.push_back(cv::Point3f(j * m_square_size, i * m_square_size, 0));
//     return;
//  }

//  void ImageFeatureDetector::estimate_RT(vector<cv::Point2d>& chessboard_corners, cv::Mat& rvec, cv::Mat&tvec)
//  {
//     cv::solvePnP(m_3d_corners, chessboard_corners, m_camera_matrix, m_dist_coeffs, rvec, tvec);
//     return;
//  }

// void MonoPattern::getOrderedCorner(const cv::Mat& cam_corners, pcl::PointCloud<pcl::PointXYZ>::Ptr corner_cloud)
// {
//   corner_cloud->clear();
//   // camera coordinate: x axis points to right, y axis points to down, z axis points to front which is vertical to x-y
//   // plane. So the top point's y is smallest.
//   double min_y = cam_corners.at<float>(1, 0);
//   int min_y_pos = 0;
//   for (int i = 1; i < 4; ++i)
//   {
//     if (cam_corners.at<float>(1, i) < min_y)
//     {
//       min_y = cam_corners.at<float>(1, i);
//       min_y_pos = i;
//     }
//   }
//   for (int i = 0; i < 4; ++i)
//   {
//     int cur_pos = (i + min_y_pos) % 4;
//     corner_cloud->points.emplace_back(cam_corners.at<float>(0, cur_pos), cam_corners.at<float>(1, cur_pos),
//                                       cam_corners.at<float>(2, cur_pos));
//   }
// }

void ImageFeatureDetector::estimatePose(vector<cv::Point2f> &chessboard_corners, cv::Mat &rvec, cv::Mat &tvec)
{
    std::vector<cv::Point2f> undistort_corners;
    cv::undistortPoints(chessboard_corners, undistort_corners, m_camera_matrix, m_dist_coeffs, cv::noArray(), m_camera_matrix);
    int &board_width = m_board_size.width;
    int &board_height = m_board_size.height;
    std::vector<cv::Point3d> board_points;
    for (int i = 0; i < board_height; i++)
    {
        for (int j = 0; j < board_width; j++)
        {
            board_points.push_back(
                cv::Point3d(double(i) * m_square_size, double(j) * m_square_size, 0.0));
        }
    }
    cv::solvePnP(cv::Mat(board_points), cv::Mat(chessboard_corners), m_camera_matrix, m_dist_coeffs, rvec, tvec);
    return;
}

void ImageFeatureDetector::calculate3DCorners(vector<cv::Point3d>& chessboard_3d_corners, cv::Mat& rvec, cv::Mat& tvec)
{
    int col = m_board_size.width + 1;
    int row = m_board_size.height + 1;
    // 添加padding
    chessboard_3d_corners.push_back(cv::Point3d((row-1) * m_square_size + m_padding[0], (col-1) * m_square_size + m_padding[2], 0));
    chessboard_3d_corners.push_back(cv::Point3d((row-1) * m_square_size - m_padding[2], -m_square_size + m_padding[1], 0));
    chessboard_3d_corners.push_back(cv::Point3d(-m_square_size - m_padding[2], -m_square_size - m_padding[3], 0));
    chessboard_3d_corners.push_back(cv::Point3d(-m_square_size + m_padding[0], (col-1) * m_square_size - m_padding[3], 0));

    cv::Matx33d rot_matrix;
    cv::Rodrigues(rvec, rot_matrix);
    for(auto& corner: chessboard_3d_corners){
        corner = rot_matrix*corner + cv::Point3d(tvec);
    }
    return ;
}

    // void calculateLines(std::vector<cv::Point3d>& corners, std::vector<Eigen::VectorXd>& lines); // 通过四个角点计算四条边的方程
    // void calculatePlane(std::vector<cv::Point3d>& corners, Eigen::VectorXd& plane); // 通过四个角点计算平面的方程

void ImageFeatureDetector::calculateLines(std::vector<cv::Point3d>& corners, std::vector<Eigen::VectorXd>& lines)
{
    assert(corners.size()==4);
    lines.clear();
    for(int i=0;i<4;++i){
        Eigen::VectorXd line(6);
        cv::Point3d line_pt = corners[i];
        cv::Point3d line_dir = corners[(i+1)%4]- corners[i];
        line<<line_pt.x, line_pt.y, line_pt.z, line_dir.x, line_dir.y, line_dir.z;
        lines.push_back(line);
    }
    return;
}

void ImageFeatureDetector::calculatePlane(std::vector<cv::Point3d>& corners, Eigen::VectorXd& plane)
{
    assert(corners.size()==4);
    Eigen::Vector3d a(corners[0].x, corners[0].y, corners[0].z );
    Eigen::Vector3d b(corners[1].x, corners[1].y, corners[1].z);
    Eigen::Vector3d c(corners[2].x, corners[2].y, corners[2].z);
    Eigen::Vector3d n;
    n = (b-a).cross(c-b);
    double d = -n.dot(a);
    plane = Eigen::Vector4d::Zero();
    plane.head<3>() = n;
    plane(3) = d;
    return;
}



void ImageFeatureDetector::calculatePlane1(vector<Point3d>& corners, VectorXd& plane)
{
    //Point3d zero(0.,0.,0.);
    //auto centroid = accumulate(corners.begin(), corners.end(),zero)/4.0;
    assert(corners.size()==4);
    auto func1 = [](vector<Point3d>&corners){
        Point3d p(0.,0.,0.);
        for(auto& corner:corners){
            p += corner;
        }
        return p;
    };
    auto centroid = func1(corners);
    centroid /= 4.0;
    Eigen::Vector3d a(corners[0].x, corners[0].y, corners[0].z );
    Eigen::Vector3d b(corners[1].x, corners[1].y, corners[1].z);
    Eigen::Vector3d c(corners[2].x, corners[2].y, corners[2].z);
    Eigen::Vector3d n;
    n = (b-a).cross(c-b);
    plane.resize(6);
    plane.head<3>() = n;
    plane.tail<3>() = Vector3d(centroid.x,centroid.y, centroid.z );
    return;
}

void processImage(Config& config, vector<string>& image_paths, ImageResults& images_features)
{
    ImageFeatureDetector image_feature_detector(config);
    auto& valid_image_index = images_features.valid_index;
    auto& image_3d_corners = images_features.corners_3d; // 图像3d角点
    auto& image_planes = images_features.planes_3d; // 图像的平面方程
    auto& image_lines = images_features.lines_3d; 

    for (int i = 0;i<image_paths.size();++i)
    {
        cv::Mat img = cv::imread(image_paths[i], cv::IMREAD_COLOR);
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
        image_3d_corners.push_back(corners_3d);
        valid_image_index.push_back(i);
    }
    return;
}