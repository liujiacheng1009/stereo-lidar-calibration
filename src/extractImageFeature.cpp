 #include "extractImageFeature.hpp"

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