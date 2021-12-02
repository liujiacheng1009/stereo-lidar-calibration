 #include "extractImageFeature.hpp"

bool ImageFeatureDetector::detectImageCorner(cv::Mat &input_image, vector<cv::Point2f> &image_corners)
{
    cv::Mat gray_img;
    cv::cvtColor(input_image, gray_img, cv::COLOR_BGR2GRAY);
    std::vector<cv::Point2f> points;
    const int chessBoardFlags = cv::CALIB_CB_ADAPTIVE_THRESH | cv::CALIB_CB_NORMALIZE_IMAGE;
    bool found = cv::findChessboardCorners(gray_img, m_board_size, points, chessBoardFlags);
    // std::cout << points.size() << std::endl;
    if (found)
    {
        cv::cornerSubPix(gray_img, points, cv::Size(11, 11), cv::Size(-1, -1), cv::TermCriteria(cv::TermCriteria::EPS + cv::TermCriteria::COUNT, 30, 0.1));
        image_corners = points;
        cv::drawChessboardCorners(input_image, m_board_size, image_corners, found);
        return true;
    }
    return false;
    ;
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