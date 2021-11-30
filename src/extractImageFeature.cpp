 #include "extractImageFeature.hpp"
 
 void ImageFeatureDetector::detectImageCorner(cv::Mat& input_image, vector<cv::Point2f>& image_corners)
 {
     std::vector<cv::Point2f> points;
     const int chessBoardFlags = cv::CALIB_CB_ADAPTIVE_THRESH | cv::CALIB_CB_NORMALIZE_IMAGE;
     bool found = cv::findChessboardCorners(input_image, m_board_size, points, chessBoardFlags);
     if (found)
     {
         cv::Mat viewGray;
         cv::cvtColor(input_image, viewGray, cv::COLOR_BGR2GRAY);
         cornerSubPix(viewGray, points, cv::Size(11, 11), cv::Size(-1, -1), cv::TermCriteria(cv::TermCriteria::EPS + cv::TermCriteria::COUNT, 30, 0.1));
         image_corners = points;
     }
     return;
 }

 void ImageFeatureDetector::calcBoardCornerPositions()
 {
     m_3d_corners.clear();
     for (int i = 0; i < m_board_size.height; ++i)
         for (int j = 0; j < m_board_size.width; ++j)
             m_3d_corners.push_back(cv::Point3f(j * m_square_size, i * m_square_size, 0));
    return;
 }

 void ImageFeatureDetector::estimate_RT(vector<cv::Point2d>& chessboard_corners, cv::Mat& rvec, cv::Mat&tvec)
 {
    cv::solvePnP(m_3d_corners, chessboard_corners, m_camera_matrix, m_dist_coeffs, rvec, tvec);
    return;
 }