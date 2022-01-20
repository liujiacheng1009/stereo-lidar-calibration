#include "extractImageFeature.hpp"
#include "detect_corners.hpp"

using namespace std;
using namespace Eigen;

void processImage(vector<string>& image_paths, ImageResults& images_features)
{
    ImageFeatureDetector image_feature_detector;
    auto& valid_image_index = images_features.valid_index;
    auto& image_3d_corners = images_features.corners_3d; // 图像3d角点
    auto& chessboard_2d_points = images_features.chessboard_points_2d;
    auto& chessboard_3d_points = images_features.chessboard_points_3d;
    auto& image_planes = images_features.planes_3d; // 图像的平面方程
    auto& image_lines = images_features.lines_3d;

    for (int i = 0;i<image_paths.size();++i)
    {
        cv::Mat img = cv::imread(image_paths[i], cv::IMREAD_COLOR);
        std::vector<cv::Point2f> image_corners;
        cv::Mat rvec, tvec;

        // if (!image_feature_detector.detectImageCorner1(img, image_corners)) continue;
        // image_feature_detector.estimatePose1(image_corners,chessboard_points_3d, rvec, tvec);
        
        if(!image_feature_detector.detectCheckerboardInImage(img, image_corners,rvec, tvec)) continue;

        // debug 
        // if(1){
        //     cv::drawChessboardCorners(img, Config::checkerboardGridSize(), image_corners, true);
        //     draw_axis(img, rvec,  tvec, Config::leftCameraMatrix(), Config::leftCameraDistCoeffs());
        //     cv::imshow("debug", img);
        //     cv::waitKey(0);
        // }

        vector<Vector3d> chessboard_points_3d;
        vector<Vector2d> chessboard_points_2d;
        image_feature_detector.transform_to_normalized_plane(image_corners, chessboard_points_2d);
        std::vector<cv::Point3d> chessboard_3d_corners, reordered_image_3d_corners;
        image_feature_detector.calculate3DCorners(chessboard_3d_corners, rvec, tvec);
        //debug
        // if(1){
        //     for(auto& corner:chessboard_3d_corners){
        //         std::cout<< corner<<std::endl;
        //     }
        //     std::cout<<std::endl;
        // }
        // if(1){
        //     cv::Mat rvec_temp = cv::Mat::zeros(cv::Size(3,1), CV_64FC1);
        //     cv::Mat tvec_temp = cv::Mat::zeros(cv::Size(3,1), CV_64FC1);
        //     draw_corners_in_image(img,chessboard_3d_corners,rvec_temp, tvec_temp, Config::leftCameraMatrix(), Config::leftCameraDistCoeffs() );
        //     cv::imshow("debug", img);
        //     cv::waitKey(0);
        // }

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


bool ImageFeatureDetector::detectImageCorner(cv::Mat &input_image, vector<cv::Point2f> &image_corners)
{
    image_corners.clear();
    cv::Mat gray_img;
    cv::cvtColor(input_image, gray_img, cv::COLOR_BGR2GRAY);
    std::vector<cv::Point2f> points;
    const int chessBoardFlags = cv::CALIB_CB_ADAPTIVE_THRESH | cv::CALIB_CB_FILTER_QUADS;
    // cout<< m_board_size<<endl;
    // cv::imshow("debug", gray_img);
    // cv::waitKey(0);
    bool found = cv::findChessboardCorners(gray_img, m_board_size, points, chessBoardFlags);

    if (found)
    {
        cv::cornerSubPix(gray_img, points, cv::Size(11, 11), cv::Size(-1, -1), cv::TermCriteria(cv::TermCriteria::EPS + cv::TermCriteria::COUNT, 30, 0.1));
        // image_corners = points;
        cv::undistortPoints(points, image_corners, m_camera_matrix, m_dist_coeffs, cv::noArray(), m_camera_matrix);
        // debug 
        // for(auto& corner: image_corners){
        //     cout<< corner <<endl;
        // }
        // cv::drawChessboardCorners(input_image, m_board_size, image_corners, found);
        // cv::imshow("debug", input_image);
        // cv::waitKey(0);
        // exit(17);
        return true;
    }
    return false;
}

void ImageFeatureDetector::estimatePose(vector<cv::Point2f> &chessboard_corners, cv::Mat &rvec, cv::Mat &tvec)
{
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
    // chessboard_points_3d.clear();
    // cv::Matx33d rot_matrix;
    // cv::Rodrigues(rvec, rot_matrix);
    // for(auto& point:board_points){
    //     auto p = rot_matrix*point + cv::Point3d(tvec);
    //     chessboard_points_3d.push_back(Vector3d(p.x,p.y,p.z));
    // }
    return;
}


void ImageFeatureDetector::calculate3DCorners(vector<cv::Point3d>& chessboard_3d_corners, cv::Mat& rvec, cv::Mat& tvec)
{
    int col = m_board_size.width + 1;
    int row = m_board_size.height + 1;
    // 添加padding
    chessboard_3d_corners.push_back(cv::Point3d((row-1) * m_square_size + m_padding[0], (col-1) * m_square_size + m_padding[3], 0));
    chessboard_3d_corners.push_back(cv::Point3d((row-1) * m_square_size + m_padding[0], -m_square_size - m_padding[1], 0));
    chessboard_3d_corners.push_back(cv::Point3d(-m_square_size - m_padding[2], -m_square_size - m_padding[1], 0));
    chessboard_3d_corners.push_back(cv::Point3d(-m_square_size - m_padding[2], (col-1) * m_square_size + m_padding[3], 0));

    cv::Matx33d rot_matrix;
    cv::Rodrigues(rvec, rot_matrix);
    for(auto& corner: chessboard_3d_corners){
        corner = rot_matrix*corner + cv::Point3d(tvec);
    }
    
    // debug
    // vector<pcl::PointXYZ> corners_temp;
    // for(auto& corner:chessboard_3d_corners){
    //     corners_temp.push_back(pcl::PointXYZ(corner.x, corner.y, corner.z));
    // }
    // display_four_corners(corners_temp);
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


void ImageFeatureDetector::calculatePlane1(vector<cv::Point3d>& corners, VectorXd& plane)
{
    //Point3d zero(0.,0.,0.);
    //auto centroid = accumulate(corners.begin(), corners.end(),zero)/4.0;
    assert(corners.size()==4);
    auto func1 = [](vector<cv::Point3d>&corners){
        cv::Point3d p(0.,0.,0.);
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


void ImageFeatureDetector::transform_to_normalized_plane(vector<cv::Point2f>& corners1, vector<Eigen::Vector2d>& corners2)
{
    corners2.clear();
    cv::Matx33d K((double*)m_camera_matrix.ptr());
    for(auto& corner:corners1){
        cv::Point3d p(corner.x, corner.y, 1.0);
        auto p1 = K.inv()*p;
        corners2.push_back(Vector2d(p1.x,p1.y));
    }
    return;
}

bool ImageFeatureDetector::detectCheckerboardInImage(cv::Mat &img,
                                                     std::vector<cv::Point2f> &image_corners,
                                                     cv::Mat &rvec,
                                                     cv::Mat &tvec,
                                                     string method)
{
    if(method=="cbdetect"){
        return detectCheckerboardInImageByCbdetect(img, image_corners, rvec, tvec);
    }else if(method=="opencv"){
        return detectCheckerboardInImageByOpenCV(img, image_corners, rvec, tvec);
    }
    else{
        std::cout<< "角点检测方法只能是cbdetect和opencv ..."<<std::endl;
    }
    return false;
}

bool ImageFeatureDetector::detectCheckerboardInImageByCbdetect(cv::Mat &input_image,
                                                               std::vector<cv::Point2f> &chessboard_corners,
                                                               cv::Mat &rvec,
                                                               cv::Mat &tvec)
{
    if(!cbdetectModified::detectCorner(input_image,chessboard_corners)) return false;
    cbdetectModified::estimatePoseAndCompleteCorners(chessboard_corners,rvec, tvec);
    return true;
}

bool ImageFeatureDetector::detectCheckerboardInImageByOpenCV(cv::Mat &input_image,
                                                             std::vector<cv::Point2f> &chessboard_corners,
                                                             cv::Mat &rvec,
                                                             cv::Mat &tvec)
{
    // To do 
    if(!detectImageCorner(input_image,chessboard_corners)) return false;
    estimatePose(chessboard_corners,rvec, tvec);
    return true;
}