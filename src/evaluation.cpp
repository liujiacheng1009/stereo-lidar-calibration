#include "evaluation.hpp"

void projectLidarOnImage(cv::Mat &img,
                         pcl::PointCloud<pcl::PointXYZ> &cloud,
                         cv::Mat &rvec,
                         cv::Mat &tvec,
                         cv::Mat K,
                         cv::Mat dist)
{
    std::vector<cv::Point3d> cloud_points;
    for(auto& point:cloud){
        cloud_points.push_back(cv::Point3d(point.x, point.y, point.z));
    }
    draw_corners_in_image(img, cloud_points, rvec,tvec, K, dist);
}

pcl::PointCloud<pcl::PointXYZRGB>::Ptr colorizeCloud(cv::Mat &img,
                                                     pcl::PointCloud<pcl::PointXYZ> &cloud,
                                                     cv::Mat &rvec,
                                                     cv::Mat &tvec,
                                                     cv::Mat K,
                                                     cv::Mat dist)
{
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr color_pcd(new pcl::PointCloud<pcl::PointXYZRGB>);
    std::vector<cv::Point3d> cloud_points;
    for(auto& point:cloud){
        cloud_points.push_back(cv::Point3d(point.x, point.y, point.z));
    }
    std::vector<cv::Point2d> image_corners;
    cv::projectPoints(cloud_points, rvec, tvec, K , dist, image_corners);
    int width = img.size().width, height = img.size().height;
    for(int i = 0;i<cloud_points.size();++i){
        auto& cloud_point = cloud_points[i];
        auto& image_corner = image_corners[i];
        int x = image_corner.x, y = image_corner.y;
        pcl::PointXYZRGB rgb_point;
        rgb_point.x = cloud_point.x;
        rgb_point.y = cloud_point.y;
        rgb_point.z = cloud_point.z;

        //debug
        // {
        //     std::cout<< cloud_point.x<<std::endl;
        //     std::cout<< cloud_point.y<<std::endl;
        //     std::cout<< cloud_point.z<<std::endl;
        // }


        if(cloud_point.x<0.0){
            rgb_point.r = 255;
            rgb_point.g = 0;
            rgb_point.b = 0;
        }
        else if (x >= 0 && x < width && y >= 0 && y < height)
        {
            rgb_point.b = img.at<cv::Vec3b>(y, x)[0]; // blue
            rgb_point.g = img.at<cv::Vec3b>(y, x)[1]; // green
            rgb_point.r = img.at<cv::Vec3b>(y, x)[2];

            //debug
            // {
            //     std::cout << x <<" "<< y<<std::endl;
            //     std::cout<< img.at<cv::Vec3b>(y,x)[0]<< " "<<img.at<cv::Vec3b>(y,x)[1] << " "<< img.at<cv::Vec3b>(y,x)[2] <<std::endl;
            //     // exit(17);
            // }
        }
        color_pcd->points.push_back(rgb_point);
    }
    return color_pcd;
}

Eigen::Vector3d get_centroid(std::vector<Eigen::Vector3d>& corners_3d)
{
    int n = corners_3d.size();
    Eigen::Vector3d centroid = Eigen::Vector3d::Zero();
    for(auto& corner:corners_3d){
        centroid += corner/n;
    }
    return centroid;
}


Eigen::Vector3d get_normal(std::vector<Eigen::Vector3d>& corners_3d)
{
    int n = corners_3d.size();
    assert(n>=3);
    auto line1 = corners_3d[0]-corners_3d[1];
    auto line2 = corners_3d[1]-corners_3d[2];
    auto normal = line1.cross(line2);
    normal /= normal.norm();
    return normal;
}

void project_to_image_axis(std::vector<std::vector<Eigen::Vector3d>> &cloud_corners_3d,
                                            std::vector<std::vector<Eigen::Vector3d>> &transformed_cloud_corners_3d,
                                            Eigen::Matrix4d &tform)
{
    transformed_cloud_corners_3d.clear();
    auto rot = tform.block(0,0,3,3);
    auto trans = tform.block(0,3,3,1);
    int n_pairs = cloud_corners_3d.size();
    for(int i = 0;i<n_pairs;++i){
        for(int j = 0;j<4;++j){
            auto& cloud_corner_3d = cloud_corners_3d[i][j];
            transformed_cloud_corners_3d[i].push_back(rot*cloud_corner_3d+trans);
        }
    }
}

std::vector<double> computeTranslationError(std::vector<std::vector<Eigen::Vector3d>> &image_corners_3d,
                                            std::vector<std::vector<Eigen::Vector3d>> &cloud_corners_3d,
                                            Eigen::Matrix4d &tform)
{
    int n_pairs = image_corners_3d.size();
    std::vector<double> translation_errors;
    std::vector<std::vector<Eigen::Vector3d>> transformed_cloud_corners_3d(n_pairs);
    project_to_image_axis(cloud_corners_3d, transformed_cloud_corners_3d, tform);
    for(int i = 0;i<n_pairs;++i){
        auto image_centroid = get_centroid(image_corners_3d[i]);
        auto cloud_centroid = get_centroid(transformed_cloud_corners_3d[i]);
        translation_errors.push_back((image_centroid-cloud_centroid).norm());
    }
    return translation_errors;
}

std::vector<double> computeRotationError(std::vector<std::vector<Eigen::Vector3d>> &image_corners_3d,
                                            std::vector<std::vector<Eigen::Vector3d>> &cloud_corners_3d,
                                            Eigen::Matrix4d &tform)
{
    int n_pairs = image_corners_3d.size();
    std::vector<double> rotate_errors;
    std::vector<std::vector<Eigen::Vector3d>> transformed_cloud_corners_3d(n_pairs);
    project_to_image_axis(cloud_corners_3d, transformed_cloud_corners_3d, tform);
    for(int i = 0;i<n_pairs;++i){
        auto image_normal = get_normal(image_corners_3d[i]);
        auto cloud_normal = get_normal(transformed_cloud_corners_3d[i]);

        double rot_error =image_normal.dot(cloud_normal) /(image_normal.norm()*cloud_normal.norm());
        rot_error = acos(rot_error) * 180 / M_PI;
        if(rot_error>90){
            rot_error = 180- rot_error;
        }
        rotate_errors.push_back(rot_error);
    }
    return rotate_errors;
}

std::vector<double> computeReprojectionError(std::vector<std::vector<Eigen::Vector3d>> &image_corners_3d,
                                            std::vector<std::vector<Eigen::Vector3d>> &cloud_corners_3d,
                                            Eigen::Matrix4d &tform)
{
    // todo
    auto camera_matrix = Config::leftCameraMatrix();
    auto dist_coeff = Config::leftCameraDistCoeffs();
    int n_pairs = image_corners_3d.size();
    std::vector<double> reprojection_errors;
    std::vector<std::vector<Eigen::Vector3d>> transformed_cloud_corners_3d(n_pairs);
    project_to_image_axis(cloud_corners_3d, transformed_cloud_corners_3d, tform);

    std::vector<cv::Point3d> image_centroids, cloud_centroids;
    for(int i = 0;i<n_pairs;++i){
        auto image_centroid = get_centroid(image_corners_3d[i]);
        auto cloud_centroid = get_centroid(transformed_cloud_corners_3d[i]);
        image_centroids.push_back(cv::Point3d(image_centroid(0),image_centroid(1),image_centroid(2)));
        cloud_centroids.push_back(cv::Point3d(cloud_centroid(0),cloud_centroid(1),cloud_centroid(2)));
    }
    cv::Mat rvec = cv::Mat::zeros(cv::Size(3,1), CV_64FC1);
    cv::Mat tvec = cv::Mat::zeros(cv::Size(3,1), CV_64FC1);
    std::vector<cv::Point2d> image_centroids_2d, cloud_centroids_2d;
    cv::projectPoints(image_centroids, rvec, tvec, camera_matrix, dist_coeff, image_centroids_2d);
    cv::projectPoints(cloud_centroids, rvec, tvec, camera_matrix, dist_coeff, cloud_centroids_2d);

    for(int i = 0;i<n_pairs;++i ){
        reprojection_errors.push_back(cv::norm(image_centroids_2d[i]-cloud_centroids_2d[i]));
    }
    return reprojection_errors;
}