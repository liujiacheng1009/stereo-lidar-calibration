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
        pcl::PointXYZRGB rgb_point(cloud_point.x, cloud_point.y,cloud_point.z);
        if(x>=0&&x<width&&y>=0&&y<height){
            cv::Vec3b color = img.at<cv::Vec3b>(cv::Point(x, y));
            rgb_point.r = color[0];
            rgb_point.g = color[1];
            rgb_point.b = color[2];
        }
        color_pcd->push_back(rgb_point);
    }
    return color_pcd;
}