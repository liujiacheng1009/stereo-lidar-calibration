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