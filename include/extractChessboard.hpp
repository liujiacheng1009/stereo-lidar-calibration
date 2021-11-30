#include <pcl/common/common.h>

typedef pcl::PointXYZ LidarPoint;
typedef pcl::PointCloud<LidarPoint> LidarPointCloud;




class chessboardExtractor{
public:
    // 主函数， 从环境点云中提取marker board 点云
    void extract(LidarPointCloud::Ptr input_lidar_point_cloud, LidarPointCloud::Ptr
        chessboard_point_cloud);

    // 点云聚类， 获取可能的点云簇
    void get_pcds_by_cluster(pcl::PointCloud<pcl::PointXYZ>::Ptr input_pcd,
        vector<pcl::PointIndices> pcd_clusters);
    // 对上面的点云簇进行平面拟合 
    bool fitPlane(pcl::PointCloud<pcl::PointXYZ>::Ptr input_pcd,
        pcl::PointCloud<pcl::PointXYZ>::Ptr plane_pcd);

private:
    // 预处理， 滤出固定范围外的点云
    void getPointCloudInROI();
    void extract_pcd_by_indices(pcl::PointIndices &indices, pcl::PointCloud<pcl::PointXYZ>::Ptr &input_pcd,
                                pcl::PointCloud<pcl::PointXYZ>::Ptr &output_pcd);
    double max_x_ = 10;
    double min_x_ = -10;
    double max_y_ = 10;
    double min_y_ = -10;
    double max_z_ = 10;
    double min_z_ = -10;

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in_roi_;

};

