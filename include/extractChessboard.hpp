#include <pcl/common/common.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/eigen.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/passthrough.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_plane.h>


struct PassFilterParams{
    double min_x,max_x;
    double min_y,max_y;
    double min_z, max_z;
    PassFilterParams(std::vector<double> params){
        assert(params.size()==6);
        min_x = params[0];
        max_x = params[1];
        min_y = params[2];
        max_y = params[3];
        min_z = params[4];
        max_z = params[5];
    }
};

class chessboardExtractor{
public:
    chessboardExtractor(PassFilterParams& pass_filter_params):m_pass_filter_params(pass_filter_params){}
public:
    // 主函数， 从环境点云中提取marker board 点云
    void extract(pcl::PointCloud<pcl::PointXYZ>::Ptr input_lidar_point_cloud,
                 pcl::PointCloud<pcl::PointXYZ>::Ptr chessboard_point_cloud);

    // 点云聚类， 获取可能的点云簇
    void get_pcds_by_cluster(pcl::PointCloud<pcl::PointXYZ>::Ptr input_pcd,
        std::vector<pcl::PointIndices> pcd_clusters);
    // 对上面的点云簇进行平面拟合 
    bool fitPlane(pcl::PointCloud<pcl::PointXYZ>::Ptr input_pcd,
        pcl::PointCloud<pcl::PointXYZ>::Ptr plane_pcd);

    // 将原始点云投影到估计的平面上
    void projPlaneFilter(pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud,
                         pcl::PointCloud<pcl::PointXYZ>::Ptr &proj_cloud,
                         pcl::ModelCoefficients::Ptr &coeff);

    void pass_filter(pcl::PointCloud<pcl::PointXYZ>::Ptr& input_pcd);

private:
    // 预处理， 滤出固定范围外的点云
    void getPointCloudInROI();
    void extract_pcd_by_indices(pcl::PointIndices &indices, pcl::PointCloud<pcl::PointXYZ>::Ptr &input_pcd,
                                pcl::PointCloud<pcl::PointXYZ>::Ptr &output_pcd);
    PassFilterParams m_pass_filter_params;
};

