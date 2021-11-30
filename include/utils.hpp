#include <pcl/ModelCoefficients.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/point_types.h>
#include <string>
#include <unistd.h>


void display_colored_by_depth(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
{
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));

    pcl::visualization::PointCloudColorHandlerGenericField<pcl::PointXYZ> fildColor(cloud, "z"); // 按照z字段进行渲染

    viewer->addPointCloud<pcl::PointXYZ>(cloud, fildColor, "sample cloud");
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "sample cloud"); // 设置点云大小

    while (!viewer->wasStopped())
    {
        viewer->spinOnce(100);
        //boost::this_thread::sleep(boost::posix_time::microseconds(100000));
        sleep(0.1);
    }
}