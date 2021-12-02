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

cv::Point2d project(cv::Point3d p, cv::Mat CameraMat)
{
    cv::Point2d pix;
    pix.x = (p.x * CameraMat.at<double>(0, 0)) / p.z + CameraMat.at<double>(0, 2);
    pix.y = ((p.y * CameraMat.at<double>(1, 1)) / p.z + CameraMat.at<double>(1, 2));
    return pix;
}
// #include <stdio.h>
// #include <stdlib.h>
// #include "mat.h"

// int diagnose(const char *file) {
//   MATFile *pmat;
//   const char **dir;
//   const char *name;
//   int	  ndir;
//   int	  i;
//   mxArray *pa;
  
//   printf("Reading file %s...\n\n", file);

//   /*
//    * Open file to get directory
//    */
//   pmat = matOpen(file, "r");
//   if (pmat == NULL) {
//     printf("Error opening file %s\n", file);
//     return(1);
//   }
  
//   /*
//    * get directory of MAT-file
//    */
//   dir = (const char **)matGetDir(pmat, &ndir);
//   if (dir == NULL) {
//     printf("Error reading directory of file %s\n", file);
//     return(1);
//   } else {
//     printf("Directory of %s:\n", file);
//     for (i=0; i < ndir; i++)
//       printf("%s\n",dir[i]);
//   }
//   mxFree(dir);

//   /* In order to use matGetNextXXX correctly, reopen file to read in headers. */
//   if (matClose(pmat) != 0) {
//     printf("Error closing file %s\n",file);
//     return(1);
//   }
//   pmat = matOpen(file, "r");
//   if (pmat == NULL) {
//     printf("Error reopening file %s\n", file);
//     return(1);
//   }

//   /* Get headers of all variables */
//   printf("\nExamining the header for each variable:\n");
//   for (i=0; i < ndir; i++) {
//     pa = matGetNextVariableInfo(pmat, &name);
//     if (pa == NULL) {
// 	printf("Error reading in file %s\n", file);
// 	return(1);
//     }
//     /* Diagnose header pa */
//     printf("According to its header, array %s has %d dimensions\n",
// 	   name, mxGetNumberOfDimensions(pa));
//     if (mxIsFromGlobalWS(pa))
//       printf("  and was a global variable when saved\n");
//     else
//       printf("  and was a local variable when saved\n");
//     mxDestroyArray(pa);
//   }

//   /* Reopen file to read in actual arrays. */
//   if (matClose(pmat) != 0) {
//     printf("Error closing file %s\n",file);
//     return(1);
//   }
//   pmat = matOpen(file, "r");
//   if (pmat == NULL) {
//     printf("Error reopening file %s\n", file);
//     return(1);
//   }

//   /* Read in each array. */
//   printf("\nReading in the actual array contents:\n");
//   for (i=0; i<ndir; i++) {
//       pa = matGetNextVariable(pmat, &name);
//       if (pa == NULL) {
// 	  printf("Error reading in file %s\n", file);
// 	  return(1);
//       } 
//       /*
//        * Diagnose array pa
//        */
//       printf("According to its contents, array %s has %d dimensions\n",
// 	     name, mxGetNumberOfDimensions(pa));
//       if (mxIsFromGlobalWS(pa))
// 	printf("  and was a global variable when saved\n");
//       else
// 	printf("  and was a local variable when saved\n");
//       mxDestroyArray(pa);
//   }

//   if (matClose(pmat) != 0) {
//       printf("Error closing file %s\n",file);
//       return(1);
//   }
//   printf("Done\n");
//   return(0);
// }

// int main(int argc, char **argv)
// {
  
//   int result;

//   if (argc > 1)
//     result = diagnose(argv[1]);
//   else{
//     result = 0;
//     printf("Usage: matdgns <matfile>");
//     printf(" where <matfile> is the name of the MAT-file");
//     printf(" to be diagnosed\n");
//   }

//   return (result==0)?EXIT_SUCCESS:EXIT_FAILURE;

// }