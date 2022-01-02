#include <limits.h>
#include "extractLidarFeature.hpp"
#include "gtest/gtest.h"
using namespace std;
using namespace pcl;

TEST(UtilsTest, LoadPointXYZ) {
  string path1 = "/home/jc/Documents/stereo-lidar-calibration/exclude_dir/lcc/HDL64/pointCloud/0001.pcd";
  string path2 = "/home/jc/Documents/stereo-lidar-calibration/exclude_dir/lcc/vlp16/pointCloud/0001.pcd";
  PointCloud<PointXYZ>::Ptr cloud(new PointCloud<PointXYZ>);
  // EXPECT_EQ(true, loadPointXYZ(path1,cloud));
  EXPECT_EQ(true, loadPointXYZ(path2,cloud));
}