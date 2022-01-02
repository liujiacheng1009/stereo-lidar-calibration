import open3d as o3d
print("Testing IO for point cloud ...")
pcd = o3d.io.read_point_cloud("/home/jc/Documents/stereo-lidar-calibration/matlab/0001.pcd")
print(pcd)
o3d.visualization.draw_geometries([pcd])
# o3d.io.write_point_cloud("copy_of_fragment.pcd", pcd)