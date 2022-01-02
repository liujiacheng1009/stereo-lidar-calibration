import open3d as o3d
print("Testing IO for point cloud ...")
pcd = o3d.io.read_point_cloud("/home/jc/Documents/stereo-lidar-calibration/exclude_dir/lcc/HDL64/pointCloud/0001.pcd")
print(pcd)
o3d.visualization.draw_geometries([pcd],
                                  zoom=0.3412,
                                  front=[0.4257, -0.2125, -0.8795],
                                  lookat=[2.6172, 2.0475, 1.532],
                                  up=[-0.0694, -0.9768, 0.2024])
# o3d.io.write_point_cloud("copy_of_fragment.pcd", pcd)