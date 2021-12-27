lidar_corners_3d0 = load("lidar3DCorners.mat").lidar3DCorners;
left_camera_corners_3d = load("left_imageCorners3d.mat").imageCorners3d;
tform = load("left_lidar_tform.mat").tform;
tform.Rotation
tform.Translation

lidar_corners_3d = reshape(permute(lidar_corners_3d0, [2, 1, 3]), size(lidar_corners_3d0, 2), [])';
lidar_corners_3d = lidar_corners_3d*(tform.Rotation) + tform.Translation;
lidar_corners_3d = permute(reshape(lidar_corners_3d', 3, 4, []), [2, 1, 3]);


left_camera_corners_3d(:,:,:)- lidar_corners_3d(:,:,:)

