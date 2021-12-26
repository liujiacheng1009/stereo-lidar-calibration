tform_left = load("left_lidar_tform.mat");
tform_right = load("right_lidar_tform.mat");
tform_lr = (tform_left.tform.T)\(tform_right.tform.T)
tform_rl = inv(tform_lr)