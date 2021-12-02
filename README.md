# stereo-lidar-calibration


// note 
1、matlab没有提取点云的边缘，数据中有些没有呈菱形摆放，如1，3，4，7；
所以，这里先以2号点云测试， marker board的提取，
先查看下点云能不能读成POINTXYZIR格式，
matlab 的toolbox 直接读取的点云PointXYZ, 没有用到 ring和 intensity特征

2、pass filter 需要确保包括chess board [-10,10]

3、可以直接通过限制点云簇的大小获得chessboard 点云

4、cv::findChessboardCorners 和 cv::cornerSubPix 获得的角点类型是cv::Point2f, 其他类型会报错。

5、cv::undistortPoints的调用方式为：
cv::undistortPoints(chessboard_corners, undistort_corners, m_camera_matrix, m_dist_coeffs, cv::noArray(), m_camera_matrix);
最后的camera_matrix不能是noArray();

6、cv::findChessboardCorners和matlab 得到的角点的排列顺序不同