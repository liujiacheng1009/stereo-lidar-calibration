# stereo-lidar-calibration


// note 
1、matlab没有提取点云的边缘，数据中有些没有呈菱形摆放，如1，3，4，7；
所以，这里先以2号点云测试， marker board的提取，
先查看下点云能不能读成POINTXYZIR格式，
matlab 的toolbox 直接读取的点云PointXYZ, 没有用到 ring和 intensity特征

2、pass filter 需要确保包括chess board [-10,10]

3、可以直接通过限制点云簇的大小获得chessboard 点云

4、
