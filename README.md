# stereo-lidar-calibration
**双目与激光雷达外参联合标定算法**   
该算法利用位姿闭环约束联合了双目外参标定和相机与激光雷达标定过程，同时获得双目外参和相机与激光雷达外参估计。算法框图如下：
![image](https://user-images.githubusercontent.com/26436149/168028076-f15c1050-d20f-4cd4-ad9d-e7f3fddae00f.png)

**算法特点**
- 利用libcbdetect进行棋盘格检测，并对原接口进行了相应修改，相比于OpenCV函数具有更高的鲁棒性。
- 对于标定板点云的分割，实现了类似Matlab LCC的立方体拟合。标定板点云的顶点提取采用了拟合边缘直线交点的方式，之后还进行了顶点的检验和优化。该部分主要基于pcl实现。
- 对于图像和点云中的标定板同时提取了点、面特征。参数优化过程中将点约束、面约束、双目反投影约束、闭环位姿约束统一在同一优化框架下，参数初值利用kabsch算法估计得到。

/*当前算法精度不算高，仍在调试优化中，突出优势在于特征提取的鲁棒性较好*/

**使用方法**
本算法可以进行单目相机与激光雷达的外参标定和双目相机与激光雷达的外参标定，分别对应了samples/mono_lidar_calib.cpp，samples/stereo_lidar_calib.cpp。相机内参需要预先标定，推荐使用双目标定工具获得内参。

工程编译：mkdir build && cd build && cmake .. && make -j16 

- 单目相机与激光雷达外参标定  
首先，需要提供配置文件，以Matlab提供的hdl64线激光雷达数据为例(config/hdl64_mono.yaml), 需要提供的参数包括：标定类型、相机内参、图像和点云文件夹目录(文件名保持一致)、标定板参数、点云的虚拟边界、tform_l1_c1是用来对比的标定结果，这里提供的是Matlab LCC标定得到的参数。

标定命令：./samples/mono_lidar_calib -c  config/hdl64_mono.yaml

**标定结果**

特征提取：  
![image](https://user-images.githubusercontent.com/26436149/168028332-c4f058c8-8558-4bca-b21b-00b8edd300fa.png)  
![image](https://user-images.githubusercontent.com/26436149/168028368-1214535c-696b-4030-b8db-83b3634c398d.png)

标定外参对比:  

Matlab LCC  
![image](https://user-images.githubusercontent.com/26436149/168028778-7811aff9-ebd9-44c6-8514-dcdbc073276a.png)

Ours    
![image](https://user-images.githubusercontent.com/26436149/168028738-cface46f-0b5a-453d-b293-29fdad468cd1.png)

可视化结果：  
![image](https://user-images.githubusercontent.com/26436149/168028575-62710d46-e4e7-46ec-9b0b-bdc8dcb5c331.png)

- 双目相机与激光雷达外参标定  

标定命令：./samples/stereo_lidar_calib -c  config/$Your_Config_File$  
当前正在开发从Gazebo中自动获取仿真数据的工具，waiting for update.


**License**

任君取用：-）
