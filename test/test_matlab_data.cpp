#include "matlab_data.hpp"
#include <iostream>

using namespace std;
using namespace Eigen;
int main()
{
    matlab_data1 m1("left");
    matlab_data1 m2("right");
    MatrixXd left_camera_matrix = m1.get_camera_intrinsic_matrix();
    cout<< "left camera intrinsic matrix： "<<endl;
    cout<<left_camera_matrix<< endl;

    VectorXd left_camera_distortion = m1.get_camera_distortion_vector();
    cout<< "left camera distortion vector: "<<endl;
    cout<< left_camera_distortion <<endl;

    vector<vector<Vector3d>> left_image_corners_3d = m1.get_image_corners_3d();
    cout<< "left image corners 3d : "<<endl;
    cout<< left_image_corners_3d.size()<<" X "<<left_image_corners_3d[0].size() <<endl;

    vector<vector<Vector2d>> left_camera_points = m1.get_image_points();
    cout<< "left camera points: " <<endl;
    cout<<left_camera_points.size() <<" X " << left_camera_points[0].size()<<endl;

    vector<vector<Vector3d>> left_lidar_corners_3d = m1.get_lidar_corners_3d();
    cout<< "left lidar corners 3d: "<<endl;
    cout<< left_lidar_corners_3d.size()<<" X "<<left_lidar_corners_3d[0].size()<<endl;

    MatrixXd left_camera_lidar_tform = m1.get_camera_lidar_tform();
    cout<< "left camera lidar tform: "<<endl;
    cout<< left_camera_lidar_tform<< endl;

    // for(int i = 0;i<left_lidar_corners_3d.size();++i){
    //     for(int j = 0;j<left_lidar_corners_3d[0].size();++j){
    //         VectorXd  lidar_p0 = left_lidar_corners_3d[i][j];
    //         VectorXd camera_p0 = left_image_corners_3d[i][j];
    //         cout<< lidar_p0.transpose() << endl;
    //         cout<< camera_p0.transpose()<<endl;
    //         VectorXd trans_p0 = left_camera_lidar_tform.block(0,0,3,3)* lidar_p0 + left_camera_lidar_tform.block(0,3,3,1);
    //         cout << (camera_p0 - trans_p0).transpose() << endl;
    //         cout << endl;
    //         cout << endl;
    //     }
    // }

    for(int i=0;i<8;++i){
        for(int j= 0;j<28;++j){
             Vector2d&  lidar_p0 = left_camera_points[i][j];
             cout<< lidar_p0.transpose()<< endl;
        }
    }
    return 0;
}