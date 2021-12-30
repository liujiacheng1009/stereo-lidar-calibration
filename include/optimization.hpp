#ifndef STEREO_LIDAR_CALIBRATION_OPTIMIZATION_HPP
#define STEREO_LIDAR_CALIBRATION_OPTIMIZATION_HPP

#include <Eigen/Core>
#include <ceres/ceres.h>
#include <ceres/rotation.h>     /// ceres::AngleAxisRotatePoint
#include <opencv2/core.hpp>
#include <Eigen/Geometry>
#include <ceres/autodiff_cost_function.h>
#include <pcl/io/ply_io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/console/parse.h>
#include <pcl/features/eigen.h>
#include <pcl/features/feature.h>
#include <pcl/features/normal_3d.h>
#include <pcl/impl/point_types.hpp>
#include <pcl/features/boundary.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_line.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/project_inliers.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/common/intersections.h>
#include <pcl/registration/icp.h>
#include <pcl/common/distances.h>
#include <sophus/so3.hpp>
using namespace std;
using namespace Eigen;


// closedup error

class ClosedupError
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    ClosedupError(const double& w):
                    m_w(w){}
    template<typename T>
    bool operator()(const T* const Rt1, const T* const Rt2, const T* const Rt3,T* residual) const{
        T rot1[3*3];
        ceres::AngleAxisToRotationMatrix(Rt1, rot1);
        Eigen::Matrix<T, 3, 3> R1 = Eigen::Matrix<T, 3, 3>::Identity();
        for (int i = 0; i < 3; ++i) {
            for (int j = 0; j < 3; ++j) {
                R1(i, j) = rot1[i+j*3];
            }
        }
        Eigen::Map<const Eigen::Matrix<T,3,1>> t1(Rt1+3);
        Eigen::Matrix<T,4,4> trans1 = Eigen::Matrix<T,4,4>::Identity();
        trans1.block(0,0,3,3) = R1;
        trans1.block(0,3,3,1) = t1;

        T rot2[3*3];
        ceres::AngleAxisToRotationMatrix(Rt2, rot2);
        Eigen::Matrix<T, 3, 3> R2 = Eigen::Matrix<T, 3, 3>::Identity();
        for (int i = 0; i < 3; ++i) {
            for (int j = 0; j < 3; ++j) {
                R2(i, j) = rot2[i+j*3];
            }
        }
        Eigen::Map<const Eigen::Matrix<T,3,1>> t2(Rt2+3);
        Eigen::Matrix<T,4,4> trans2 = Eigen::Matrix<T,4,4>::Identity();
        trans2.block(0,0,3,3) = R2;
        trans2.block(0,3,3,1) = t2;

        T rot3[3*3];
        ceres::AngleAxisToRotationMatrix(Rt3, rot3);
        Eigen::Matrix<T, 3, 3> R3 = Eigen::Matrix<T, 3, 3>::Identity();
        for (int i = 0; i < 3; ++i) {
            for (int j = 0; j < 3; ++j) {
                R3(i, j) = rot3[i+j*3];
            }
        }
        Eigen::Map<const Eigen::Matrix<T,3,1>> t3(Rt3+3);
        Eigen::Matrix<T,4,4> trans3 = Eigen::Matrix<T,4,4>::Identity();
        trans3.block(0,0,3,3) = R3;
        trans3.block(0,3,3,1) = t3;

        Eigen::Matrix<T,4,4> loop = trans1*trans3*(trans2.inverse().template cast<T>());
        residual[0] = loop.block(0,3,3,1).norm();
        residual[0] *= m_w;
        residual[1] = (Matrix<T,3,3>::Identity()-loop.block(0,0,3,3)).trace();
        residual[1] *= m_w;
        return true;
    }
private:
    const double m_w;
};


// 点到点的距离损失
class PointToPointError
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW 
    PointToPointError(const Eigen::Vector3d& point_lidar,
                    const Eigen::Vector3d& point_cam,
                    const double& w):
                    m_point_lidar(point_lidar),
                    m_point_cam(point_cam),
                    m_w(w){}

    template<typename T>
    bool operator()(const T* const R_t,T*residual) const{
        T rot[3*3];
        ceres::AngleAxisToRotationMatrix(R_t, rot);
        Eigen::Matrix<T, 3, 3> R = Eigen::Matrix<T, 3, 3>::Identity();
        for (int i = 0; i < 3; ++i) {
            for (int j = 0; j < 3; ++j) {
                R(i, j) = rot[i+j*3];
            }
        }
        Eigen::Map<const Eigen::Matrix<T,3,1>> trans(R_t+3);
        residual[0] = (m_point_cam.template cast<T>() - (R * m_point_lidar.template cast<T>() + trans)).norm();
        residual[0] *= m_w;
        return true;
    }

private:
    const Eigen::Vector3d m_point_lidar;
    const Eigen::Vector3d m_point_cam;
    const double m_w;
};

class PointToPointError1
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW 
    PointToPointError1(const Eigen::Vector3d& point_lidar,
                    const Eigen::Vector3d& point_cam,
                    const double& w):
                    m_point_lidar(point_lidar),
                    m_point_cam(point_cam),
                    m_w(w){}

    template<typename T>
    bool operator()(const T* const R_t,T*residual) const{
        T p1[3] = {
            T(m_point_lidar(0)),
            T(m_point_lidar(1)),
            T(m_point_lidar(2))
        };
        T p2[3];
        ceres::AngleAxisRotatePoint(R_t, p1, p2);
        p2[0] += R_t[3];
        p2[1] += R_t[4];
        p2[2] += R_t[5];
        T p3[3] = {
            T(m_point_cam(0)),
            T(m_point_cam(1)),
            T(m_point_cam(2))
        };
        residual[0] = m_w * (p3[0]-p2[0]);
        residual[1] = m_w * (p3[1]-p2[1]);
        residual[2] = m_w * (p3[2]-p2[2]);
        return true;
    }

private:
    const Eigen::Vector3d m_point_lidar;
    const Eigen::Vector3d m_point_cam;
    const double m_w;
};

// 计算点到直线的距离损失

// class PointToLineError2
// {
// public:
//     EIGEN_MAKE_ALIGNED_OPERATOR_NEW 
//     PointToLineError2(const Eigen::Vector3d& pt,
//                     const Eigen::Vector3d& line_pt,
//                     const Eigen::Vector3d& line_dir, 
//                     const double& w):
//                     m_pt(pt),
//                     m_line_pt(line_pt),
//                     m_line_dir(line_dir),
//                     m_w(w){}
//     template<typename T>
//     bool operator ()(const T* const R_t, T* residual) const
//     {
//         T p1[3] = {
//             T(m_pt(0)),
//             T(m_pt(1)),
//             T(m_pt(2))
//         };
//         T p2[3];
//         ceres::AngleAxisRotatePoint(R_t, p1, p2);
//         p2[0] += R_t[3];
//         p2[1] += R_t[4];
//         p2[2] += R_t[5];
//         Eigen::Matrix<T,4, 1> pt, line_pt, line_dir;
//         pt<< (T)p2[0], (T)p2[1], (T)p2[2], (T)0.0;
//         line_pt << (T)m_line_pt[0], (T)m_line_pt[1], (T)m_line_pt[2], (T)0.0;
//         line_dir<< (T)m_line_dir[0], (T)m_line_dir[1], (T)m_line_dir[2], (T)0.0;
//         double err = sqrt(pcl::sqrPointToLineDistance(pt, line_pt, line_dir));
//         Eigen::Matrix<double,1,1> err1;
//         err1(0) = err;
//         residual[0] = err1.template cast<T>().norm();
//         residual[0] *= m_w;
//         return true;
//     }
// private:
//     const Eigen::Vector3d m_pt;
//     const Eigen::Vector3d m_line_pt;
//     const Eigen::Vector3d m_line_dir;
//     const double m_w;
// };



// 计算点到直线的损失
class PointToLineError1
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW 
    PointToLineError1(const Eigen::Vector3d& point,
                    const Eigen::Vector3d& line_normal,
                    const double& w):
                    m_point(point),
                    m_line_normal(line_normal),
                    m_w(w){}
    template<typename T>
    bool operator () (const T* const R_t,
                      T* residual) const {
        T l_pt_L[3] = {T(m_point(0)),
                       T(m_point(1)),
                       T(m_point(2))};
        T n_C[3] = {T(m_line_normal(0)),
                    T(m_line_normal(1)),
                    T(m_line_normal(2))};
        T l_pt_C[3];
        ceres::AngleAxisRotatePoint(R_t, l_pt_L, l_pt_C);
        l_pt_C[0] += R_t[3];
        l_pt_C[1] += R_t[4];
        l_pt_C[2] += R_t[5];
        Eigen::Matrix<T, 3, 1> laser_point_C(l_pt_C);
        Eigen::Matrix<T, 3, 1> normal_C(n_C);
        residual[0] = normal_C.normalized().dot(laser_point_C);
        residual[0] = m_w * residual[0];
        return true;
    }
private:
    const Eigen::Vector3d m_point;
    const Eigen::Vector3d m_line_normal;
    const double m_w;
};

// 计算点到平面的损失
class PointToPlaneError
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    PointToPlaneError(const Eigen::Vector3d &point,
                      const Eigen::Vector3d &plane_normal,
                      const Eigen::Vector3d &plane_centroid,
                      const double &w) : m_point(point),
                                         m_plane_normal(plane_normal),
                                         m_plane_centroid(plane_centroid),
                                         m_w(w) {}

    template<typename T>
    bool operator() (const T* const R_t,
                     T* residual) const {
        T l_pt_L[3] = {T(m_point(0)), T(m_point(1)), T(m_point(2))};
        T r_3[3] = {T(m_plane_normal(0)), T(m_plane_normal(1)), T(m_plane_normal(2))};
        T t_vec[3] = {T(m_plane_centroid(0)), T(m_plane_centroid(1)), T(m_plane_centroid(2))};

        T l_pt_C[3];
        ceres::AngleAxisRotatePoint(R_t, l_pt_L, l_pt_C);
        l_pt_C[0] += R_t[3];
        l_pt_C[1] += R_t[4];
        l_pt_C[2] += R_t[5];

        Eigen::Matrix<T, 3, 1> laser_point_C(l_pt_C);
        Eigen::Matrix<T, 3, 1> r3_C(r_3);
        Eigen::Matrix<T, 3, 1> tvec_C(t_vec);
        residual[0] = r3_C.dot(laser_point_C - tvec_C);
        residual[0] = m_w * residual[0];
        return true;
    }
private:
    const Eigen::Vector3d m_point;
    const Eigen::Vector3d m_plane_normal;
    const Eigen::Vector3d m_plane_centroid;
    const double m_w;
};

// class PoseLoopClosingError
// {
// public:
//     EIGEN_MAKE_ALIGNED_OPERATOR_NEW
//     PoseLoopClosingError(Sophus::SE3d t_ij):m_t_ij(t_ij){}
//     template <typename T>
//     bool operator()(const T *const R_t_L1_C1,
//                     const T *const R_t_L1_C2,
//                     T *residual) const
//     {
//         Eigen::Map<const Sophus::SE3<T>> t_i(R_t_L1_C1);
//         Eigen::Map<const Sophus::SE3<T>> t_j(R_t_L1_C2);
//         Eigen::Map<Eigen::Matrix<T, 6, 1>> residual(residual_ptr);

//         residual = (t_i * t_ij.template cast<T>() * t_j.inverse()).log();
//         return true;
//     }

// private:
//     Sophus::SE3d m_t_ij;
// };

class StereoMatchingError
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    StereoMatchingError(const Eigen::MatrixXd &camera_matrix,
                        const Eigen::Vector2d &camera1_corner,
                        const Eigen::Vector2d &camera2_corner,
                        const double &w) : m_camera_matrix(camera_matrix),
                                           m_camera1_corner(camera1_corner),
                                           m_camera2_corner(camera2_corner),
                                           m_w(w) {}

    template<typename T>
    bool operator() (const T* const R_t,
                     T* residual) const {
        T rot[3*3];
        ceres::AngleAxisToRotationMatrix(R_t, rot);
        Eigen::Matrix<T, 3, 3> R = Eigen::Matrix<T, 3, 3>::Identity();
        for (int i = 0; i < 3; ++i) {
            for (int j = 0; j < 3; ++j) {
                R(i, j) = rot[i+3*j];
            }
        }
        Eigen::Map<const Eigen::Matrix<T,3,1>> t(R_t+3);
        // T rot[3*3];
        // ceres::AngleAxisToRotationMatrix(R_t, rot);
        // Eigen::Map<const Eigen::Matrix<T,3,3>> R(rot);
        // Eigen::Map<const Eigen::Matrix<T,3,1>> t(R_t+3);
        Eigen::Matrix<T, 3, 1> camera1_corner(T(m_camera1_corner(0)),T(m_camera1_corner(1)), T(1.0));
        Eigen::Matrix<T, 3, 1> camera2_corner(T(m_camera2_corner(0)),T(m_camera2_corner(1)), T(1.0));
        Eigen::Matrix<T, 3, 3> camera_matrix_inv = m_camera_matrix.inverse().template cast<T>();
        Eigen::Matrix<T, 3, 3> skew_t;
        skew_t << T(0), T(t(0)), T(-t(1)), T(-t(0)), T(0), T(t(2)), T(t(1)), T(-t(2)), T(0);
        Eigen::Matrix<T, 3, 3> F_matrix = camera_matrix_inv.transpose().template cast<T>()*skew_t*R*camera_matrix_inv;
        residual[0] = camera1_corner.transpose().dot(F_matrix*camera2_corner);
        residual[0] = m_w* residual[0];
        return true;
    }

private:
    const Eigen::Matrix3d m_camera_matrix;
    const Eigen::Vector2d m_camera1_corner;
    const Eigen::Vector2d m_camera2_corner;
    const double m_w;
};



class OptimizationLC
{
public:
    OptimizationLC(){}
    ~OptimizationLC(){}
    OptimizationLC(Eigen::VectorXd &R_t) : m_R_t(R_t) {}
    void addPointToPlaneConstraints(ceres::Problem &problem,
                                    pcl::PointCloud<pcl::PointXYZ>::Ptr &plane_pcd,
                                    Eigen::Vector3d &plane_normal,
                                    Eigen::Vector3d &plane_centroid);
    void addPointToLineConstriants(ceres::Problem &problem,
                                   std::vector<pcl::PointCloud<pcl::PointXYZ>> &lines_pcd,
                                   std::vector<Eigen::VectorXd> &line_normals);
    // ceres::Problem constructProblem();
    void addPointToPointConstriants(ceres::Problem &problem,
                                    std::vector<pcl::PointXYZ> &image_corners_3d,
                                    std::vector<pcl::PointXYZ> &lidar_corners_3d);


    Eigen::VectorXd get_R_t()
    {
        return m_R_t;
    }

private:
    Eigen::VectorXd m_R_t;
    // Eigen::Vector3d m_plane_normal;
    // Eigen::Vector3d m_plane_centroid;
    // std::vector<Eigen::Vector3d> m_line_normal;
};

class OptimizationLCC : public OptimizationLC
{
public:
    OptimizationLCC(){}
    ~OptimizationLCC(){}
    OptimizationLCC(Eigen::VectorXd &Rt_l1_c1,
                    Eigen::VectorXd &Rt_l1_c2,
                    Eigen::VectorXd &Rt_c1_c2) : m_Rt_l1_c1(Rt_l1_c1),
                                                 m_Rt_l1_c2(Rt_l1_c2),
                                                 m_Rt_c1_c2(Rt_c1_c2) {}

    OptimizationLCC(Eigen::VectorXd &Rt_c1_c2)
        :m_Rt_c1_c2(Rt_c1_c2) {}

    void addStereoMatchingConstraints(ceres::Problem &problem,
                                    std::vector<cv::Point2f> &left_image_corners,
                                    std::vector<cv::Point2f> &right_image_corners,
                                    cv::Mat& camera_matrix);

    void addPointToPlaneConstraints(ceres::Problem &problem,
                                    pcl::PointCloud<pcl::PointXYZ>::Ptr &plane_pcd,
                                    Eigen::Vector3d &plane_normal,
                                    Eigen::Vector3d &plane_centroid,
                                    Eigen::VectorXd &params);
    void addPointToLineConstriants(ceres::Problem &problem,
                                   std::vector<pcl::PointCloud<pcl::PointXYZ>> &lines_pcd,
                                   std::vector<Eigen::VectorXd> &line_normals,
                                   Eigen::VectorXd &params);
    // ceres::Problem constructProblem();
    void addPointToPointConstriants(ceres::Problem &problem,
                                    std::vector<pcl::PointXYZ> &image_corners_3d,
                                    std::vector<pcl::PointXYZ> &lidar_corners_3d,
                                    Eigen::VectorXd &params);

    Eigen::VectorXd get_Rt_l1_c1()
    {
        return m_Rt_l1_c1;
    }
    Eigen::VectorXd get_Rt_l1_c2()
    {
        return m_Rt_l1_c2;
    }
    Eigen::VectorXd get_Rt_c1_c2()
    {
        return m_Rt_c1_c2;
    }
private:
    Eigen::VectorXd m_Rt_l1_c1;
    Eigen::VectorXd m_Rt_l1_c2;
    Eigen::VectorXd m_Rt_c1_c2;
    // Eigen::Vector3d m_plane_normal_l1_c1, m_plane_normal_l1_c2;
    // Eigen::Vector3d m_plane_centroid_l1_c1, m_plane_centroid_l1_c2;
    // vector<Eigen::Vector3d> m_line_normal_l1_c1, m_line_normal_l1_c2;  
};

#endif