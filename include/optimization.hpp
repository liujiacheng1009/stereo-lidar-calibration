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


// corners refinement error
class BoardCornersError
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    BoardCornersError(const pcl::PointCloud<pcl::PointXYZ> &edge_cloud, const int line_index, const vector<double>& board_length)
        : m_cloud(edge_cloud), m_index(line_index), m_length(board_length)
    {
    }
    template <typename T>
    bool operator()(const T *const corner_v, T *residual) const
    {
        if (m_cloud.empty())
        {
            return false;
        }
        T dist_sum_1 = T(0.0);
        for (size_t i = 0; i < m_cloud.size(); ++i)
        {
            pcl::PointXYZ point = m_cloud.points[i];
            T point_c[3];
            point_c[0] = T(point.x);
            point_c[1] = T(point.y);
            point_c[2] = T(point.z);
            T a[3], b[3];
            a[0] = corner_v[((m_index + 1) % 4) * 3] - corner_v[m_index * 3];
            a[1] = corner_v[((m_index + 1) % 4) * 3 + 1] - corner_v[m_index * 3 + 1];
            a[2] = corner_v[((m_index + 1) % 4) * 3 + 2] - corner_v[m_index * 3 + 2];
            b[0] = corner_v[m_index * 3] - point_c[0];
            b[1] = corner_v[m_index * 3 + 1] - point_c[1];
            b[2] = corner_v[m_index * 3 + 2] - point_c[2];
            T a_norm = ceres::sqrt(a[0] * a[0] + a[1] * a[1] + a[2] * a[2]);
            T b_norm = ceres::sqrt(b[0] * b[0] + b[1] * b[1] + b[2] * b[2]);
            T a_dot_b = a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
            T dist = b_norm * ceres::sin(ceres::acos(a_dot_b / (a_norm * b_norm)));
            dist_sum_1 += ceres::abs(dist);
        }
        dist_sum_1 /= T(m_cloud.size());

        T dist_sum_2 = T(0.0);
        T length_diagonal = ceres::sqrt(T(m_length[0])*T(m_length[0]) + T(m_length[1])*T(m_length[1]));
        for (size_t i = 0; i < 4; ++i)
        {
            T a[3], b[3];
            a[0] = corner_v[((i + 1) % 4) * 3] - corner_v[i * 3];
            a[1] = corner_v[((i + 1) % 4) * 3 + 1] - corner_v[i * 3 + 1];
            a[2] = corner_v[((i + 1) % 4) * 3 + 2] - corner_v[i * 3 + 2];
            T a_norm = ceres::sqrt(a[0] * a[0] + a[1] * a[1] + a[2] * a[2]);
            dist_sum_2 += ceres::abs(a_norm - T(m_length[i]));
            if (i < 2)
            {
                b[0] = corner_v[(i + 2) * 3] - corner_v[i * 3];
                b[1] = corner_v[(i + 2) * 3 + 1] - corner_v[i * 3 + 1];
                b[2] = corner_v[(i + 2) * 3 + 2] - corner_v[i * 3 + 2];
                T b_norm = ceres::sqrt(b[0] * b[0] + b[1] * b[1] + b[2] * b[2]);
                dist_sum_2 += ceres::abs(b_norm - length_diagonal);
            }
        }
        dist_sum_2 /= T(6.0);

        residual[0] = dist_sum_1 + dist_sum_2;
        return true;
    }

    const pcl::PointCloud<pcl::PointXYZ> m_cloud;
    const int m_index;
    const vector<double> m_length;
};

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
        Eigen::Map<const Eigen::Matrix<T,3,3>> R1(rot1);
        Eigen::Map<const Eigen::Matrix<T,3,1>> t1(Rt1+3);
        Eigen::Matrix<T,4,4> trans1 = Eigen::Matrix<T,4,4>::Identity();
        trans1.block(0,0,3,3) = R1;
        trans1.block(0,3,3,1) = t1;

        T rot2[3*3];
        ceres::AngleAxisToRotationMatrix(Rt2, rot2);
        Eigen::Map<const Eigen::Matrix<T,3,3>> R2(rot2);
        Eigen::Map<const Eigen::Matrix<T,3,1>> t2(Rt2+3);
        Eigen::Matrix<T,4,4> trans2 = Eigen::Matrix<T,4,4>::Identity();
        trans2.block(0,0,3,3) = R2;
        trans2.block(0,3,3,1) = t2;

        T rot3[3*3];
        ceres::AngleAxisToRotationMatrix(Rt3, rot3);
        Eigen::Map<const Eigen::Matrix<T,3,3>> R3(rot3);
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
    StereoMatchingError(const Eigen::Vector2d &camera1_corner,
                        const Eigen::Vector2d &camera2_corner,
                        const double &w) : m_camera1_corner(camera1_corner),
                                           m_camera2_corner(camera2_corner),
                                           m_w(w) {}

    template<typename T>
    bool operator() (const T* const R_t,
                     T* residual) const {
        // T rot[3*3];
        // ceres::AngleAxisToRotationMatrix(R_t, rot);
        // Eigen::Matrix<T, 3, 3> R = Eigen::Matrix<T, 3, 3>::Identity();
        // for (int i = 0; i < 3; ++i) {
        //     for (int j = 0; j < 3; ++j) {
        //         R(i, j) = rot[i+3*j];
        //     }
        // }
        // Eigen::Map<const Eigen::Matrix<T,3,1>> t(R_t+3);
        T rot[3*3];
        ceres::AngleAxisToRotationMatrix(R_t, rot);
        Eigen::Map<const Eigen::Matrix<T,3,3>> R(rot);
        Eigen::Map<const Eigen::Matrix<T,3,1>> t(R_t+3);
        Eigen::Matrix<T, 3, 1> camera1_corner(T(m_camera1_corner(0)),T(m_camera1_corner(1)), T(1.0));
        Eigen::Matrix<T, 3, 1> camera2_corner(T(m_camera2_corner(0)),T(m_camera2_corner(1)), T(1.0));
        Eigen::Matrix<T, 3, 3> skew_t;
        skew_t << T(0), T(t(0)), T(-t(1)), T(-t(0)), T(0), T(t(2)), T(t(1)), T(-t(2)), T(0);
        Eigen::Matrix<T, 3, 3> E_matrix = skew_t*R;
        residual[0] = camera1_corner.transpose().dot(E_matrix*camera2_corner);
        residual[0] = m_w* residual[0];
        return true;
    }

private:
    const Eigen::Vector2d m_camera1_corner;
    const Eigen::Vector2d m_camera2_corner;
    const double m_w;
};

class StereoReprojectionError
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    StereoReprojectionError(const Vector3d &P1,
                            const Vector2d &p1,
                            const Vector3d &P2,
                            const Vector2d &p2,
                            const double &w) : m_P1(P1),
                                               m_p1(p1),
                                               m_P2(P2),
                                               m_p2(p2),
                                               m_w(w) {}

    template <typename T>
    bool operator()(const T* const Rt,
                    T* residuals) const
    {
        T P1[3] = {T(m_P1(0)),
                   T(m_P1(1)),
                   T(m_P1(2))};
        T P12[3];
        ceres::AngleAxisRotatePoint(Rt, P1, P12);
        P12[0] += Rt[3];
        P12[1] += Rt[4];
        P12[2] += Rt[5];
        Eigen::Map<const Eigen::Matrix<T,3,1>> p2(P12);
        residuals[0] = m_w*(T(m_p2(0))-p2(0)/p2(2));
        residuals[1] = m_w*(T(m_p2(1))-p2(1)/p2(2));

        T rot[3*3];
        ceres::AngleAxisToRotationMatrix(Rt, rot);
        Eigen::Map<const Eigen::Matrix<T,3,3>> R(rot);
        Eigen::Map<const Eigen::Matrix<T,3,1>> t(Rt+3);
        Eigen::Matrix<T,4,4> trans = Eigen::Matrix<T,4,4>::Identity();
        trans.block(0,0,3,3) = R;
        trans.block(0,3,3,1) = t;
        Eigen::Matrix<T,4,4> trans_inv = trans.inverse();
        Eigen::Matrix<T,3,3> R_inv = trans_inv.block(0,0,3,3);
        Eigen::Matrix<T,3,1> t_inv = trans_inv.block(0,3,3,1);
        Eigen::Matrix<T,3,1> p1 = R_inv * m_P2 + t_inv;
        residuals[2] = m_w*(T(m_p1(0))-p1(0)/p1(2));
        residuals[3] = m_w*(T(m_p1(1))-p1(1)/p1(2));
        return true;
    }

private:
    const Vector3d m_P1, m_P2;
    const Vector2d m_p1, m_p2;
    const MatrixXd m_K1, m_K2;
    const double m_w;
};


class Optimizer
{
public:
    Optimizer() {}
    ~Optimizer() {}
    void addPointToPointConstriants(ceres::Problem &problem,
                                    std::vector<Vector3d> &image_corners_3d,
                                    std::vector<Vector3d> &lidar_corners_3d,
                                    VectorXd &params);
    void addStereoMatchingConstraints(ceres::Problem &problem,
                                      std::vector<Vector2d> &left_image_corners,
                                      std::vector<Vector2d> &right_image_corners,
                                      VectorXd &params);

    void addClosedLoopConstraints(ceres::Problem &problem,
                                    VectorXd& params1,
                                    VectorXd& params2,
                                    VectorXd& params3);

    void addStereoMatchingConstraints(ceres::Problem &problem,
                                      vector<Vector3d> &P1,
                                      vector<Vector2d> &p1,
                                      vector<Vector3d> &P2,
                                      vector<Vector2d> &p2,
                                      VectorXd &params);

    void addPointToPlaneConstriants(ceres::Problem &problem,
                                    pcl::PointCloud<pcl::PointXYZ> &plane_cloud,
                                    VectorXd &image_plane,
                                    VectorXd &params);
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
                                    std::vector<Vector3d> &image_corners_3d,
                                    std::vector<Vector3d> &lidar_corners_3d);


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
                                    std::vector<Vector2d> &left_image_corners,
                                    std::vector<Vector2d> &right_image_corners);

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
                                    vector<Vector3d> &image_corners_3d,
                                    vector<Vector3d> &lidar_corners_3d,
                                    VectorXd &params);

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