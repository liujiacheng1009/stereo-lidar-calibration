#include <Eigen/Core>
#include <ceres/ceres.h>
#include <ceres/rotation.h>     /// ceres::AngleAxisRotatePoint
#include <opencv2/core.hpp>
#include <Eigen/Geometry>
#include <ceres/autodiff_cost_function.h>

class StereoMatchingError
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    StereoMatchingError(const cv::Mat &camera_matrix,
                        const cv::Point2d &camera1_corner,
                        const cv::Point2d &camera2_corner,
                        const double &w) : m_camera_matrix(camera_matrix),
                                           m_camera1_corner(camera1_corner),
                                           m_camera2_corner(camera2_corner),
                                           m_w(w) {}
    template<typename T>
    bool operator () (const T* const R_t,
                      T* residual) const {
    {
        Eigen::Matrix<T, 3, 1> r();
        cv::Mat R,t;
        cv::Mat camera1_corner({m_camera1_corner.x,m_camera1_corner.y, 1});
        cv::Mat camera2_corner({m_camera2_corner.x,m_camera2_corner.y, 1});
        cv::Mat camera_matrix_inv = m_camera_matrix.inverse();
        cv::Mat F_matrix = camera_matrix_inv.transpose()*skew(t)*R*camera_matrix_inv;
        residual[0] = camera1_corner.transpose()*F_matrix*camera2_corner(0,0);
        residual[0] = m_w * residual[0];
    }

    cv::Mat skew(cv::Mat& t)
    {
        return cv::Mat({0, t(0, 0), -t(0, 1),
                        -t(0, 0), 0, t(0, 2),
                        t(0, 1), -t(0, 2), 0});
    }

private:
    const cv::Mat m_camera_matrix;
    const cv::Point2d m_camera1_corner, m_camera2_corner;
    const double m_w;
};

class PointToPointError
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW 
    PointToPointError(const Eigen::Vector3d& point,
                    const Eigen::Vector3d& point_cam,
                    const double& w):
                    m_point(point),
                    m_point_cam(point_cam),
                    m_w(w){}

    template<typename T>
    bool operator () (const T* const R_t,
                      T* residual) const {
        T l_pt_L[3] = {T(m_point(0)),
                T(m_point(1)),
                T(m_point(2))};
        T l_pt_C_gt = {T(m_point_cam(0)),
                T(m_point_cam(1)),
                T(m_point_cam(2))};
        T l_pt_C[3];
        ceres::AngleAxisRotatePoint(R_t, l_pt_L, l_pt_C);
        l_pt_C[0] += R_t[3];
        l_pt_C[1] += R_t[4];
        l_pt_C[2] += R_t[5];
        for(int i=0;i<3;++i){
            residual[0] += (l_pt_C_gt[i]-l_pt_C[i])*(l_pt_C_gt[i]-l_pt_C[i]);
        }
        residual[0] = m_w * residual[0];
        return true;
    }

private:
    const Eigen::Vector3d m_point;
    const Eigen::Vector3d m_point_cam;
    const double m_w;
};

// 计算点到直线的损失
class PointToLineError
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW 
    PointToLineError(const Eigen::Vector3d& point,
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
        T n_C[3] = {T(line_normal(0)),
                    T(line_normal(1)),
                    T(line_normal(2))};
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
        T r_3[3] = {T(m_normal(0)), T(m_normal(1)), T(m_normal(2))};
        T t_vec[3] = {T(m_centroid(0)), T(m_centroid(1)), T(m_centroid(2))};

        T l_pt_C[3];
        ceres::AngleAxisRotatePoint(R_t, l_pt_L, l_pt_C);
        l_pt_C[0] += R_t[3];
        l_pt_C[1] += R_t[4];
        l_pt_C[2] += R_t[5];

        Eigen::Matrix<T, 3, 1> laser_point_C(l_pt_C);
        Eigen::Matrix<T, 3, 1> r3_C(r_3);
        Eigen::Matrix<T, 3, 1> tvec_C(t_vec);
        residual[0] = r3_C.dot(laser_point_C - tvec_C);
        residual[0] = sqrt_wt_ * residual[0];
        return true;
    }
private:
    const Eigen::Vector3d m_point;
    const Eigen::Vector3d m_plane_normal;
    const Eigen::Vector3d m_plane_controid;
    const double m_w;
};

class PoseLoopClosingError
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    PoseLoopClosingError(Sophus::SE3d t_ij):m_t_ij(t_ij){}
    template <typename T>
    bool operator()(const T *const R_t_L1_C1,
                    const T *const R_t_L1_C2,
                    T *residual) const
    {
        Eigen::Map<const Sophus::SE3<T>> t_i(R_t_L1_C1);
        Eigen::Map<const Sophus::SE3<T>> t_j(R_t_L1_C2);
        Eigen::Map<Eigen::Matrix<T, 6, 1>> residual(residual_ptr);

        residual = (t_i * t_ij.template cast<T>() * t_j.inverse()).log();
        return true;
    }

private:
    Sophus::SE3d m_t_ij;
};



class OptimizationLC
{
public:
    void addPointToPlaneConstraints(ceres::Problem& problem, pcl::PointCloud<pcl::PointXYZIr>::Ptr& plane_pcd);
    void addPointToPointConstriants(ceres::Problem& problem, pcl::PointCloud<pcl::PointXYZIr>::Ptr& plane_pcd);
    void addPointToLineConstriants(ceres::Problem& problem, pcl::PointCloud<pcl::PointXYZIr>::Ptr& plane_pcd);
    ceres::Problem constructProblem();


private:
    Eigen::VectorXd(6) m_R_t;
    Eigen::Vector3d m_plane_normal;
    Eigen::Vector3d m_plane_centroid;
    vector<Eigen::Vector3d> m_line_normal;

};

class OptimizationLCC : public OptimizationLC
{
public:
    ceres::Problem constructProblem();

private:
    Eigen::VectorXd(6) m_R_t_l1_c1, m_R_t_l1_c2, m_R_t_c1_c2;
    Eigen::Vector3d m_plane_normal_l1_c1, m_plane_normal_l1_c2;
    Eigen::Vector3d m_plane_centroid_l1_c1, m_plane_centroid_l1_c2;
    vector<Eigen::Vector3d> m_line_normal_l1_c1, m_line_normal_l1_c2;  
};