#include "optimization.hpp"
#include <bits/stdc++.h>
using namespace Eigen;
using namespace std;

void generate_simulation_points(std::vector<Eigen::Vector3d>& p1, std::vector<Eigen::Vector3d>& p2,int n)
{
    p1.clear();
    p2.clear();
    std::vector<Eigen::Vector3d> modelP;
    double a = 1,b = 2.,c = 3.;//椭球参数
    for(auto i = -1.4;i<=1.4;i+=0.28){
        for(auto j= -3.1;j<=3.1;j+=0.62){
            double x = a*cos(i)*cos(j);
            double y = b*cos(i)*sin(j);
            double z = c*sin(i);
            modelP.push_back(Vector3d(x,y,z));
        }
    }
    
    Vector3d angles = {0.5,0.5,0.5};
    Vector3d translation = {0.5,0.5,0.5};
    std::default_random_engine dre;
    std::normal_distribution<double> noise(0,0.001);
    vector<Vector3d> modelT;
    Matrix3d R;
    R = AngleAxisd(angles(0), Vector3d::UnitX())
        * AngleAxisd(angles(1),  Vector3d::UnitY())
        * AngleAxisd(angles(2), Vector3d::UnitZ());
    // cout<< "original rotation: \n"<< R <<endl;
    // cout<< "original translation: "<< translation[0]<<" "<<translation[1]<<" "<<translation[2]<<endl;
    for(int i=0;i<modelP.size();i++){
        Vector3d xyz = R*modelP[i] + translation;
        modelT.push_back(xyz);
    }
    if(n>modelP.size()){
        n = modelP.size();
    }
    int m = modelP.size();
    int k = m/(n-1);
    for(int i=0;i<n*k;i+=k){
        p1.push_back(modelP[i]);
        p2.push_back(modelT[i]);
    }
    return;
}



int main()
{
    Eigen::VectorXd R_t(6);
    R_t << 0.4, 0.4, 0.4, 0.4, 0.4, 0.1;
    OptimizationLC optimizer_lc(R_t);
    ceres::Problem problem;
    // problem.AddParameterBlock(R_t.data(), 6);
    std::vector<Eigen::Vector3d> p1, p2;
    generate_simulation_points(p1, p2, 4);
    // for(auto& p:p1){
    //     std::cout<<p<<std::endl;
    // }
    // for(auto& p:p2){
    //     std::cout<<p<<std::endl;
    // }
    // optimizer_lc.addPointToPointConstriants(problem, p1, p2);
    // ceres::Solver::Options options;
    // options.max_num_iterations = 200;
    // options.linear_solver_type = ceres::SPARSE_NORMAL_CHOLESKY;
    // options.minimizer_progress_to_stdout = false;
    // ceres::Solver::Summary summary;
    // ceres::Solve(options, &problem, &summary);
    // std::cout << summary.BriefReport() << "\n";
    // cout << optimizer_lc.get_R_t() << std::endl;
    return 0;
}