//
// Created by gao on 2021/6/29.
//

#ifndef TEASER_CPP_PLY_FACTORS_H
#define TEASER_CPP_PLY_FACTORS_H

#include "utility.h"
#include <ceres/ceres.h>

class SE3Parameterization:public ceres::LocalParameterization{
public:
    virtual bool Plus(const double*x, const double*delta, double*x_plus_delta) const{
        Vector6d lie;
        Vector6d delta_lie;
        lie << x[0], x[1], x[2], x[3], x[4], x[5];
        delta_lie << delta[0], delta[1], delta[2], delta[3], delta[4], delta[5];
        Sophus::SE3d T = Sophus::SE3d::exp(lie);
        Sophus::SE3d delta_T = Sophus::SE3d::exp(delta_lie);
        Vector6d x_plus_delta_lie = (delta_T * T).log();
        for(int i=0; i < 6; i++){
            x_plus_delta[i] = x_plus_delta_lie.matrix()(i,0);
        }
        return true;
    }
    virtual bool ComputeJacobian(const double*x, double*jacobian)const{
        Eigen::Map<Matrix<double,6,6,Eigen::RowMajor>> j(jacobian);
        j = Matrix<double,6,6>::Identity();
//        ceres::MatrixRef(jacobian,6,6) = ceres::Matrix::Identity(6,6);
        return true;
    }
    virtual int GlobalSize() const{return 6;}
    virtual int LocalSize() const{return 6;}
};


class SinglePointCostFunction:public ceres::SizedCostFunction<3,6>{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
    SinglePointCostFunction(const Vector3d p1, const Vector3d p2):point_1(p1), point_2(p2){}

    ~SinglePointCostFunction(){}

    virtual bool Evaluate(double const*const* parameters, double*residuals, double**jacobians)const override{

        Vector6d lie(parameters[0]);
        Sophus::SE3d T = Sophus::SE3d::exp(lie);

        double sigma_r = 0.1, sigma_phi = 0.3 * M_PI / 180;
        double r1, phi_1, r2, phi_2;
        r1 = point_1.norm();
        r2 = point_2.norm();
        phi_1 = point_1.y() > 0? atan2(point_1.y(), point_1.x()) : atan2(point_1.y(), point_1.x()) + 2*M_PI;
        phi_2 = point_2.y() > 0? atan2(point_2.y(), point_2.x()) : atan2(point_2.y(), point_2.x()) + 2*M_PI;

        double C00, C01, C10, C11;
        Matrix3d Cov1, Cov2, sqrt_info;
        C00 = sigma_r*sigma_r*cos(phi_1)*cos(phi_1) + r1*r1*sin(phi_1)*sin(phi_1)*sigma_phi*sigma_phi;
        C01 = -r1*r1*sin(phi_1)*cos(phi_1)*sigma_phi*sigma_phi + sigma_r*sigma_r*sin(phi_1)*cos(phi_1);
        C10 = -r1*r1*sin(phi_1)*cos(phi_1)*sigma_phi*sigma_phi + sigma_r*sigma_r*sin(phi_1)*cos(phi_1);
        C11 = sigma_r*sigma_r*sin(phi_1)*sin(phi_1) + r1*r1*cos(phi_1)*cos(phi_1)*sigma_phi*sigma_phi;
        Cov1 << C00, C01, 0,
                C10, C11, 0,
                0,   0,   1;

        C00 = sigma_r*sigma_r*cos(phi_2)*cos(phi_2) + r2*r2*sin(phi_2)*sin(phi_2)*sigma_phi*sigma_phi;
        C01 = -r2*r2*sin(phi_2)*cos(phi_2)*sigma_phi*sigma_phi + sigma_r*sigma_r*sin(phi_2)*cos(phi_2);
        C10 = -r2*r2*sin(phi_2)*cos(phi_2)*sigma_phi*sigma_phi + sigma_r*sigma_r*sin(phi_2)*cos(phi_2);
        C11 = sigma_r*sigma_r*sin(phi_2)*sin(phi_2) + r2*r2*cos(phi_2)*cos(phi_2)*sigma_phi*sigma_phi;
        Cov2 << C00, C01, 0,
                C10, C11, 0,
                0,   0,   1;

        Matrix2d R = T.matrix().block<2,2>(0,0);
        Matrix2d C_tmp;
        C_tmp << C00, C01, C10, C11;
        C_tmp = R * C_tmp * R.transpose();
        Cov2 = Matrix3d::Identity();
        Cov2.block<2,2>(0,0) = C_tmp;

        Matrix3d Cov = Cov1 + Cov2;
        sqrt_info = LLT<Matrix3d>(Cov.inverse()).matrixL().transpose();
        Eigen::Map<Vector3d> residual(residuals);
        residual = sqrt_info * (point_1 - T * point_2);
//        residual = point_1 - T * point_2;

        if(jacobians){
            if(jacobians[0]){
                Eigen::Map<Eigen::Matrix<double, 3, 6, Eigen::RowMajor>> jacobian_0(jacobians[0]);
                jacobian_0.block<3,3>(0,0) = -Eigen::Matrix3d::Identity();
                jacobian_0.block<3,3>(0,3) = Sophus::SO3d::hat(T * point_2);
                jacobian_0 = sqrt_info * jacobian_0;
            }
        }
        return true;
    }
private:
    Eigen::Vector3d point_1, point_2;
};

#endif //TEASER_CPP_PLY_FACTORS_H
