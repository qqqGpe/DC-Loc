//
// Created by gao on 2021/6/29.
//

#include "registration.h"
#include <ceres/ceres.h>


bool AddKeyPoints(std::vector<cv::KeyPoint>& _kp, const vector<Vector3d> _pcl, const int patch_size){
    double u,v;
    _kp.clear();
    for(uint i = 0; i < _pcl.size(); i++){
        u = _pcl[i].x() * SCALE + WIDTH/2;
        v = _pcl[i].y() * SCALE + HIGHT/2;
        _kp.push_back(cv::KeyPoint(u,v,patch_size));
    }
    return true;
}

bool SarahDescriptor(cv::Mat &descriptors, const std::vector<cv::KeyPoint> keypoints){

    uint M = AZIMUTH_RANGE;
//  float azimuth_step = (2 * M_PI) / float(M);
    uint max_range = MAX_RANGE;
    uint range;
//  float azimuth;
    cv::Mat desc1 = cv::Mat::zeros(keypoints.size(), max_range, CV_32F);
    cv::Mat desc2 = cv::Mat::zeros(keypoints.size(), M, CV_32F);
    vector<Eigen::Vector2i> imgKeyPoints;

    for(auto kp:keypoints){
        Eigen::Vector2i p;
        p.x() = kp.pt.x;
        p.y() = kp.pt.y;
        imgKeyPoints.push_back(p);
    }

    for(auto kp:keypoints){
        for(uint k = 0; k < keypoints.size(); k++){
            range = sqrt(pow(kp.pt.y -imgKeyPoints[k].y(), 2) + pow(kp.pt.x -imgKeyPoints[k].x(), 2));
            if(range > max_range)
                continue;
            else{
                desc1.at<float>(k, range)++;
            }
        }
    }

    for(uint i = 0; i < keypoints.size(); i++){
        cv::normalize(desc1.row(i), desc1.row(i), 0, 1, cv::NORM_MINMAX);
//        cv::normalize(desc1.row(i), desc1.row(i), 1, cv::NORM_L1);
    }

    desc1.copyTo(descriptors);
//  desc2.copyTo(descriptors);
    return true;
}

bool refineRegistration(const vector<Vector3d> pts_query, const vector<Vector3d> pts_train,
                  Matrix4d &trans){

    vector<Vector3d> pcl_q, pcl_t; //pcl_query pcl_train
    pcl_q = pts_query;
    pcl_t = pts_train;

    double lie[6] = {0,0,0,0,0,0};
    ceres::LocalParameterization* local_param = new SE3Parameterization();
    ceres::Problem problem;
    problem.AddParameterBlock(lie, 6, local_param);

    for (size_t i = 0; i < pcl_t.size(); i++){
        ceres::LossFunction *lossFunction = new ceres::HuberLoss(1.0);
        ceres::CostFunction *costFunction = new SinglePointCostFunction(pcl_t[i], pcl_q[i]);
        problem.AddResidualBlock(costFunction, lossFunction, lie);
    }

    ceres::Solver::Options options;
    options.linear_solver_type = ceres::DENSE_QR;
    options.trust_region_strategy_type = ceres::LEVENBERG_MARQUARDT;
    options.max_num_iterations = 100;
    ceres::Solver::Summary summary;
    ceres::Solve(options, &problem, &summary);

    Vector6d lie_vec;
    lie_vec << lie[0], lie[1], lie[2], lie[3], lie[4], lie[5];
    trans = Sophus::SE3d::exp(lie_vec).matrix();

    return true;
}