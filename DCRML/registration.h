//
// Created by gao on 2021/6/29.
//

#ifndef TEASER_CPP_PLY_REGISTRATION_H
#define TEASER_CPP_PLY_REGISTRATION_H

#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/xfeatures2d.hpp>
#include <opencv2/core/base.hpp>

#include "factors.h"
#include "utility.h"

#define USE_RCS false


bool refineRegistration(vector<Vector3d> pts_query, vector<Vector3d> pts_train,
                  Matrix4d &trans);

bool SarahDescriptor(cv::Mat &descriptors, const std::vector<cv::KeyPoint> keypoints);

bool AddKeyPoints(std::vector<cv::KeyPoint>& _kp, const vector<Vector3d> _pcl, const int patch_size);


#endif //TEASER_CPP_PLY_REGISTRATION_H
