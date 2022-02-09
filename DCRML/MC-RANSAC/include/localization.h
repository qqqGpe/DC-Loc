//
// Created by gao on 2021/7/29.
//

#ifndef TEASER_CPP_PLY_LOCALIZATION_H
#define TEASER_CPP_PLY_LOCALIZATION_H

#include <iostream>
#include <string>
#include <fstream>
#include <chrono>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/features2d.hpp>
#include <Eigen/Core>

#include "radar_utils.hpp"
#include "features.hpp"
#include "association.hpp"
#include "registration.h"

using namespace std;
using namespace Eigen;

void Registration(const vector<Vector3d> pts_query, const vector<Vector3d> pts_train, Eigen::Matrix<double,4,4> &trans);

void load_times(string cur_gt_fpath, string next_gt_fpath,
                vector<int64_t> &query_times, vector<int64_t> &train_times);
void toronto_load_gt(char *fpath, Vector3d &gt, double &v_d);

#endif //TEASER_CPP_PLY_LOCALIZATION_H
