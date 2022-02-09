//
// Created by gao on 2021/6/29.
//

#ifndef TEASER_CPP_PLY_UTILITY_H
#define TEASER_CPP_PLY_UTILITY_H

#include <chrono>
#include <dirent.h>
#include <fstream>
#include <Eigen/Core>
#include <sophus/se3.hpp>

#define HIGHT 720
#define WIDTH 960
#define SCALE 3.0

#define MAX_RANGE 400
#define AZIMUTH_RANGE 360
#define OUTLIER_THRESHOLD 5.0

using namespace std;
using namespace Eigen;
typedef Matrix<double, 6, 1> Vector6d;

void read_pcl(string path, vector<Vector3d> &pc);
Eigen::Matrix<double,4,4> read_calib(string path);
void load_calibs(string dir, vector<Matrix4d> &calibs);
void load_pts(char *dir, vector<Matrix4d> T_calibs,
              vector<Vector3d> &pcl_query, vector<Vector3d> &pcl_train);
void load_gt(char *dir, Vector3d &gt, double &v_d, Quaterniond &q);
void loadPairList(string fpath, vector<Vector4i> &pairs);


#endif //TEASER_CPP_PLY_UTILITY_H
