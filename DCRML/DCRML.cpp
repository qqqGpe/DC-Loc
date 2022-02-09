//
// Created by gao on 2020/12/28.
//
#include <iostream>
#include <string>
#include <eigen3/Eigen/Core>

#include "teaser/registration.h"
#include "utility.h"
#include "localization.h"

// Run TEASER++ registration
// Prepare solver parameters
/// left multiplication on SE3

int main(int argc, char** argv){
    string preprocessDataDir = "../processed_data/";
    string pairListPath = "../pair_list.csv";
    string pathToSaveResults = "./results/outputs.csv";
    vector<Vector4i> pairList;
    loadPairList(pairListPath, pairList);

    auto start = chrono::system_clock::now();

    ofstream ofs;
    ofs.open(pathToSaveResults, ios::out);
    ofs << "X err (m)" << "," << "Y err (m)" << "," << "V_diff (m/s)" << "," << "Rot err (rad)" << "\n";
    ofs.close();
    int n = 0;

    for(auto pair:pairList){

        char calib_dir[200], pcDir[200], gt_fpath[200];
        sprintf(calib_dir, "%s%d_%d/", preprocessDataDir.c_str(), pair(0), pair(1));
        sprintf(gt_fpath, "%s%d_%d/%d_%d/info.csv", preprocessDataDir.c_str(), pair(0), pair(1), pair(2), pair(3));
        vector<Vector3d> pc_query, pc_train;
        vector<Matrix4d> T_calibs;
        Vector3d gt_t; Quaterniond gt_q;
        Matrix4d transformation;
        double velocityDiff;

        load_gt(gt_fpath, gt_t, velocityDiff, gt_q);

        load_calibs(calib_dir, T_calibs);

        sprintf(pcDir, "%s%d_%d/%d_%d/", preprocessDataDir.c_str(), pair(0), pair(1), pair(2), pair(3));

        load_pts(pcDir, T_calibs, pc_query, pc_train);

        Registration(pc_query, pc_train, transformation);
        Vector3d est_t = transformation.block<3,1>(0,3);
        Quaterniond est_q(transformation.block<3,3>(0,0));
        Vector3d residual_err = gt_t - est_t;
        Quaterniond residual_q = est_q.inverse() * gt_q;

        ofs.open(pathToSaveResults, ios::app);
        ofs << residual_err.x() << "," << residual_err.y() << "," << velocityDiff << "," << residual_q.z() << "\n";
        ofs.close();

        cout << ++n << "/" << pairList.size()
             << ", residual err: " << residual_err(0)<< " " << residual_err(1) << " "
             << "  vd: " << velocityDiff << "  yaw:" << residual_q.z() << endl;
    }
    auto end = chrono::system_clock::now();
    auto duration = chrono::duration_cast<chrono::microseconds>(end-start);
    cout << "cost time is: " << double(duration.count()) * chrono::microseconds::period::num / chrono::microseconds::period::den << "s" << endl;

    return 0;
}
