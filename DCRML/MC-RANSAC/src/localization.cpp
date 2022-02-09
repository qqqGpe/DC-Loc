#include "localization.h"

void load_times(string cur_gt_fpath, string next_gt_fpath,
                vector<int64_t> &query_times, vector<int64_t> &train_times){
    for(int j = 0; j < 2; j++){
        string fpath;
        if(j == 0)
            fpath = cur_gt_fpath;
        else
            fpath = next_gt_fpath;

        ifstream fin(fpath);
        string line;
        getline(fin, line);
        for(int i = 0; i < 2; i++){
            vector<double> fields;
            getline(fin, line);
            istringstream sin(line);
            string field;
            while(getline(sin, field, ',')){
                fields.push_back(atof(field.c_str()));
            }
            if(i==0){
                query_times.push_back(fields.back());
            }
            else{
                train_times.push_back(fields.back());
            }
        }
        fin.close();
    }
}

Vector4d t_read_gt(char *fpath){
    ifstream fin(fpath);
    string line;
    getline(fin, line);
    getline(fin, line);
    getline(fin, line);
    getline(fin, line);
    vector<double> fields;
    istringstream sin(line);
    string field;
    while(getline(sin, field, ',')){
        fields.push_back(atof(field.c_str()));
    }

    Vector4d gt;
    double velocity_diff = abs(fields[7]);
    gt << fields[0], fields[1], 0.0, velocity_diff;
    fin.close();
    return gt;
}

void toronto_load_gt(char *fpath, Vector3d &gt, double &v_d){
    Vector4d t_gt = t_read_gt(fpath);
    gt << t_gt(0), t_gt(1), t_gt(2);
    v_d = t_gt(3);
}

void removeDoppler(Eigen::MatrixXd &p, Eigen::Vector3d vbar, double beta) {
    for (uint j = 0; j < p.cols(); ++j) {
        double phi = atan2f(p(1, j), p(0, j));
        double delta_r = beta * (vbar(0) * cos(phi) + vbar(1) * sin(phi));
        p(0, j) += delta_r * cos(phi);
        p(1, j) += delta_r * sin(phi);
    }
}

void removeMotionDistortion(Eigen::MatrixXd &p, std::vector<int64_t> tprime, Eigen::VectorXd wbar, int64_t t_ref) {
    for (uint j = 0; j < p.cols(); ++j) {
        double delta_t = (tprime[j] - t_ref) / 1.0e6;
        Eigen::MatrixXd T = se3ToSE3(wbar * delta_t);
        Eigen::Vector4d pbar = {p(0, j), p(1, j), 0, 1};
        pbar = T * pbar;
        p(0, j) = pbar(0);
        p(1, j) = pbar(1);
    }
}

// Rigid RANSAC
Eigen::Matrix4d computeAndGetTransform(Eigen::MatrixXd p2, Eigen::MatrixXd p1, double ransac_threshold,
    double inlier_ratio, int max_iterations){
    Ransac ransac(p2, p1, ransac_threshold, inlier_ratio, max_iterations);
    ransac.computeModel();
    Eigen::Matrix4d T;
    ransac.getTransform(T);
    return T;
}

// Assuming rotations are SO(2), this function returns the angle (yaw/heading) corresponding to the rotation
double getRotation(Eigen::MatrixXd T) {
    Eigen::MatrixXd Cmin = T.block(0, 0, 2, 2);
    Eigen::MatrixXd C = Eigen::MatrixXd::Identity(3, 3);
    C.block(0, 0, 2, 2) = Cmin;
    double eps = 1e-15;
    int i = 2, j = 1, k = 0;
    double c_y = sqrt(pow(C(i, i), 2) + pow(C(j, i), 2));
    double phi = 0;
    if (c_y > eps) {
        phi = atan2f(C(k, j), C(k, k));
    } else {
        phi = atan2f(-C(j, k), C(j, j));
    }
    return phi;
}

void pcMatching(const vector<Vector3d> pts_query, const vector<Vector3d> pts_train,
                   Eigen::MatrixXd &p1, Eigen::MatrixXd &p2){

    vector<Vector3d> pcl_q, pcl_t; //pcl_query pcl_train
    cv::Mat img_q, img_t;
    cv::Mat desc_q, desc_t;//descriptor
    vector<cv::KeyPoint> kp_q, kp_t;
    pcl_q = pts_query;
    pcl_t = pts_train;

    int patch_size = 2;
    AddKeyPoints(kp_q, pcl_q, patch_size);
    AddKeyPoints(kp_t, pcl_t, patch_size);
    SarahDescriptor(desc_q, kp_q);
    SarahDescriptor(desc_t, kp_t);

    cv::BFMatcher matcher = cv::BFMatcher();
    std::vector<std::vector<cv::DMatch>> knn_matches;
    vector<cv::DMatch> good_matches;
    matcher.knnMatch(desc_q, desc_t, knn_matches, 1);

    for (uint j = 0; j < knn_matches.size(); ++j) {
        if (!knn_matches[j].size())
            continue;
        if (knn_matches[j][0].distance < OUTLIER_THRESHOLD){
            good_matches.push_back(knn_matches[j][0]); // distance the smaller the better
        }
    }

    p1 = Eigen::MatrixXd::Zero(2, good_matches.size());
    p2 = p1;
//    std::vector<int64_t> t1prime = t1, t2prime = t2;
    for (uint j = 0; j < good_matches.size(); ++j) {
        p1(0, j) = pcl_q[good_matches[j].queryIdx].x();
        p1(1, j) = pcl_q[good_matches[j].queryIdx].y();
        p2(0, j) = pcl_t[good_matches[j].trainIdx].x();
        p2(1, j) = pcl_t[good_matches[j].trainIdx].y();
    }
}

void Registration(const vector<Vector3d> pts_query, const vector<Vector3d> pts_train,
                  Eigen::Matrix<double,4,4> &trans){
    double ransac_threshold = 0.75;
    double inlier_ratio = 0.90;
    int max_iterations = 1000;

    MatrixXd p1_cur, p2_cur;
    pcMatching(pts_query, pts_train, p1_cur, p2_cur);
    srand(time(NULL));
    Eigen::Matrix4d T2 = computeAndGetTransform(p2_cur, p1_cur, ransac_threshold, inlier_ratio, max_iterations);

    trans = T2;
}
