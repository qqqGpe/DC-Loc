//
// Created by gao on 2021/6/30.
//

#include "utility.h"


void read_pcl(string path, vector<Vector3d> &pc){
    pc.clear();
    ifstream fin(path);
    string line;
    getline(fin, line);
    while(getline(fin, line)){
        istringstream sin(line);
        Eigen::Vector3d point;
        vector<double> fields;
        string field;
        while(getline(sin, field, ',')){
            fields.push_back(atof(field.c_str()));
        }
        Vector3d p;
        Vector3d vel;
        double rcs;
        p << fields[0],fields[1],0.0;
        vel << fields[10], fields[11],0.0;
        pc.push_back(p);
    }
    fin.close();
}


Eigen::Matrix<double,4,4> read_calib(string path){
    Eigen::Matrix<double,4,4> calib_trans;
    ifstream fin(path);
    string line;
    vector<double> fields;
    getline(fin, line);
    while(getline(fin, line)){
        istringstream sin(line);
        Eigen::Vector3d point;
        string field;
        while(getline(sin, field, ',')){
            fields.push_back(atof(field.c_str()));
        }
    }
    try{
        for(int i = 0; i < 16; i++){
            calib_trans(i/4, i%4) = fields[i];
        }
    }
    catch(exception &e){
        cout << e.what() << endl;
    }
    fin.close();
    return calib_trans;
}

void read_gt(char *fpath, Vector4d &gt_t_vd, Quaterniond &q_b1_b0){
    ifstream fin(fpath);
    string line;
    vector<double> fields;
    string field;
    getline(fin, line);
    // get q_w_b_0
    getline(fin, line);
    istringstream sin(line);
    while(getline(sin, field, ',')){
        fields.push_back(atof(field.c_str()));
    }
    Quaterniond qwb_0(fields[3], fields[4], fields[5], fields[6]);
    fields.clear();

    // get q_w_b_1
    getline(fin, line);
    istringstream sin1(line);
    while(getline(sin1, field, ',')){
        fields.push_back(atof(field.c_str()));
    }
    Quaterniond qwb_1(fields[3], fields[4], fields[5], fields[6]);
    fields.clear();
    q_b1_b0 = qwb_1.inverse() * qwb_0;

    // get t_b1_b0
    getline(fin, line);
    istringstream sin2(line);
    while(getline(sin2, field, ',')){
        fields.push_back(atof(field.c_str()));
    }
    double velocity_diff = abs(fields[7]);
    gt_t_vd << fields[0], fields[1], 0.0, velocity_diff;
    fin.close();
}

void loadPairList(string fpath, vector<Vector4i> &pairs){
    ifstream fin(fpath);
    string line;
    while(getline(fin, line)){
        istringstream sin(line);
        vector<int> fields;
        string field;
        while(getline(sin, field, ',')){
            fields.push_back(atof(field.c_str()));
        }
        pairs.emplace_back(fields[0],fields[1],fields[2],fields[3]);
    }
    fin.close();
}


void load_calibs(string dir, vector<Matrix4d> &calibs){
    string calib_dir = dir + "calib/";
    vector<string> chans = {"RADAR_FRONT", "RADAR_FRONT_LEFT", "RADAR_FRONT_RIGHT",
                            "RADAR_BACK_LEFT", "RADAR_BACK_RIGHT"};

    for(size_t i = 0; i < chans.size(); i++){
        char fpath[300];
        sprintf(fpath, "%s%s/calib_1_%s.csv", calib_dir.c_str(), chans[i].c_str(), chans[i].c_str());
        Matrix4d calib = read_calib(fpath);
        calibs.push_back(calib);
    }
}

void load_gt(char *fpath, Vector3d &gt, double &v_d, Quaterniond &q){
//    cout << "fpath is: " << fpath << endl;
    Vector4d gt_t_vd;
    Quaterniond q_b1_b0;
    read_gt(fpath, gt_t_vd, q_b1_b0);
    q = q_b1_b0;
    gt << gt_t_vd(0), gt_t_vd(1), gt_t_vd(2);
    v_d = gt_t_vd(3);
}

void load_pts(char *dir, vector<Matrix4d> T_calibs,
              vector<Vector3d> &pcl_query, vector<Vector3d> &pcl_train){

    vector<string> chans = {"RADAR_FRONT", "RADAR_FRONT_LEFT", "RADAR_FRONT_RIGHT",
                            "RADAR_BACK_LEFT", "RADAR_BACK_RIGHT"};
    vector<Vector3d> pc_1, pc_2;
    for(size_t i = 0; i < chans.size(); i++){
        char fpath[200];
        vector<Vector3d> cur_pc;
        vector<double> cur_rcs;
        sprintf(fpath, "%s%s/pcl_1.csv", dir, chans[i].c_str(), chans[i].c_str());
//        cout << "fpath is: " << fpath << endl;
        read_pcl(fpath, cur_pc);
        for(size_t j=0; j < cur_pc.size(); j++)
            cur_pc[j] = Sophus::SE3d(T_calibs[i]) * cur_pc[j];
        pc_1.insert(pc_1.end(), cur_pc.begin(), cur_pc.end());

        sprintf(fpath, "%s%s/pcl_2.csv", dir, chans[i].c_str(), chans[i].c_str());
        read_pcl(fpath, cur_pc);
        for(size_t j=0; j < cur_pc.size(); j++)
            cur_pc[j] = Sophus::SE3d(T_calibs[i]) * cur_pc[j];
        pc_2.insert(pc_2.end(), cur_pc.begin(), cur_pc.end());
    }

    for(size_t i = 0; i < pc_1.size(); i++)
        pc_1[i] = Sophus::SE3d(T_calibs[0]).inverse() * pc_1[i];
    for(size_t i = 0; i < pc_2.size(); i++)
        pc_2[i] = Sophus::SE3d(T_calibs[0]).inverse() * pc_2[i];

    pcl_query = pc_2;
    pcl_train = pc_1;
}