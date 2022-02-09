import tqdm
from generic import *
import numpy as np
import pandas as pd
import shutil
import os

threshold = 5.0

class Radar_info(object):
    def __init__(self):
        self.point_cloud_info = list()
        self.calib_info = list()
        self.local_radar_velocity = list()
        self.v_abs = list()
        self.timestamp = list()
        self.translation = 0.0
        self.rotation = Quaternion()
        self.origin_velocity = list()
        self.radar_refs = ['RADAR_BACK_LEFT', 'RADAR_BACK_RIGHT', 'RADAR_FRONT', 'RADAR_FRONT_LEFT', 'RADAR_FRONT_RIGHT']
        self.radial_velocity = list()
        self.t_ego = []
        self.q_ego = []

    def get_origin_velocity(self):
        for i in range(len(self.point_cloud_info)):
            v_origin_dict = dict()
            for ref in self.radar_refs:
                v_origin = np.zeros((0, 2))
                v_radar = self.local_radar_velocity[i][ref][:-1]
                for j in range(self.point_cloud_info[i][ref].shape[0]):
                    position_uni = -self.point_cloud_info[i][ref][j, :2]/np.linalg.norm(self.point_cloud_info[i][ref][j, :2])
                    v_comp = self.point_cloud_info[i][ref][j, -2:]
                    v_origin_cur = -v_radar.dot(position_uni) * position_uni + v_comp
                    v_origin_cur = v_origin_cur.dot(position_uni) * position_uni
                    v_origin_cur.reshape((-1, 2))
                    v_origin = np.vstack((v_origin, v_origin_cur))
                v_origin_dict[ref] = v_origin
            self.origin_velocity.append(v_origin_dict)

    def get_radial_velocity(self):
        for i in range(len(self.point_cloud_info)):
            v_radial_dict = dict()
            for ref in self.radar_refs:
                v_radial = np.zeros((0, 2))
                for j in range(self.point_cloud_info[i][ref].shape[0]):
                    loc = self.point_cloud_info[i][ref][j,:2]
                    raw_vel = self.point_cloud_info[i][ref][j,4:6]
                    loc_normalized = loc / np.sqrt(np.sum(loc**2))
                    vr_cur = loc_normalized.dot(raw_vel) * loc_normalized
                    v_radial = np.vstack((v_radial, vr_cur))
                v_radial_dict[ref] = v_radial
            self.radial_velocity.append(v_radial_dict)

    def save_data(self, scenePair:list, sample_pair:list):
        data_path = data_dir + str(sample_pair[0]) + "_" + str(sample_pair[1]) + "/"
        if not osp.exists(data_path):
            os.makedirs(data_path)
        for radar_id in self.radar_refs:
            pcl_path = ''.join([data_path, radar_id, '/'])

            cur_calib_path = data_dir + "calib" + '/' + radar_id + '/'
            if not osp.exists(cur_calib_path):
                os.makedirs(cur_calib_path)
                calibFrame_1 = pd.DataFrame(self.calib_info[0][radar_id])
                calibFrame_2 = pd.DataFrame(self.calib_info[1][radar_id])
                calibFrame_1.to_csv(cur_calib_path + 'calib_1_' + radar_id + '.csv', index=False, sep=',')
                calibFrame_2.to_csv(cur_calib_path + 'calib_2_' + radar_id + '.csv', index=False, sep=',')
            if not osp.exists(pcl_path):
                os.mkdir(pcl_path)

            dataFrame_1 = pd.DataFrame({'x':self.point_cloud_info[0][radar_id][:,0], 'y':self.point_cloud_info[0][radar_id][:,1], 'z':self.point_cloud_info[0][radar_id][:,2],
                                        'rcs':self.point_cloud_info[0][radar_id][:,3],
                                        'vx':self.point_cloud_info[0][radar_id][:,4], 'vy':self.point_cloud_info[0][radar_id][:,5],
                                        'vx_comp':self.point_cloud_info[0][radar_id][:,6],'vy_comp':self.point_cloud_info[0][radar_id][:,7],
                                        'vx_full':self.origin_velocity[0][radar_id][:,0], 'vy_full':self.origin_velocity[0][radar_id][:,1],
                                        'vx_radial': self.radial_velocity[0][radar_id][:,0], 'vy_radial':self.radial_velocity[0][radar_id][:,1]})

            dataFrame_2 = pd.DataFrame({'x':self.point_cloud_info[1][radar_id][:,0], 'y':self.point_cloud_info[1][radar_id][:,1], 'z':self.point_cloud_info[1][radar_id][:,2],
                                        'rcs':self.point_cloud_info[1][radar_id][:,3],
                                        'vx':self.point_cloud_info[1][radar_id][:,4], 'vy':self.point_cloud_info[1][radar_id][:,5],
                                        'vx_comp':self.point_cloud_info[1][radar_id][:,6], 'vy_comp':self.point_cloud_info[1][radar_id][:,7],
                                        'vx_full':self.origin_velocity[1][radar_id][:,0], 'vy_full':self.origin_velocity[1][radar_id][:,1],
                                        'vx_radial': self.radial_velocity[1][radar_id][:,0], 'vy_radial':self.radial_velocity[1][radar_id][:,1]})

            dataFrame_2.to_csv(pcl_path + 'pcl_2' + '.csv', index=False, sep=',')
            dataFrame_1.to_csv(pcl_path + 'pcl_1' + '.csv', index=False, sep=',')

        df3 = pd.DataFrame({'x':[self.t_ego[0][0], self.t_ego[1][0], self.translation[0]],
                            'y':[self.t_ego[0][1], self.t_ego[1][1], self.translation[1]],
                            'z':[self.t_ego[0][2], self.t_ego[1][2], 0],
                            'w':[self.q_ego[0][0], self.q_ego[1][0], 0],
                            'qx':[self.q_ego[0][1], self.q_ego[1][1], 0],
                            'qy':[self.q_ego[0][2], self.q_ego[1][2], 0],
                            'qz':[self.q_ego[0][3], self.q_ego[1][3], 0],
                            'v':[self.v_abs[0], self.v_abs[1], abs(self.v_abs[0] - self.v_abs[1])],
                            'scene':[scenePair[0], scenePair[1], 0],
                            'timestamp':[self.timestamp[0], self.timestamp[1], 0]
                            })
        df3.to_csv(data_path + "info.csv", index=False, sep=',')


#%% search for specific sample pairs given specific scene pairs
def searchForLoopClosureSamplePairs(val, _scenePair):
    nbr_samples_list = list()
    for scene in _scenePair:
        val.to_scene(scene)
        nbr_samples_list.append(val.scene['nbr_samples'])

    nbr_samples = min(nbr_samples_list)
    egoPoseMatrix = np.zeros((nbr_samples-3,4,0))

    for scene in _scenePair:
        val.to_scene(scene)
        egoPoseArray = np.zeros((0, 3))
        index = np.zeros((0, 1))
        for i in range(nbr_samples):
            if i in [0, 1, 2]:
                val.to_next_sample()
                continue
            val.get_sample_data('RADAR_FRONT')
            _,egoPoseCur = val.get_sample_abs_ego_pose('RADAR_FRONT')
            egoPoseArray = np.vstack((egoPoseArray, egoPoseCur.reshape((1,3))))
            index = np.vstack((index, np.array([i])))
            val.to_next_sample()
        egoPoseArray = np.hstack((index, egoPoseArray))
        egoPoseArray = egoPoseArray[:,:,np.newaxis]
        egoPoseMatrix = np.concatenate((egoPoseMatrix, egoPoseArray), axis=2)

#%% use knn to search closure loop pairs
    from sklearn.neighbors import NearestNeighbors
    closureList = []

    for i in range(egoPoseMatrix.shape[0]):
        neigh = NearestNeighbors(n_neighbors=1)
        neigh.fit(egoPoseMatrix[:,1:,1].reshape(-1,3))
        dis,index = neigh.kneighbors(egoPoseMatrix[i,1:,0].reshape(-1,3), 1, return_distance = True)
        index = np.squeeze(index)
        if dis < threshold:
            closureList.append([egoPoseMatrix[i,0,0], egoPoseMatrix[index,0,1]])

    for j in range(egoPoseMatrix.shape[0]):
        neigh = NearestNeighbors(n_neighbors=1)
        neigh.fit(egoPoseMatrix[:,1:,0].reshape(-1,3))
        dis,index = neigh.kneighbors(egoPoseMatrix[j,1:,1].reshape(-1,3), 1, return_distance = True)
        index = np.squeeze(index)
        if dis < threshold:
            closureList.append([egoPoseMatrix[index,0,0], egoPoseMatrix[j,0,1]])

    closureList = np.array(closureList)
    closureList = np.unique(closureList, axis=0).astype(np.int)

    if closureList.shape[0] == 0:
        shutil.rmtree(data_dir)
        return

#%% save radar information
    for k in range(closureList.shape[0]):
        sample_pair = [closureList[k,0], closureList[k,1]]

        radar_info = Radar_info()
        p_array = list()
        q_array = list()

        for i in range(len(_scenePair)):
            val.to_scene(_scenePair[i])
            val.to_sample(sample_pair[i])  # move to the specified scene and samples

            radar_info.timestamp.append(val.get_sample_data('RADAR_FRONT')['timestamp'])
            # get point cloud after compensating doppler effect ang exclude dynamic targets
            pcl_cur = val.get_separate_pcl_pano(ex_dyn=True, compensate_doppler=True)
            radar_info.point_cloud_info.append(pcl_cur)

            # get extrinsic calibration information
            calib_dict_cur = dict()
            for ref in radar_info.radar_refs:
                q_calib,t_calib = val.get_calib_data(ref)
                T_calib_cur = np.identity(4)
                T_calib_cur[:3,:3] = q_calib.rotation_matrix
                T_calib_cur[:3,-1] = t_calib
                calib_dict_cur[ref] = T_calib_cur
            radar_info.calib_info.append(calib_dict_cur)

            # get velocity in radar coordinate system
            v_dict_cur = dict()
            for ref in radar_info.radar_refs:
                sensor_vel = val.get_velocity(ref)  # calculate ego-velocity
                q_ego_cur,_ = val.get_sample_abs_ego_pose(ref)
                v_dict_cur[ref] = np.linalg.inv(calib_dict_cur[ref][:3,:3]).dot(q_ego_cur.conjugate.rotation_matrix.dot(sensor_vel)) # 将速度转化到当前雷达坐标系下得到local_velocity
            radar_info.local_radar_velocity.append(v_dict_cur)

            # get ground truth absolute velocity, translation, and rotation information
            q_ego, p_ego = val.get_sample_abs_ego_pose('RADAR_FRONT')
            v = np.linalg.norm(val.get_velocity('RADAR_FRONT'))
            p_array.append(p_ego)
            q_array.append(q_ego)
            radar_info.t_ego.append(p_ego)
            radar_info.q_ego.append(q_ego)
            radar_info.v_abs.append(v)

        # get each point's full velocities
        radar_info.get_origin_velocity()
        radar_info.get_radial_velocity()

        trans = q_array[1].conjugate.rotation_matrix.dot(p_array[0] - p_array[1])
        radar_info.translation = trans
        radar_info.save_data(_scenePair, sample_pair)


#%% search all loop in specific scene
def searchForLoopClosure(location:str):
# create data structure
    scene_list = list()
    for i in tqdm.tqdm(range(len(val.nusc.scene))):
        val.to_scene(i)
        if val.nusc.get('log',val.scene['log_token'])['location'] != location:
            continue
        nbr_samples = val.scene['nbr_samples']
        sample_list = list()
        for j in range(nbr_samples):
            if j in[0, 1]:
                val.to_next_sample()
                continue
            sample_info = dict()
            _,sample_info['position'] = val.get_sample_abs_ego_pose()
            sample_info['position'] = sample_info['position'][:-1]
            sample_info['velocity'] = val.get_velocity(channel='RADAR_FRONT')[:-1]
            sample_info['timestamp'] = val.get_sample_data(channel = 'RADAR_FRONT')['timestamp']
            sample_info['scene_num'] = i
            sample_info['sample_num'] = j
            val.to_next_sample()
            sample_list.append(sample_info)
        scene_list.append(sample_list)
    # calculate average position and average velocity of each scene
    all_scenes_average_pos_vel = np.zeros((0,4))

    for scene in scene_list:
        scene_average_pos = np.zeros((1,2))
        scene_average_vel = np.zeros((1,2))
        for sample in scene:
            scene_average_pos += sample['position']
            scene_average_vel += sample['velocity']
        scene_average_pos /= len(scene)
        scene_average_vel /= len(scene)
        all_scenes_average_pos_vel = np.vstack((all_scenes_average_pos_vel, np.hstack((scene_average_pos, scene_average_vel))))

# search for nearest scenes
    from sklearn.neighbors import NearestNeighbors
    # dist = cdist(all_scenes_average_pos, all_scenes_average_pos)
    loop_pairs = np.zeros((0,2)).astype(np.int)
    for i in range(len(scene_list)):
        scene_pos_rest = np.delete(all_scenes_average_pos_vel, i, axis=0)

        DS_TH = 50
        knn = NearestNeighbors(n_neighbors=1)
        knn.fit(scene_pos_rest[:, :2])
        dis, index = knn.kneighbors(all_scenes_average_pos_vel[i,:2].reshape(-1,2), 3, return_distance = True)
        for j in range(index.shape[1]):
            if dis[0,j] > DS_TH:
                continue

            idx_1 = np.where(np.all(all_scenes_average_pos_vel[:,:2] == all_scenes_average_pos_vel[i,:2], axis=1))[0][0]
            idx_2 = np.where(np.all(all_scenes_average_pos_vel[:,:2] == scene_pos_rest[index[0,j],:2], axis=1))[0][0]

            idx_pair = np.array(sorted([idx_1, idx_2])).astype(np.int)

            loop_pairs = np.vstack((loop_pairs, idx_pair.reshape(1,2)))

#%% Check whether loop pairs are heading the same direction
    loop_pairs = np.unique(loop_pairs, axis=0)  # delete duplicate element

    loop_pairs_valid = list()
    for pair in loop_pairs:
        heading_vec_1 = scene_list[pair[0]][-1]['position'] - scene_list[pair[0]][0]['position']
        heading_vec_2 = scene_list[pair[1]][-1]['position'] - scene_list[pair[1]][0]['position']
        vec_1_2_head = scene_list[pair[1]][0]['position'] - scene_list[pair[0]][0]['position']
        vec_1_2_tail = scene_list[pair[1]][-1]['position'] - scene_list[pair[0]][0]['position']
        vec_2_1_head = scene_list[pair[0]][0]['position'] - scene_list[pair[1]][0]['position']
        vec_2_1_tail = scene_list[pair[0]][-1]['position'] - scene_list[pair[1]][0]['position']
        velocity_diff = np.linalg.norm(all_scenes_average_pos_vel[pair[0],2:] - all_scenes_average_pos_vel[pair[1],2:])

        if heading_vec_1.dot(heading_vec_2) < 0:
            loop_pairs_valid.append(False)
            continue
        if vec_1_2_head.dot(vec_1_2_tail) < 0 or vec_2_1_head.dot(vec_2_1_tail) < 0:
            loop_pairs_valid.append(True)
        else:
            loop_pairs_valid.append(False)

    loop_pairs_valid = loop_pairs[loop_pairs_valid, :]

    for k in range(loop_pairs_valid.shape[0]):
        loop_pairs_valid[k,0] = scene_list[loop_pairs_valid[k,0]][0]['scene_num']
        loop_pairs_valid[k,1] = scene_list[loop_pairs_valid[k,1]][0]['scene_num']

    np.save('loop_pairs_'+location+'.npy', loop_pairs_valid)
    print(loop_pairs_valid)


if __name__=="__main__":
    import argparse
    parser = argparse.ArgumentParser()
    parser.add_argument("-n", "--nuscenes_path", help="path to your nuscenes dataset", type=str)
    args = parser.parse_args()

    version = 'v1.0-trainval'
    dataset_path = args.nuscenes_path
    val = Generic(version, dataset_path)
    location_list = ['singapore-hollandvillage',
                     'singapore-queenstown',
                     'singapore-onenorth',
                     'boston-seaport']

    if not osp.exists("../processed_data"):
        os.mkdir("../processed_data")

    for location in location_list:
        searchForLoopClosure(location)
        location_path = "%s%s.npy" % ("loop_pairs_", location)

        loop_pairs = np.load(location_path)
        for i in tqdm.tqdm(range(loop_pairs.shape[0])):
            scenePair = [loop_pairs[i][0], loop_pairs[i][1]]
            append = "%d_%d" % (scenePair[0], scenePair[1])
            data_dir = "%s%s/" % ("../processed_data/", append)
            if not osp.exists(data_dir):
                os.mkdir(data_dir)
            searchForLoopClosureSamplePairs(val, scenePair)

    print("finished!")
