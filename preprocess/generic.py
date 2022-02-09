# -*- coding: utf-8 -*-
"""
general operation

"""
import os.path as osp
import numpy as np
from nuscenes import NuScenes
from pyquaternion import Quaternion
from data_classes import RadarPointCloud
from matplotlib import pyplot as plt


class Generic(object):
    def __init__(self, version, dataset):
        self.dataset = dataset
        self.nusc = NuScenes(version = version, dataroot = dataset, verbose=True)
        self.scene = self.nusc.scene[0]
        self.sample_token = self.scene['first_sample_token']
        self.sample = self.nusc.get('sample', self.sample_token)
        self.channel = 'RADAR_FRONT'
        self.sample_data = self.nusc.get('sample_data', self.sample['data'][self.channel])
        self.cs = self.nusc.get('calibrated_sensor', self.sample_data['calibrated_sensor_token'])
        ego_pose_record = self.nusc.get('ego_pose', self.sample_data['ego_pose_token'])
        self.sample_abs_ego_pose = np.array(ego_pose_record['translation'])
        self.sceneNum = 0
        self.frameNum = -1
        
    def to_scene(self, i):
        self.scene = self.nusc.scene[i]
        self.sample_token = self.scene['first_sample_token']
        self.sample = self.nusc.get('sample', self.sample_token)
        self.sceneNum = i
    
    def to_next_sample(self):
        self.sample_token = self.sample['next']
        if self.sample_token != '':
            self.sample = self.nusc.get('sample', self.sample_token)
        else:
            self.sample = self.nusc.get('sample', self.scene['last_sample_token'])

    def to_sample(self, sample_num):
        self.sample_token = self.scene['first_sample_token']
        self.sample = self.nusc.get('sample', self.sample_token)
        for _ in range(sample_num):
            self.to_next_sample()

    def get_timestamp(self):
        return self.sample_data['timestamp'] / 1e6

    def get_sample_data(self, channel='RADAR_FRONT'):
        self.sample_data = self.nusc.get('sample_data', self.sample['data'][channel])
        return self.sample_data
    
    def get_sample_abs_ego_pose(self, channel='RADAR_FRONT'):
        self.sample_data = self.nusc.get('sample_data', self.sample['data'][channel])
        ego_pose_record = self.nusc.get('ego_pose', self.sample_data['ego_pose_token'])
        self.sample_abs_ego_pose = np.array(ego_pose_record['translation'])
        q_w_b_sample = ego_pose_record['rotation']
        q_w_b = Quaternion(q_w_b_sample[0], q_w_b_sample[1], q_w_b_sample[2], q_w_b_sample[3]) # (w,x,y,z)
        return q_w_b, self.sample_abs_ego_pose

    def get_calib_data(self, channel='RADAR_FRONT'):
        sample_data = self.nusc.get('sample_data', self.sample['data'][channel])
        self.cs = self.nusc.get('calibrated_sensor', sample_data['calibrated_sensor_token'])
        rotation = self.cs['rotation']
        b_t_s = np.array(self.cs['translation'])
        q_b_s = Quaternion(rotation[0], rotation[1], rotation[2], rotation[3])  # (w, x, y, z)
        return q_b_s, b_t_s
    
    def get_pcl(self, channel):
        self.sample_data = self.nusc.get('sample_data', self.sample['data'][channel])
        pcl_file = osp.join(self.dataset, self.sample_data['filename'])
        pcl = RadarPointCloud.from_file(pcl_file)
        pcl = pcl.points[:3, :].transpose()
        return pcl
    
    def get_pcl_pano(self, ref_chan = 'RADAR_FRONT', comp=False):
        chans = ['RADAR_BACK_LEFT', 'RADAR_BACK_RIGHT', 'RADAR_FRONT', 'RADAR_FRONT_LEFT', 'RADAR_FRONT_RIGHT']
        pcl_all_ = np.zeros((0, 3))
        for chan in chans:
            pc, times = RadarPointCloud.from_file_multisweep(self.nusc, self.sample, chan, ref_chan, nsweeps=8, comp=comp)
            pt = pc.points[:3, :].transpose()
            pcl_all_ = np.concatenate((pcl_all_, pt), axis=0)
        return pcl_all_

    # return all points in each radar frames, and return a dict with the form radar_frame: x,y,z,vx_comp,vy_comp
    def get_separate_pcl_pano(self, ex_dyn=False, compensate_doppler=False):
        chans = ['RADAR_BACK_LEFT', 'RADAR_BACK_RIGHT', 'RADAR_FRONT', 'RADAR_FRONT_LEFT', 'RADAR_FRONT_RIGHT']
        separate_pcl_all = dict()
        for chan in chans:
            pcl_cur_ = np.zeros((0, 8))
            pc, times = RadarPointCloud.from_file_multisweep(self.nusc, self.sample, chan, chan, nsweeps=10, comp=compensate_doppler)
            pt = pc.points[[0,1,2,5,6,7,8,9], :]
            if ex_dyn:
                pt = pt[:, (pc.points[3,:]==1).tolist()]
            pcl_cur_ = np.concatenate((pcl_cur_, pt.transpose()), axis=0)
            separate_pcl_all[chan] = pcl_cur_
        return separate_pcl_all


    def get_velocity(self, channel='RADAR_FRONT'):
        _, translation_cur = self.get_sample_abs_ego_pose(channel)
        timestamp_cur = self.sample['timestamp']

        if self.sample['next'] != '':
            sample_token = self.sample['next']
        else:
            sample_token = self.sample['prev']

        sample_t = self.nusc.get('sample', sample_token)
        sample_data_t = self.nusc.get('sample_data', sample_t['data'][channel])
        ego_pose_t_token = sample_data_t['ego_pose_token']
        ego_pose_t = self.nusc.get('ego_pose', ego_pose_t_token)
        translation_t = ego_pose_t['translation']
        timestamp_t = ego_pose_t['timestamp']

        if self.sample['next'] != '':
            v_cur = np.array((translation_t - translation_cur) / abs(timestamp_t - timestamp_cur) * 1e6)
        else:
            v_cur = np.array((translation_cur - translation_t) / abs(timestamp_t - timestamp_cur) * 1e6)

        return v_cur

    @staticmethod
    def save_fig(pcl, dirRoot, append:str, showFig=True):
        plt.ion()
        plt.clf()
        plt.xlim((-150, 150))
        plt.ylim((-150, 150))
        x = list()
        y = list()
        for i in range(pcl.shape[0]):
            x.append(pcl[i][0])
            y.append(pcl[i][1])
        plt.scatter(x,y,s=5.)
        if showFig:
            plt.plot()
        plt.pause(0.1)
        plt.savefig(dirRoot + "fig_" + append + ".jpg")
       # v = pptk.viewer(pcl, point_size=10)
       # v.set(point_size=0.5)
