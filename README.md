# DC-Loc (Doppler compensated Radar Metric Localization)
 This repo is an implementation for our ICRA2022 paper [DC-Loc: Accurate Automotive Radar Based Metric Localization with Explicit Doppler Compensation](https://arxiv.org/abs/2112.14887)
```
@article{gao2021accurate,
  title={Accurate Automotive Radar Based Metric Localization with Explicit Doppler Compensation},
  author={Gao, Pengen and Zhang, Shengkai and Wang, Wei and Lu, Chris Xiaoxuan},
  booktitle={2022 IEEE International Conference on Robotics and Automation (ICRA)},
  year={2022},
  organization={IEEE}  
}
```
## :octocat: Metric localization performance 
![demo](demo.gif)


# :boat: Build instructions 
Dependencies:
```
Eigen 3.3
OpenCV 3.4
Ceres 1.14
Sophus 1.0
```
Note: this code has been build and tested on Ubuntu 18.04, but it should compile for newer versions of Ubuntu.

## 📥 Dataset 
You may need to download nuScenes dataset (radar) from [nutonomy/nuscenes-devkit](https://github.com/nutonomy/nuscenes-devkit).

## :taxi: Examples 

1. preprocess [Nuscenes dataset](https://www.nuscenes.org/) to get loop closure cases and compensate for doppler effect in each radar submap.
```bash
cd ./preprocess
python search_for_loopclosure.py -n PATH_TO_NUSCENES
python list_seq.py
```
2. metric localization process
```bash
cd ~/DCRML
mkdir build
cd ./build
cmake ../DCRML
make
./DCRML
```

# 📺 video demo 
 [Youtube demo](https://www.youtube.com/watch?v=FHUWfai00HM&ab_channel=PengenGao)
