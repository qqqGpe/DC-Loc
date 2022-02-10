# DCRML (Doppler compensated Radar Metric Localization)
 This repo is an implementation for [Accurate Automotive Radar Based Metric Localization with Explicit Doppler Compensation](https://arxiv.org/abs/2112.14887), which will be presented in ICRA'2022 !
```
@article{gao2021accurate,
  title={Accurate Automotive Radar Based Metric Localization with Explicit Doppler Compensation},
  author={Gao, Pengen and Zhang, Shengkai and Wang, Wei and Lu, Chris Xiaoxuan},
  journal={arXiv preprint arXiv:2112.14887},
  year={2021}
}
```
# Metric localization performance
![demo](demo.gif)


# Build instructions
Dependencies:
```
Eigen 3.3
OpenCV 3.4
Ceres 1.14
Sophus 1.0
```
Note: this code has been build and tested on Ubuntu 18.04, but it should compile for newer versions of Ubuntu.

## Examples

1. preprocess [Nuscenes dataset](https://www.nuscenes.org/) to get loop closure cases and compensate for doppler effect in each radar submap.
```
python search_for_loopclosure.py PATH_TO_NUSCENES
python list_seq.py
```
2. metric localization process
```
cd ~/DCRML
mkdir build
cd ./build
cmake ../DCRML
make
./DCRML
```

# video demo
📺 [Youtube demo](https://www.youtube.com/watch?v=DUsr0B203ZQ)
