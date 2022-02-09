import numpy as np
import pandas as pd
import os

root_dir = "../processed_data/"
dir_list = os.listdir(root_dir)

pairs = list()
for dir in dir_list:
    scenes = dir.split("_",1)
    scenes = [int(x) for x in scenes]
    scene_dir = "{}{}".format(root_dir, dir)
    sample_list = os.listdir(scene_dir)
    for sample_dir in sample_list:
        contents = os.listdir("{}{}/{}".format(root_dir, dir, sample_dir))
        if contents == []:
            print("{}{}/{}".format(root_dir, dir, sample_dir))
        if sample_dir == "calib":
            continue
        samples = sample_dir.split("_",1)
        samples = [int(x) for x in samples]
        pairs.append(scenes + samples)
pairs = np.array(pairs)
df = pd.DataFrame(pairs)
df.to_csv("../pair_list.csv", header=None, index=None)

