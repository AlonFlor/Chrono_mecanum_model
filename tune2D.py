import sys
import numpy as np
import matplotlib.pyplot as plt
from scipy.spatial.transform import Rotation as R

import glob
import os
import pandas as pd

selected = [1, 7, 9, 11, 21, 25, 27, 33]

state_files_dir = './data/20200114_statefile/'
state_files_list = sorted(glob.glob(os.path.join(state_files_dir, '*.txt')))

idx = np.sort((np.array(selected)-1)).tolist()
sort_selected = np.sort((np.array(selected))).tolist()
print(sort_selected)

mapping = dict(zip(sort_selected, list(range(0, 16))))
state_files = []
for i in idx:
    state_files.append(state_files_list[i])
###########################################################################

def loadDF(filepath):
    with open(filepath, 'rb') as f:
        dat_gt = pd.read_csv(f, sep=" ", header=None, names=[
                             'x', 'y', 'angle'])
    return dat_gt


def loadChronoOutput(f):
    df = pd.read_csv(f)
    df['angle'] = df.apply(lambda x: quar2euler(x['q0'], x['qx'], x['qy'], x['qz']), axis=1)
    df['x'] = -df['posz'] / 100.0
    df['y'] = df['posx'] / 100.0
    tidy = df[['x', 'y', 'angle']]
    return tidy


# df.apply(lambda x: quar2euler(x['q0'], x['qx'], x['qy'], x['qz']), axis=1)
def quar2euler(w, qx, qy, qz):
    """
    Chrono: w, q1, q2, q3
    Rotation: q1, q2, q3, w
    """
    r = R.from_quat([qx, qy, qz, w])
    rz = r.as_euler('yzx', degrees=False)[0]
    return rz

def plotComparison(idx, gt_fpath, out_fpath):
    plt.cla()
    dat_gt = loadDF(gt_fpath)
    arr_gt = dat_gt.to_numpy()
    plt.plot(arr_gt[...,0],arr_gt[...,1], 'r-.', label='Ground Truth')
    dat_out = loadChronoOutput(out_fpath)
    arr_out = dat_out.to_numpy()
    plt.plot(arr_out[...,0],arr_out[...,1], label='Chrono')
    plt.legend()
    plt.axis("equal")
    pltname = "./data/output/T{}.png".format(idx)
    plt.savefig(pltname)

######### Deal with Chrono Output
output_dir = './build/Release/'
outFiles = sorted(glob.glob(os.path.join(output_dir, '*_trajectory.csv')))

# f0 = state_files[0]
# f1 = outFiles[0]
# plotComparison('7', f0, f1)

for idx, origin, out, in zip(sort_selected, state_files,outFiles):
    plotComparison(idx, origin, out)


# ######### Bullet Exmaple

# tmp_pybullet = sorted(glob.glob("./data/example_pybullet/t*.txt"))

# for i, origin, out in zip(sort_selected, state_files, tmp_pybullet):
#     plotComparison(i, origin, out)
#     plt.cla()

