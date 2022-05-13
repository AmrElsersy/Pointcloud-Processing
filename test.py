import numpy as np
import sys
sys.path.insert(0, '../')
from visualization import Visualizer
import argparse

parser = argparse.ArgumentParser()
parser.add_argument('--path', type=str, default='../kitti/dataset/training/pointclouds/000000.bin')
args = parser.parse_args()
path = args.path

pointcloud = np.fromfile(path, dtype=np.float32).reshape(-1, 4)

print(pointcloud.shape)
print(pointcloud)

import pandas as pd
import matplotlib.pyplot as plt

def filter_range(pc):
        factor = 10
        for i in range(4):
                min_i = pc[:, i].mean() - factor * pc[:, i].std()
                max_i = pc[:, i].mean() + factor * pc[:, i].std()
                mask = np.where((pc[:, i] > min_i) & (pc[:, i] < max_i))[0]
                print(pc.shape, mask)
                pc = pc[mask]
        return pc
# pointcloud = filter_range(pointcloud)

df = pd.DataFrame(pointcloud)
print(df.describe())
df.hist(bins=100)
plt.show()

# visualize pointcloud
from mayavi import mlab
figure = mlab.figure(bgcolor=(0,0,0), fgcolor=(1,1,1), size=(1280, 720))
mlab.points3d(pointcloud[:,0], pointcloud[:,1], pointcloud[:,2], pointcloud[:,3],
        color=None, mode="point",  figure=figure)
mlab.show(stop=True)
