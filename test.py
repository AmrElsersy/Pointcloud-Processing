import numpy as np
import sys
sys.path.insert(0, '../')
from visualization import Visualizer
import argparse

parser = argparse.ArgumentParser()
parser.add_argument('path', type=str, default='../kitti/dataset/training/pointclouds/000000.bin')
args = parser.parse_args()
path = args.path

pointcloud = np.fromfile(path, dtype=np.float32).reshape(-1, 4)

print(pointcloud.shape)
print(pointcloud)

import pandas as pd
import matplotlib.pyplot as plt
df = pd.DataFrame(pointcloud)
print(df.describe())
df.hist(bins=50)
plt.show()

# visualize pointcloud
from mayavi import mlab
figure = mlab.figure(bgcolor=(0,0,0), fgcolor=(1,1,1), size=(1280, 720))
mlab.points3d(pointcloud[:,0], pointcloud[:,1], pointcloud[:,2], pointcloud[:,3],
        color=None, mode="point",  figure=figure)
mlab.show(stop=True)
