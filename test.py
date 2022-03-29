import numpy as np
import sys
sys.path.insert(0, '../')
from visualization import Visualizer

path = 'oustar/pointclouds/1645619652.853595401.bin'

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
