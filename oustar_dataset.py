import json
from mayavi import mlab
import os
import numpy as np
import pcl

from KittiUtils import LabelObject, BBox3D, Coordinates

OUSTAR_DATASET_ROOT = './oustar/parking ouster 1/parking ouster2'

class OustarDataset:
    def __init__(self, root = OUSTAR_DATASET_ROOT):
        self.root = root

        labels_path = os.path.join(self.root, "ann")
        pointclouds_path = os.path.join(self.root, "pointcloud_bin")

        self.labels_paths = sorted(os.listdir(labels_path))
        self.pointclouds_paths = sorted(os.listdir(pointclouds_path))

        self.labels_paths = [os.path.join(labels_path, path) for path in self.labels_paths]
        self.pointclouds_paths = [os.path.join(pointclouds_path, path) for path in self.pointclouds_paths]

    def __getitem__(self, index):
        '''
            Get Sample of the dataset by the givin index, Operator overloading of [] operator
            Arguments:
                index: index of the sample in the dataset
            Return:
                Pointcloud numpy array of shape (N, 4) and LabelObject array contains the boxes
        '''
        path_pointcloud = self.pointclouds_paths[index]
        path_labels = self.labels_paths[index]
        pointcloud = self.load_bin_pointcloud(path_pointcloud)
        labels = self.load_json_labels(path_labels)
        oustar_labels = self.parse_labels(labels)

        return pointcloud, oustar_labels

    def __len__(self):
        return len(self.pointclouds_paths)

    def parse_labels(self, labels_dict):
        '''
            Parse labels dictionary from json file to extract 3d boxes in LIDAR coord.

            Arguments:
                labels_dict: python dict contains the labels json file
            Return:
                List of Oustar Objects of type (LabelObject) contains label & 3D box in Lidar coord.
        '''
        oustar_objects = []

        objects_list = labels_dict['objects']
        object_key_to_class = {}
        for obj in objects_list:
            obj_class = obj['classTitle']
            # used to map the box to its class
            obj_key = obj['key']
            object_key_to_class[obj_key] = obj_class

        boxes_dicts = labels_dict['figures']
        for box_dict in boxes_dicts:
            geometry = box_dict['geometry']
            classKey = box_dict['objectKey']

            x = geometry['position']['x']
            y = geometry['position']['y']
            z = geometry['position']['z']
            rotation = geometry['rotation']['y']
            w = geometry['dimensions']['x']
            l = geometry['dimensions']['y']
            h = geometry['dimensions']['z']
            print(x,y,z,w,h,l, rotation)
            box = BBox3D(x,y,z, h,w,l, rotation)
            box.coordinates = Coordinates.LIDAR
            class_name = object_key_to_class[classKey]

            # if class_name == 'car' or class_name =='parking slots 2':
            class_name = 'Car'

            oustar_object = LabelObject(box, class_name)

            oustar_objects.append(oustar_object)

        return oustar_objects

    def load_json_labels(self, path):
        '''
            Load labels.json file contains labels of oustar
            Arguments:
                path: full path to the json file
            Return:
                Python Dict contains the json readings
        '''
        with open(path) as json_file:
            labels_json = json.load(json_file)
        return labels_json

    def load_bin_pointcloud(self, path):
        '''
            Load .bin Format Pointcloud

            Arguments:
                path: full path of the .bin file
            Return:
                np.array of shape (N, 4) for the XYZI pointcloud
        '''
        pointcloud = np.fromfile(path, dtype=np.float32)
        pointcloud = pointcloud.reshape((-1, 4))
        return pointcloud

    def load_pcd_pointcloud(self, path):
        '''
            Load PCD Format Pointcloud

            Arguments:
                path: full path of the pcd file
            Return:
                np.array of shape (N, 4) for the XYZI pointcloud
        '''
        # import open3d as o3d
        # from pypcd import pypcd
        # from pyntcloud import PyntCloud
        # =============== Open3D ===================
        # open3d_cloud = o3d.geometry.PointCloud()
        # open3d_cloud.points = o3d.utility.Vector3dVector(pointcloud[:,:3])
        # ================ Pyntcloud ==================
        # pointcloud = PyntCloud.from_file(unorganized_path)
        # # pointcloud = PyntCloud.from_file(self.pointclouds_paths[0])
        # pointcloud = np.array([pointcloud.points['x'],
        #                        pointcloud.points['y'],
        #                        pointcloud.points['z'],
        #                        pointcloud.points['intensity']
        #                           ], dtype=np.float32)
        # ================ pypcd ================
        # pointcloud = pypcd.PointCloud.from_path(self.pointclouds_paths[3])
        # pointcloud = pypcd.PointCloud.from_path(unorganized_path)
        # pointcloud = np.array([pointcloud.pc_data['x'],
        #                        pointcloud.pc_data['y'],
        #                        pointcloud.pc_data['z'],
        #                        pointcloud.pc_data['intensity']
        #                           ], dtype=np.float32)
        # pointcloud = pointcloud.transpose(1,0)
        # ================ PCL ================
        pointcloud = pcl.load_XYZI(path)

        # pointcloud = pcl.load_XYZI(unorganized_path)
        pointcloud = np.asarray(pointcloud, dtype=np.float32)
        return pointcloud

if __name__ == '__main__':
    import argparse
    parser = argparse.ArgumentParser()
    # parser.add_argument("dataset", type=str, default='./oustar_dataset')
    # parser.parse_args()

    OustarDataset()
