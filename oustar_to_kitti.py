import pwd
import cv2
import numpy as np
from oustar_dataset import OustarDataset
import argparse
import os
from KittiUtils import LabelObject, BBox3D
import matplotlib.pyplot as plt

def save_file(text, path, file_name):
    file_path = os.path.join(path, file_name+'.txt')

    # check if save directory exists
    if not os.path.exists(path):
        os.makedirs(path)

    file = open(file_path, 'w')
    ret = file.write(text)
    print(ret)


def save_pointcloud_kitti_format(pointcloud:np.array, path, file_name):
    if not os.path.exists(path):
        os.makedirs(path)

    file_path = os.path.join(path, file_name+'.bin')
    file = open(file_path, 'w')
    pointcloud.tofile(file_path)
    # file.write(pointcloud.tobytes())

def save_labels_kitti_format(labels, path, file_name):
    labels_str = ''
    for label in labels:
        class_name = label.label
        box = label.bbox_3d

        label_str = [class_name,
                     0.0, 0, 0.0, # trunsacted, occlution, observation
                     0.0 ,0.0 ,0.0 ,0.0 , # 2d image box
                     box.height, box.width, box.length,
                     box.x, box.y, box.z, box.rotation]

        label_str = [str(l) for l in label_str]
        label_str = ' '.join(label_str)
        print(label_str)
        labels_str += label_str + '\n'

    save_file(labels_str, path, file_name)

def save_calib_kitti_format(path, file_name):
    P0 = 'P0: 7.070493000000e+02 0.000000000000e+00 6.040814000000e+02 0.000000000000e+00 0.000000000000e+00 7.070493000000e+02 1.805066000000e+02 0.000000000000e+00 0.000000000000e+00 0.000000000000e+00 1.000000000000e+00 0.000000000000e+00'
    P1 = 'P1: 7.070493000000e+02 0.000000000000e+00 6.040814000000e+02 -3.797842000000e+02 0.000000000000e+00 7.070493000000e+02 1.805066000000e+02 0.000000000000e+00 0.000000000000e+00 0.000000000000e+00 1.000000000000e+00 0.000000000000e+00'
    P2 = 'P2: 7.070493000000e+02 0.000000000000e+00 6.040814000000e+02 4.575831000000e+01 0.000000000000e+00 7.070493000000e+02 1.805066000000e+02 -3.454157000000e-01 0.000000000000e+00 0.000000000000e+00 1.000000000000e+00 4.981016000000e-03'
    P3 = 'P3: 7.070493000000e+02 0.000000000000e+00 6.040814000000e+02 -3.341081000000e+02 0.000000000000e+00 7.070493000000e+02 1.805066000000e+02 2.330660000000e+00 0.000000000000e+00 0.000000000000e+00 1.000000000000e+00 3.201153000000e-03'
    R0 = 'R0_rect: 1.0 0.0 0.0 0.0 1.0 0.0 0.0 0.0 1'
    Tr = 'Tr_velo_to_cam: 1.0 0.0 0.0 0.0 0.0 1.0 0.0 0.0 0.0 0.0 1.0 0.0'
    imu ='Tr_imu_to_velo: 9.999976000000e-01 7.553071000000e-04 -2.035826000000e-03 -8.086759000000e-01 -7.854027000000e-04 9.998898000000e-01 -1.482298000000e-02 3.195559000000e-01 2.024406000000e-03 1.482454000000e-02 9.998881000000e-01 -7.997231000000e-01'

    text = [P0, P1, P2, P3, R0, Tr, imu]
    text = '\n'.join(text)
    save_file(text, path, file_name)


def save_image(images_path, name_str):
    if not os.path.exists(images_path):
        os.makedirs(images_path)

    full_path = os.path.join(images_path, name_str + '.png')
    image = np.zeros((384, 1248), dtype=np.int)
    print(full_path)
    cv2.imwrite(full_path, image)

def convert_oustar_to_kitti(args):
    oustar_path = args.oustar_path
    kitti_path = args.kitti_path

    oustar = OustarDataset(oustar_path)

    pointclouds_path = os.path.join(kitti_path, 'velodyne')
    labels_path = os.path.join(kitti_path, 'label_2')
    calibs_path = os.path.join(kitti_path, 'calib')
    images_path = os.path.join(kitti_path, 'image_2')

    name_i = 0
    for i in range(len(oustar)):
        pointcloud, labels = oustar[i]

        n_digits = len(str(name_i))
        name_str = (6-n_digits) * '0' + str(name_i)

        save_pointcloud_kitti_format(pointcloud, pointclouds_path, name_str)
        save_labels_kitti_format(labels, labels_path, name_str)
        save_calib_kitti_format(calibs_path, name_str)
        save_image(images_path, name_str)

        name_i +=1

if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--oustar_path', type=str, default='./oustar')
    parser.add_argument('--kitti_path', type=str, default='./kitti')
    args = parser.parse_args()

    convert_oustar_to_kitti(args)
