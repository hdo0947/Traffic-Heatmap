import os
import argparse
import numpy as np
import gpf
import pcl
import open3d as o3d
import time

from utils import point_cloud_utils, pcl_util
from utils.pcl_util import npy2o3d_pcd_converter

RADIUS_INTERVAL = 20
DEGREE_INTERVAL = np.pi / 8


def ground_segmentation(input_filename: str, out_ground_filename, out_not_ground_filename):
    bin = False
    if input_filename.endswith(".bin"):
        bin = True
        uniname = str(int(time.time())) + ".pcd"
        point_cloud_utils.bin2pcdfile(input_filename, uniname)
    else:
        uniname = input_filename
    pcl_pcd = pcl.load(uniname)
    g = gpf.GroudPlaneFit()

    ground, notground = g.mainLoop(pcl_pcd)
    o3d.io.write_point_cloud(out_ground_filename, ground)
    o3d.io.write_point_cloud(out_not_ground_filename, notground)
    if bin:
        os.remove(uniname)


def split_poiontCloud(pcl_pcd):
    x_min = min(pcl_pcd, key=lambda x: x[0])[0]
    x_max = max(pcl_pcd, key=lambda x: x[0])[0]
    y_min = min(pcl_pcd, key=lambda x: x[1])[1]
    y_max = max(pcl_pcd, key=lambda x: x[1])[1]

    delta_x = x_max - x_min
    delta_y = y_max - y_min
    X_INTERVALS = [(x_min, x_min + delta_x / 3), (x_min + delta_x / 3, x_min + delta_x * 2 / 3),
                   (x_min + delta_x * (2 / 3), x_min + delta_x)]
    Y_INTERVALS = [(y_min, y_min + delta_y / 3), (y_min + delta_y / 3, y_min + delta_y * 2 / 3),
                   (y_min + delta_y * (2 / 3), y_min + delta_y)]
    sections = {}
    for xid in range(1, 4):
        for yid in range(1, 4):
            sections[str(xid) + str(yid)] = []
    for point in pcl_pcd:
        x = point[0]
        y = point[1]
        for x_interval_id, x_interval in enumerate(X_INTERVALS):
            if x_interval[0] <= x <= x_interval[1]:
                xid = x_interval_id + 1
        for y_interval_id, y_interval in enumerate(Y_INTERVALS):
            if y_interval[0] <= y <= y_interval[1]:
                yid = y_interval_id + 1
        sections[str(xid) + str(yid)].append([point[0], point[1], point[2]])
    for xid in range(1, 4):
        for yid in range(1, 4):
            if len(sections[str(xid) + str(yid)]) < 250:
                continue
            sections[str(xid) + str(yid)] = np.array(sections[str(xid) + str(yid)])
            # print(sections[str(xid) + str(yid)].shape)

            sections[str(xid) + str(yid)] = pcl_util.npy2o3d_pcd_converter(sections[str(xid) + str(yid)])
            point_cloud_utils.write_pcd(sections[str(xid) + str(yid)], str(xid) + str(yid) + ".pcd")



def ground_segmentation_grid(input_filename: str, out_ground_filename, out_not_ground_filename):
    bin = False
    if input_filename.endswith(".bin"):
        bin = True
        uniname = str(int(time.time())) + ".pcd"
        point_cloud_utils.bin2pcdfile(input_filename, uniname)
    else:
        uniname = input_filename
    pcl_pcd = pcl.load(uniname)
    split_poiontCloud(pcl_pcd)
    ground_l = []
    notground_l = []
    for x_i in range(1, 4):
        for y_i in range(1, 4):
            if not os.path.exists(str(x_i) + str(y_i) + ".pcd"):
                continue

            else:
                fname = str(x_i) + str(y_i) + ".pcd"
                pcl_pcd = pcl.load(fname)
                g = gpf.GroudPlaneFit()
                ground, notground = g.mainLoop(pcl_pcd, "numpy")
                if len(ground) != 0:
                    ground_l.append(ground)
                if len(notground) != 0:
                    notground_l.append(notground)

    ground_np = np.concatenate(ground_l)
    notground_np = np.concatenate(notground_l)
    ground_o3d = npy2o3d_pcd_converter(ground_np)
    notground_o3d = npy2o3d_pcd_converter(notground_np)
    o3d.io.write_point_cloud(out_ground_filename, ground_o3d)
    o3d.io.write_point_cloud(out_not_ground_filename, notground_o3d)
    for x_i in range(1, 4):
        for y_i in range(1, 4):
            if os.path.exists(str(x_i) + str(y_i) + ".pcd"):
                os.remove(str(x_i) + str(y_i) + ".pcd")
    if bin:
        os.remove(uniname)


def main():
    parser = argparse.ArgumentParser(description="group segmentation")
    parser.add_argument("--input_filename", type=str, help="input pcd filename, acceptable extensions: .pcd, .bin",
                        dest="infile")
    parser.add_argument("--output_ground_filename", type=str, help="output pcd filename which contains points that is "
                                                                   "ground. Format: .pcd", dest="ogfile")
    parser.add_argument("--output_notground_filename", type=str, help="output pcd filename which contains points that "
                                                                      "is not ground. Format: .pcd", dest='ongfile')
    result = parser.parse_args()
    infile = result.infile
    ogfile = result.ogfile
    ongfile = result.ongfile

    ground_segmentation_grid(infile, ogfile, ongfile)




if __name__ == '__main__':
    main()
