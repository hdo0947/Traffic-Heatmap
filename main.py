import os
import argparse
import numpy as np

import gpf
import utils.point_cloud_utils as b2p
import sys
import pcl
import open3d as o3d
import time
import utils.point_cloud_utils


def ground_segmentation(input_filename: str, out_ground_filename, out_not_ground_filename):
    bin = False
    if input_filename.endswith(".bin"):
        bin = True
        uniname = str(int(time.time())) + ".pcd"
        utils.point_cloud_utils.bin2pcdfile(input_filename, uniname)
    else:
        uniname = input_filename
    pcl_pcd = pcl.load(uniname)
    o3d_pcd = o3d.io.read_point_cloud(uniname)
    g = gpf.GroudPlaneFit()
    ground, notground = g.mainLOop(pcl_pcd, o3d_pcd)
    o3d.io.write_point_cloud(out_ground_filename, ground)
    # os.system("touch " + out_ground_filename)
    # os.system("touch " + out_not_ground_filename)
    o3d.io.write_point_cloud(out_not_ground_filename, notground)
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
    # print("infile is:", infile)
    # print("ogfile is:", ogfile)
    # print("ongfile is:", ongfile)
    # ground_segmentation("data/0000000002.bin", "out/0000000002_ground.pcd", "out/0000000002_notground.pcd")
    ground_segmentation(infile, ogfile, ongfile)



if __name__ == '__main__':
    main()
