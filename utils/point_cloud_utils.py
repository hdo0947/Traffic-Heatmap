import open3d as o3d


import numpy as np
import struct
import sys
import open3d as o3d
import pcl


def bin_to_pcd(binFileName):
    size_float = 4
    list_pcd = []
    with open(binFileName, "rb") as f:
        byte = f.read(size_float * 4)
        while byte:
            x, y, z, intensity = struct.unpack("ffff", byte)
            list_pcd.append([x, y, z])
            byte = f.read(size_float * 4)
    np_pcd = np.asarray(list_pcd)
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(np_pcd)
    return pcd

def write_pcd(pcd, pcdFileName):
    o3d.io.write_point_cloud(pcdFileName, pcd)

def bin2pcdfile(bin_filename, pcd_filename):
    write_pcd(bin_to_pcd(bin_filename), pcd_filename)




if __name__ == '__main__':
    bin2pcdfile("../sync/data/0000000002.bin", "../0000000002.pcd")