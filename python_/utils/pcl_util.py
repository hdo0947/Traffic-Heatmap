# some pcl function is lacked in pcl-python_
# here is a utilized lib for implementing these functions
import pcl
import open3d as o3d
import numpy  as np


def pcl2o3d_pcd_converter(pcl_pointCloud: pcl.PointCloud):
    pcl_array = pcl_pointCloud.to_array()
    return npy2o3d_pcd_converter(pcl_array)


def npy2o3d_pcd_converter(ground_points):
    o3d_ground_p = o3d.geometry.PointCloud()
    o3d_ground_p.points = o3d.utility.Vector3dVector(ground_points)
    return o3d_ground_p
