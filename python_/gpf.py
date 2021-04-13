import utils.point_cloud_utils as b2p
import numpy as np
import pcl
import open3d as o3d

from utils.pcl_util import npy2o3d_pcd_converter, pcl2o3d_pcd_converter


class GroudPlaneFit:
    def __init__(self):
        self.num_seg = 1
        self.num_iter = 3
        self.num_lpr = 250
        self.th_seeds = 1.2
        self.th_dist = 0.3
        self.sensor_height = 2

    def estimatePlane(self, o3d_seed_points):
        """
        in this function S, U, V are independent to pcl.PointCloud, So here it is open3d.PointCloud. And python_-pcl lib
        doesn't contain `pcl::computeMeanAndCovarianceMatrix` while open3d.PointCloud contains such a function. So we
        can use the function. If there is conflict, manually coding one is acceptable.
        """
        [points_mean, cov_matrix] = o3d_seed_points.compute_mean_and_covariance()

        U, S, V = np.linalg.svd(cov_matrix, full_matrices=True)
        normal_n = U[:, 2]
        d = -points_mean[:3] @ normal_n
        return normal_n, d

    def extractInitialSeeds(self, pcl_cloud_in: pcl.PointCloud) -> pcl.PointCloud:
        # todo: use .copy() or foreach point cloud, copy or sorted directly
        # sort on z-axis values (sort on height)

        cloud_sorted = sorted(pcl_cloud_in, key=lambda point: point[2])
        index = 0
        for point_ in cloud_sorted:
            if point_[2] < -1.5 * self.sensor_height:
                index += 1
            else:
                break
        cloud_sorted = cloud_sorted[index:]
        Np = len(cloud_sorted)
        LPR_height = 0
        for i in range(self.num_lpr):
            LPR_height += cloud_sorted[i][2]

        LPR_height = LPR_height / self.num_lpr

        seed_point_list = []
        for point in cloud_sorted:
            if point[2] < LPR_height + self.th_seeds:
                seed_point_list.append(point)
        pcl_seed_pcd = pcl.PointCloud(seed_point_list)
        return pcl_seed_pcd

    def mainLoop(self, pcl_cloud_in: pcl.PointCloud, returnType="o3d"):

        pcl_seed_pcd = self.extractInitialSeeds(pcl_cloud_in)
        for i in range(self.num_iter):
            normal_n, d = self.estimatePlane(pcl2o3d_pcd_converter(pcl_seed_pcd))
            points = np.zeros(shape=(pcl_cloud_in.size, 3))

            for n, point in enumerate(pcl_cloud_in):
                points[n, 0] = point[0]  # p.x
                points[n, 1] = point[1]  # p.y
                points[n, 2] = point[2]  # p.z
            result = points @ normal_n
            th_dist_d = self.th_dist - d
            ground_points = []
            notground_points = []
            (R)= np.shape(result)[0]
            for k in range(R):
                in_point = pcl_cloud_in[k]
                if result[k] < th_dist_d:
                    ground_points.append([in_point[0], in_point[1], in_point[2]])
                else:
                    notground_points.append([in_point[0], in_point[1], in_point[2]])
            o3d_ground_p = npy2o3d_pcd_converter(ground_points)
            o3d_notground_p = npy2o3d_pcd_converter(notground_points)
        if returnType == "o3d":
            return o3d_ground_p, o3d_notground_p
        else:
            return ground_points, notground_points

