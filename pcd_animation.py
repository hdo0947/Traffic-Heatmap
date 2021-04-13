import re
import os
import open3d as o3d
vis = o3d.visualization.Visualizer()
vis.create_window()
geometry = o3d.geometry.PointCloud()
vis.add_geometry(geometry)
is_ground = False
if is_ground is True:
    pattern = "^(\d){10}ground.pcd$"
else:
    pattern = "^(\d){10}notground.pcd$"
files = []
for out_file in os.listdir("out"):
    if re.match(pattern, out_file) is None:
        continue
    files.append(os.path.join("out", out_file))
for sorted_file in sorted(files):
    pcd = o3d.io.read_point_cloud(sorted_file)
    geometry.points = pcd.points
    vis.add_geometry(geometry)
    vis.poll_events()
    vis.update_renderer()



#
#
# # geometry is the point cloud used in your animaiton
# geometry = o3d.geometry.PointCloud()
# vis.add_geometry(geometry)
#
# for i in range(icp_iteration):
#     # now modify the points of your geometry
#     # you can use whatever method suits you best, this is just an example
#     geometry.points = pcd_list[i].points
#     vis.update_geometry(geometry)
#     vis.poll_events()
#     vis.update_renderer()
