import numpy as np
import open3d as o3d

# array_from_file = np.loadtxt("./Visualize_tracker_data/ellipsoid_surface_sweep_data.txt", dtype=str)
array_from_file = np.loadtxt("./Visualize_tracker_data/Injector_Head_Registration_points.txt", dtype=str)
# array_from_file = np.loadtxt("./Visualize_tracker_data/Trial_1_reg_pts_Copy.txt", dtype=str)
array_from_file = array_from_file[:,0:3]
sweep_data = array_from_file.astype(np.float)

axis_pcd = o3d.create_mesh_coordinate_frame(size=50, origin=[0, 0, 0])

pcd = o3d.geometry.PointCloud()
pcd.points = o3d.utility.Vector3dVector(sweep_data)
o3d.io.write_point_cloud("./Visualize_tracker_data/data.ply", pcd)

# pcd = o3d.io.read_point_cloud("./Visualize_tracker_data/RubberHeart.ply")

# o3d.visualization.draw_geometries([pcd])
o3d.visualization.draw_geometries([pcd] + [axis_pcd])


# #Voxel
# downpcd = o3d.geometry.voxel_down_sample(pcd, voxel_size=0.05)
#
# #Voxel normals
# o3d.geometry.estimate_normals(
#     downpcd,
#     search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=0.1, max_nn=30)
# )
#
# #Visualize downpcd and surface normals (press "n")
# o3d.visualization.draw_geometries([downpcd])
#
# #save normals
# downpcd_normals = downpcd.normals



