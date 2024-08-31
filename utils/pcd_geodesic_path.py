# import sys
import os
# import numpy as np
# import open3d as o3d
import pygeodesic.geodesic as geodesic
import vtk
from Visualize_tracker_data.vtk_helpers import *

#!/usr/bin/env python
import sys
import os

# header = "# .PCD v.7 - Point Cloud Data file format\n\
# VERSION .7\n\
# FIELDS x y z rgb\n\
# SIZE 4 4 4 1\n\
# TYPE F F F F\n\
# COUNT 1 1 1 1\n\
# WIDTH XXXX\n\
# HEIGHT 1\n\
# VIEWPOINT 0 0 0 1 0 0 0\n\
# POINTS XXXX\n\
# DATA ascii"
#
# def convertionOfPlyToPcd(ply_file, pcd_file):
#     input_file = open(ply_file)
#     out = pcd_file
#     output = open(out, 'w')
#     write_points = False
#     points_counter = 0
#     nr_points = 0
#     for s in input_file.readlines():
#         if s.find("element vertex") != -1:
#             nr_points = int(s.split(" ")[2].rstrip().lstrip())
#             new_header = header.replace("XXXX", str(nr_points))
#             output.write(new_header)
#             output.write("\n")
#         if s.find("end_header") != -1:
#             write_points = True
#             continue
#         if write_points and points_counter < nr_points:
#             points_counter = points_counter + 1
#             output.write(" ".join(s.split(" ", 4)[:4]))
#             output.write("\n")
#     input_file.close()
#     output.close()
#
#
#
# # pcd_heartscan = o3d.io.read_point_cloud("heartscan_12_5.pcd")
# # o3d.io.write_point_cloud("./heartscan_12_5.ply", pcd_hearts   can)
# # o3d.visualization.draw_geometries([pcd_heartscan])
#
#
# arm_mesh = o3d.io.read_triangle_mesh("Trial_upward_5_settle_cut.stl")
# # o3d.visualization.draw_geometries([arm_mesh])
#
# convertionOfPlyToPcd("Trial_upward_5_settle_cut.ply", "Trial_upward_5_settle_cut_TEST.pcd")
# arm_pcd = o3d.io.read_point_cloud("Trial_upward_5_settle_cut_TEST.pcd")
# # o3d.visualization.draw_geometries([arm_pcd])
# arm_pcd_pts = np.asarray(arm_pcd.points)
# # np.savetxt('Trial_upward_5_settle_cut_TEST.txt',arm_pcd_pts)


# Read the mesh to get the points and faces of the mesh
filename = "./Visualize_tracker_data/heartd435_default_cleaned_aligned.ply"
reader = vtk.vtkPLYReader()
reader.SetFileName(filename)
reader.Update()
polydata = reader.GetOutput()
points, faces = getPointsAndCellsFromPolydata(polydata)

print(len(points))
print(type(points))
np.savetxt('./Visualize_tracker_data/points_collected.txt', points)


# Initialise the PyGeodesicAlgorithmExact class instance
geoalg = geodesic.PyGeodesicAlgorithmExact(points, faces)


# Define the source and target point ids with respect to the points array
sourceIndex = 0
targetIndex = 2000


# Compute the geodesic distance and the path
distance, path = geoalg.geodesicDistance(sourceIndex, targetIndex)
# print(distance)
# print(path)


# Compute geodesic distance between source point and all other points
source_indices = np.array([sourceIndex])
distances, best_source = geoalg.geodesicDistances(source_indices)


# Create actors
polydata_actor = createPolyDataActor(polydataFromPointsAndCells(points, faces))
path_actor = createPolyLineActor(path, color=(1,1,1))
point_actors = [createSphereActor(points[indx], radius=0.001) for indx in [sourceIndex, targetIndex]]

# Add "distances" to polydata_actor to visualise distance contour from source point
result = polydata_actor.GetMapper().GetInput().GetPointData().SetScalars(nps.numpy_to_vtk(distances))
dmin = distances[np.where(distances != np.inf)].min()
dmax = distances[np.where(distances != np.inf)].max()
polydata_actor.GetMapper().SetScalarRange([dmin, dmax])

# Show VTK render window
v = Viewer()
v.addActors([polydata_actor, path_actor, *point_actors])
v.show()