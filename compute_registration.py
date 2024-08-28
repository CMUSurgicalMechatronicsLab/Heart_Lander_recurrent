import numpy as np
import open3d as o3d
import matplotlib.pyplot as plt


def create_pcd(array_from_file):
    points_array = np.empty([len(array_from_file), 3])

    i = 0
    for k in array_from_file:
        vector = [float(value) for value in k.split(',')]
        points_array[i] = vector
        i += 1

    sweep_data = points_array.astype(np.float)
    suction_bases_data = sweep_data[-3:, :]
    sweep_data = sweep_data[:-3, :]

    source = o3d.geometry.PointCloud()
    suction_bases = o3d.geometry.PointCloud()

    source.points = o3d.utility.Vector3dVector(sweep_data)
    suction_bases.points = o3d.utility.Vector3dVector(suction_bases_data)

    source_color_array = np.tile([0, 0.702, 1], (len(source.points),1))
    source_color_array = o3d.utility.Vector3dVector(source_color_array)
    source.colors = source_color_array

    suction_bases_color_array = np.array([[0, 0, 0], [0, 0.35, 0], [0.38, 0, 0.568]])  # inferior (black), anterior (dark green), posterior (dark purple)
    suction_bases_color_array = o3d.utility.Vector3dVector(suction_bases_color_array)
    suction_bases.colors = suction_bases_color_array

    return source, suction_bases


def preprocess_pcd(pcd: o3d.geometry.PointCloud, voxel_size):
    pcd_downsample = pcd.voxel_down_sample(voxel_size)

    radius_normal = voxel_size*2
    pcd_downsample.estimate_normals(
        o3d.geometry.KDTreeSearchParamHybrid(radius=radius_normal, max_nn=20))

    radius_feature = voxel_size*2
    pcd_fpfh = o3d.pipelines.registration.compute_fpfh_feature(
        pcd_downsample,
        o3d.geometry.KDTreeSearchParamHybrid(radius=radius_feature, max_nn=40))

    return pcd_downsample, pcd_fpfh


def global_registration(source_down, target_down, source_fpfh, target_fpfh, voxel_size):
    distance_threshold = voxel_size * 1
    result = o3d.pipelines.registration.registration_ransac_based_on_feature_matching(
        source_down, target_down, source_fpfh, target_fpfh, mutual_filter=True,
        max_correspondence_distance=distance_threshold, checkers=[o3d.pipelines.registration.CorrespondenceCheckerBasedOnDistance(distance_threshold)])
    return result
# o3d.pipelines.registration.CorrespondenceCheckerBasedOnEdgeLength(1.5)


def icp_registration(source, target, threshold, trans_init):
    reg_p2p = o3d.pipelines.registration.registration_icp(
        source, target, threshold, trans_init,
        o3d.pipelines.registration.TransformationEstimationPointToPoint(),
        o3d.pipelines.registration.ICPConvergenceCriteria(max_iteration=2000))


    return reg_p2p


def rms_calc(icp_result, transformed_source, target):
    correspondence_array = np.asarray(icp_result.correspondence_set)

    tf_source_points = np.asarray(transformed_source.points)
    target_points = np.asarray(target.points)

    x_rmse_sq = 0
    y_rmse_sq = 0
    z_rmse_sq = 0

    for i in range(len(correspondence_array)):
        source_index = correspondence_array[i, 0]
        target_index = correspondence_array[i, 1]
        tf_source_point = tf_source_points[source_index]
        target_point = target_points[target_index]
        x_calc = (tf_source_point[0] - target_point[0]) ** 2
        x_rmse_sq += x_calc
        y_calc = (tf_source_point[1] - target_point[1]) ** 2
        y_rmse_sq += y_calc
        z_calc = (tf_source_point[2] - target_point[2]) ** 2
        z_rmse_sq += z_calc

    x_rms = np.sqrt(x_rmse_sq / len(correspondence_array))
    y_rms = np.sqrt(y_rmse_sq / len(correspondence_array))
    z_rms = np.sqrt(z_rmse_sq / len(correspondence_array))
    rms_tot = np.sqrt(x_rms ** 2 + y_rms ** 2 + z_rms ** 2)

    return x_rms, y_rms, z_rms, rms_tot


if __name__ == '__main__':

    # Load source and target data
    target_from_file = np.loadtxt("./Visualize_tracker_data/Registration_data/Transformed_data/target_trial5_filter_2.txt", dtype=str)
    target, suction_bases = create_pcd(target_from_file)
    # o3d.io.write_point_cloud("./Visualize_tracker_data/Registration_data/Transformed_data/target_trial1.ply", target)

    source = o3d.io.read_point_cloud("./Visualize_tracker_data/RubberHeart_edit_cropped.ply")

    # o3d.visualization.draw_geometries([source, suction_bases, target])
    # o3d.visualization.draw_geometries([suction_bases, target])

    # Global Registration from CloudCompare
    # transformation = np.array([[-0.313, -0.198, -0.929, 255.474], [-0.697, -0.616, 0.366, 5.095], [-0.645, 0.763, 0.055, -77.816], [0.0, 0.0, 0.0, 1.0]])  # For Trial 1
    # transformation = np.array([[0.002, -0.288, -0.958, 249.801], [-0.732, -0.653, 0.195, 21.559], [-0.681, 0.701, -0.212, -53.105], [0.0, 0.0, 0.0, 1.0]])  # For Trial 2
    # transformation = np.array([[0.164, -0.206, -0.965, 246.386], [-0.494, -0.864, 0.100, 20.990], [-0.854, 0.460, -0.243, -49.322], [0.0, 0.0, 0.0, 1.0]])  # For Trial 3
    transformation = np.array([[0.159, -0.119, -0.980, 251.759], [-0.466, -0.884, 0.031, 26.614], [-0.871, 0.451, -0.196, -54.509], [0.0, 0.0, 0.0, 1.0]])  # For Trial 5
    print("Global Registration Transformation from CloudCompare is:")
    print(transformation)
    print("-----------------------------------------------------------------------------------")
    # Visualize the Global Registration
    # o3d.visualization.draw_geometries([source.transform(transformation), suction_bases, target])


    # Local ICP Registration
    # i = 0.1
    # threshold_list = []
    # while i <= 5.5:
    #     threshold_list.append(i)
    #     i += 0.1
    #
    # print(threshold_list)
    #
    # trans_init = transformation
    # fitness_list = []
    # rmse_list = []
    #
    # for threshold in threshold_list:
    #     refined_result = icp_registration(source, target, threshold, trans_init)
    #     fitness_list.append(refined_result.fitness)
    #     rmse_list.append(refined_result.inlier_rmse)
    #
    # print(threshold_list)
    # print(fitness_list)
    # print(rmse_list)


    threshold = 3.2
    trans_init = transformation
    refined_result = icp_registration(source, target, threshold, trans_init)
    print("Refined Result:", refined_result)
    print("Refined Registration Transformation from Open3D is:")
    print(refined_result.transformation)
    transformed_source = source.transform(refined_result.transformation)
    # Visualize the Local ICP Registration
    o3d.visualization.draw_geometries([transformed_source, suction_bases, target])
    # o3d.visualization.draw_geometries([transformed_source, target])


    # Calculate RMS
    x_rms, y_rms, z_rms, rms_tot = rms_calc(refined_result, transformed_source, target)
    print("x_rmse =", x_rms)
    print("y_rmse =", y_rms)
    print("z_rmse =", z_rms)
    print("rmse_tot =", rms_tot)

