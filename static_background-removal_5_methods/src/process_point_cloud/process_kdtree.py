# process_point_cloud.py

import numpy as np
import open3d as o3d

from ..visualization.visualization import visualize_dynamic_points
from ..utils.utils import create_pcd
from .utils import filter_points

# Constants
SEARCH_RADIUS = 0.2
VISUALIZE = True

def filter_background_kd(pcd, background_pcd, radius=SEARCH_RADIUS,min_neighbors=3, max_distance_diff=0.1,visualize=VISUALIZE):
    """
    Filter background points using Cartesian coordinates and KD tree.
    :param pcd:
    :param background_pcd:
    :param radius:
    :param min_neighbors:
    :param max_distance_diff:
    :param visualize:
    :return:
    """
    background_tree = o3d.geometry.KDTreeFlann(background_pcd)
    points = np.asarray(pcd.points)
    dynamic_points = []
    static_points = []
    for point in points:
        # Search for points within a radius
        [k, idx, dist] = background_tree.search_radius_vector_3d(point, radius)
        # If no background points are found
        if k == 0:
            dynamic_points.append(point)
        else:
            background_neighbors = np.asarray(background_pcd.points)[idx]
            # Check the number of neighbors
            if k < min_neighbors:
                dynamic_points.append(point)
                continue
            # Check the distance variance
            distances = np.linalg.norm(background_neighbors - point, axis=1)
            distance_std = np.std(distances)
            if distance_std > max_distance_diff:
                dynamic_points.append(point)
                continue
            # Check the height variance
            height_diff = np.abs(background_neighbors[:, 2] - point[2])
            if np.mean(height_diff) > 0.2:  # 0.2 meters
                dynamic_points.append(point)
                continue
            static_points.append(point)
    # Create dynamic and static point clouds
    dynamic_points = np.array(dynamic_points)
    static_points = np.array(static_points)
    dynamic_pcd = create_pcd(dynamic_points)
    static_pcd = create_pcd(static_points)
    if visualize and len(dynamic_points) > 0 and len(static_points) > 0:
        visualize_dynamic_points(dynamic_pcd, static_pcd)
    return dynamic_pcd

# def post_process_dynamic_points(dynamic_pcd, min_cluster_size=50):
#     """
#     Post-process dynamic points using DBSCAN clustering.
#     :param dynamic_pcd:
#     :param min_cluster_size:
#     :return:
#     """
#     # Use DBSCAN to cluster dynamic points
#     points = np.asarray(dynamic_pcd.points)
#     if len(points) == 0:
#         return dynamic_pcd
#     clustering = DBSCAN(eps=0.5, min_samples=5).fit(points)
#     labels = clustering.labels_
#     # Only keep clusters with a minimum number of points
#     valid_points = []
#     unique_labels = np.unique(labels)
#     for label in unique_labels:
#         if label == -1:  # Skip noise points
#             continue
#         cluster_points = points[labels == label]
#         if len(cluster_points) >= min_cluster_size:
#             valid_points.extend(cluster_points)
#     return create_pcd(np.array(valid_points))

def main(pcd, bg_pcd, trans_matrix):
    """
    main function
    :param pcd:
    :param bg_pcd:
    :param trans_matrix:
    :return:
    """
    # Remove background points
    filtered_pcd = filter_background_kd(pcd, bg_pcd)
    # Post-process dynamic points
    # filtered_pcd = post_process_dynamic_points(filtered_pcd)
    # Apply transformation matrix
    filtered_pcd.transform(trans_matrix)
    pcd.transform(trans_matrix)
    # Filter points based on distance and height
    processed_pcd = filter_points(filtered_pcd)
    return processed_pcd, pcd


