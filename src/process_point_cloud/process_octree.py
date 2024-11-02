import numpy as np
import open3d as o3d

from ..visualization.visualization import visualize_dynamic_points
from ..utils.utils import create_pcd
from .utils import filter_points

# Constants
SEARCH_RADIUS = 0.2
VISUALIZE = True


def filter_background_octree(pcd, background_pcd, radius=SEARCH_RADIUS,
                             min_neighbors=3, max_distance_diff=0.1,
                             visualize=VISUALIZE):
    """
    Filter background points using Octree.
    """
    # 创建octree
    octree = o3d.geometry.Octree(max_depth=8)
    octree.convert_from_point_cloud(background_pcd, size_expand=0.01)

    points = np.asarray(pcd.points)
    background_points = np.asarray(background_pcd.points)
    dynamic_points = []
    static_points = []

    # 创建KDTree作为备选方案
    background_tree = o3d.geometry.KDTreeFlann(background_pcd)

    for point in points:
        # 使用KDTree进行半径搜索
        [k, idx, dist] = background_tree.search_radius_vector_3d(point, radius)

        # 如果没有找到邻居点
        if k == 0:
            dynamic_points.append(point)
            continue

        # 获取邻居点
        background_neighbors = background_points[idx]

        # 检查邻居数量
        if k < min_neighbors:
            dynamic_points.append(point)
            continue

        # 检查距离变化
        distances = np.linalg.norm(background_neighbors - point, axis=1)
        distance_std = np.std(distances)
        if distance_std > max_distance_diff:
            dynamic_points.append(point)
            continue

        # 检查高度变化
        height_diff = np.abs(background_neighbors[:, 2] - point[2])
        if np.mean(height_diff) > 0.2:
            dynamic_points.append(point)
            continue

        static_points.append(point)

    # 创建点云对象
    dynamic_points = np.array(dynamic_points) if dynamic_points else np.zeros((0, 3))
    static_points = np.array(static_points) if static_points else np.zeros((0, 3))

    dynamic_pcd = create_pcd(dynamic_points)
    static_pcd = create_pcd(static_points)

    if visualize and len(dynamic_points) > 0 and len(static_points) > 0:
        visualize_dynamic_points(dynamic_pcd, static_pcd)

    return dynamic_pcd


# def post_process_dynamic_points(dynamic_pcd, min_cluster_size=50):
#     """
#     Post-process dynamic points using DBSCAN clustering.
#     """
#     points = np.asarray(dynamic_pcd.points)
#     if len(points) == 0:
#         return dynamic_pcd
#
#     clustering = DBSCAN(eps=0.5, min_samples=5).fit(points)
#     labels = clustering.labels_
#
#     valid_points = []
#     unique_labels = np.unique(labels)
#     for label in unique_labels:
#         if label == -1:  # Skip noise points
#             continue
#         cluster_points = points[labels == label]
#         if len(cluster_points) >= min_cluster_size:
#             valid_points.extend(cluster_points)
#
#     return create_pcd(np.array(valid_points) if valid_points else np.zeros((0, 3)))

def main(pcd, bg_pcd, trans_matrix):
    """
    Main processing pipeline using Octree for background filtering.

    Args:
        pcd: Input point cloud
        bg_pcd: Background point cloud
        trans_matrix: Transformation matrix

    Returns:
        tuple: (processed point cloud, transformed input point cloud)
    """
    # Remove background points using octree
    filtered_pcd = filter_background_octree(pcd, bg_pcd)

    # Post-process dynamic points
    # filtered_pcd = post_process_dynamic_points(filtered_pcd)

    # Apply transformation matrix
    filtered_pcd.transform(trans_matrix)
    pcd.transform(trans_matrix)

    # Filter points based on distance and height
    processed_pcd = filter_points(filtered_pcd)

    return processed_pcd, pcd