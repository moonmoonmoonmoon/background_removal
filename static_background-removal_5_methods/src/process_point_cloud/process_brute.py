import numpy as np

from ..visualization.visualization import visualize_dynamic_points
from ..utils.utils import create_pcd

# Constants
MIN_DISTANCE = 1.5
GROUND_LEVEL = 0.0
MAX_HEIGHT = 3.0
SEARCH_RADIUS = 0.2
VISUALIZE = True

def filter_background_brute(pcd, background_pcd, radius=SEARCH_RADIUS,
                            min_neighbors=3, max_distance_diff=0.1,
                            visualize=VISUALIZE):
    """
    Filter background points using brute force method.
    :param pcd:
    :param background_pcd:
    :param radius:
    :param min_neighbors:
    :param max_distance_diff:
    :param visualize:
    :return:
    """
    points = np.asarray(pcd.points)
    background_points = np.asarray(background_pcd.points)
    dynamic_points = []
    static_points = []
    for point in points:
        # 计算当前点到所有背景点的距离
        distances = np.linalg.norm(background_points - point, axis=1)
        # 找到半径范围内的邻居
        neighbor_mask = distances <= radius
        neighbors = background_points[neighbor_mask]
        # 如果邻居数量不够
        if len(neighbors) < min_neighbors:
            dynamic_points.append(point)
            continue
        # 检查距离变化
        distance_std = np.std(distances[neighbor_mask])
        if distance_std > max_distance_diff:
            dynamic_points.append(point)
            continue
        # 检查高度变化
        height_diff = np.abs(neighbors[:, 2] - point[2])
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

def main(pcd, bg_pcd, trans_matrix):
    """
    Main function for processing point cloud.
    :param pcd:
    :param bg_pcd:
    :param trans_matrix:
    :return:
    """
    # Remove background points using brute force method
    filtered_pcd = filter_background_brute(pcd, bg_pcd)
    # Apply transformation matrix
    filtered_pcd.transform(trans_matrix)
    pcd.transform(trans_matrix)
    # Filter points based on distance and height
    processed_pcd = filter_points(filtered_pcd)

    return processed_pcd, pcd