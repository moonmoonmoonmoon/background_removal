import numpy as np

from ..visualization.visualization import visualize_dynamic_points
from ..utils.utils import create_pcd
from .utils import filter_points

# Constants
SEARCH_RADIUS = 0.2
VISUALIZE = True

def filter_background_voxel(pcd, background_pcd, voxel_size=0.1,
                            max_distance_diff=0.1, visualize=VISUALIZE):
    """
    Filter background points using voxel grid method.
    :param pcd:
    :param background_pcd:
    :param voxel_size:
    :param max_distance_diff:
    :param visualize:
    :return:
    """
    points = np.asarray(pcd.points)
    background_points = np.asarray(background_pcd.points)
    dynamic_points = []
    static_points = []
    # 创建背景点的体素网格
    background_voxels = {}
    # 将背景点放入体素
    bg_voxel_indices = np.floor(background_points / voxel_size).astype(int)
    for i, vidx in enumerate(bg_voxel_indices):
        key = tuple(vidx)
        if key not in background_voxels:
            background_voxels[key] = []
        background_voxels[key].append(background_points[i])
    # 计算每个体素的平均点
    for key in background_voxels:
        background_voxels[key] = np.mean(background_voxels[key], axis=0)
    # 检查每个输入点
    voxel_indices = np.floor(points / voxel_size).astype(int)
    for i, point in enumerate(points):
        key = tuple(voxel_indices[i])
        # 检查周围的体素
        is_dynamic = True
        for dx in [-1, 0, 1]:
            for dy in [-1, 0, 1]:
                for dz in [-1, 0, 1]:
                    neighbor_key = (key[0] + dx, key[1] + dy, key[2] + dz)
                    if neighbor_key in background_voxels:
                        bg_point = background_voxels[neighbor_key]
                        distance = np.linalg.norm(point - bg_point)
                        if distance <= max_distance_diff:
                            is_dynamic = False
                            break
                if not is_dynamic:
                    break
            if not is_dynamic:
                break
        if is_dynamic:
            dynamic_points.append(point)
        else:
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
    Main function.
    :param pcd:
    :param bg_pcd:
    :param trans_matrix:
    :return:
    """
    # Remove background points using voxel grid method
    filtered_pcd = filter_background_voxel(pcd, bg_pcd)
    # Apply transformation matrix
    filtered_pcd.transform(trans_matrix)
    pcd.transform(trans_matrix)
    # Filter points based on distance and height
    processed_pcd = filtered_pcd
    processed_pcd = filter_points(filtered_pcd)
    return processed_pcd, pcd