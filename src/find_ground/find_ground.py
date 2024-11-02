# find_ground.py
import numpy as np
import os
from glob import glob

from ..visualization.visualization import visualize_ground
from ..utils.utils import read_pcd
import numpy as np
from scipy.spatial.transform import Rotation as R

# Constants
VOXEL_SIZE = 0.1
RANSAC_DISTANCE_THRESHOLD = 0.1
RANSAC_N = 3
RANSAC_NUM_ITERATIONS = 1000
TRANSLATION = True
VISUALIZE = True

def find_ground_plane(pcd):
    """
    Find the ground plane
    :param pcd:
    :return:
    """
    plane_model, inliers = pcd.segment_plane(RANSAC_DISTANCE_THRESHOLD, RANSAC_N, RANSAC_NUM_ITERATIONS)
    ground_points = pcd.select_by_index(inliers)
    non_ground_points = pcd.select_by_index(inliers, invert=True)
    return plane_model, ground_points, non_ground_points


import open3d as o3d
import numpy as np


def fast_find_ground_plane(pcd, z_threshold=-0.8):
    """
    根据 z 值阈值查找地面平面并分离地面和非地面点。
    :param pcd: 输入的 Open3D 点云对象
    :param z_threshold: z 值阈值，低于该值的点将被视为地面
    :return: (平面模型参数（None），地面点云，非地面点云)
    """
    # 转换为 numpy 数组以便处理
    points = np.asarray(pcd.points)

    # 使用 z 阈值区分地面点和非地面点
    ground_mask = points[:, 2] <= z_threshold
    ground_points = points[ground_mask]
    non_ground_points = points[~ground_mask]

    # 将地面和非地面点分别创建为新的点云对象
    ground_pcd = o3d.geometry.PointCloud()
    ground_pcd.points = o3d.utility.Vector3dVector(ground_points)

    non_ground_pcd = o3d.geometry.PointCloud()
    non_ground_pcd.points = o3d.utility.Vector3dVector(non_ground_points)

    # 返回的平面模型参数设为 None，因为 z 阈值方法不生成具体平面模型
    return None, ground_pcd, non_ground_pcd


def compute_translation_matrix(plane_model, translation=TRANSLATION):
    """
    Compute the transformation matrix
    :param plane_model:
    :return:
    """
    a,b,c,d = plane_model
    normal = np.array([a,b,c])
    normal /= np.linalg.norm(normal)
    target_normal = np.array([0, 0, 1])
    rotation_axis = np.cross(normal, target_normal)
    rotation_angle = np.arccos(np.clip(np.dot(normal, target_normal), -1.0, 1.0))

    k_matrix = np.array([
        [0, -rotation_axis[2], rotation_axis[1]],
        [rotation_axis[2], 0, -rotation_axis[0]],
        [-rotation_axis[1], rotation_axis[0], 0]
    ])
    rotation_matrix = np.eye(3) + np.sin(rotation_angle) * k_matrix + (1 - np.cos(rotation_angle)) * np.dot(k_matrix, k_matrix)
    transform_matrix = np.eye(4)
    transform_matrix[:3, :3] = rotation_matrix
    # Compute translation vector if required
    if translation:
        # Assuming the plane equation is ax + by + cz + d = 0
        translation_vector = np.array([0, 0, d/c])
        transform_matrix = np.eye(4)
        transform_matrix[:3, 3] = translation_vector
    return transform_matrix

def compute_transformation_matrix(plane_model, translation=True):
    """
    Compute the transformation matrix to align a plane's normal with the z-axis.

    :param plane_model: List or array with plane coefficients [a, b, c, d] (ax + by + cz + d = 0).
    :param translation: Boolean indicating whether to apply translation.
    :return: 4x4 transformation matrix.
    """
    # Plane coefficients
    a, b, c, d = plane_model
    normal = np.array([a, b, c])
    normal /= np.linalg.norm(normal)  # Normalize the normal vector

    # Target normal vector aligned with the z-axis
    target_normal = np.array([0, 0, 1])

    # Calculate rotation using scipy.spatial.transform.Rotation
    if not np.allclose(normal, target_normal):  # Check if normal is already aligned with target
        rotation_vector = np.cross(normal, target_normal)
        rotation_angle = np.arccos(np.clip(np.dot(normal, target_normal), -1.0, 1.0))
        rotation = R.from_rotvec(rotation_angle * rotation_vector / np.linalg.norm(rotation_vector))
        rotation_matrix = rotation.as_matrix()
    else:
        rotation_matrix = np.eye(3)  # If already aligned, use identity matrix

    # Initialize transformation matrix as 4x4 identity
    transform_matrix = np.eye(4)
    transform_matrix[:3, :3] = rotation_matrix  # Apply rotation

    # Compute translation vector if required
    if translation:
        # Translate the plane to the origin by moving along the normal direction by d
        translation_vector = -d * normal
        transform_matrix[:3, 3] = translation_vector

    return transform_matrix

def main(pcd_folder, trans_matrix_path, visualize=VISUALIZE):
    """
    Main function to find the ground plane and generate transformation matrix
    :param args:
    :return:
    """
    # List all PCD files
    pcd_file_paths = glob(os.path.join(pcd_folder, '*.pcd'))
    # Load the first PCD file
    pcd = read_pcd(pcd_file_paths[0])
    # Downsample the point cloud
    pcd = pcd.voxel_down_sample(VOXEL_SIZE)
    # Find the ground plane
    plane_model, ground_points, non_ground_points = find_ground_plane(pcd)
    transformation_matrix = compute_transformation_matrix(plane_model)
    print(transformation_matrix.tolist())
    # Save the transformation matrix
    # ensure_directory_exists(transformation_matrix_path)
    np.save(trans_matrix_path, transformation_matrix)
    # Visualize the ground
    if visualize:
        visualize_ground(ground_points, non_ground_points)

    return transformation_matrix