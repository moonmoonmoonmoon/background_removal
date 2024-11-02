# utils.py

import open3d as o3d
import pickle
import os

def ensure_directory(directory):
    """
    Ensure that a directory exists.
    :param directory:
    :return:
    """
    if not os.path.exists(directory):
        os.makedirs(directory)
        print(f'Created directory: {directory}')

def read_pcd_custom(file_path):
    """
    Read a custom PCD file.
    :param file_path:
    :return:
    """
    points = []
    with open(file_path, 'r') as file:
        for line in file:
            if line.startswith('VERSION') or line.startswith('FIELDS') or line.startswith('SIZE') or \
                    line.startswith('TYPE') or line.startswith('COUNT') or line.startswith('WIDTH') or \
                    line.startswith('HEIGHT') or line.startswith('POINTS'):
                continue
            values = line.strip().split()
            if len(values) == 4:  # x, y, z, intensity
                x, y, z = map(float, values[:3])
                points.append([x, y, z])
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(points)
    return pcd

def read_pcd(file_path):
    """
    Read a custom PCD file.
    :param file_path:
    :return:
    """
    # Try reading the file with Open3D
    pcd = o3d.io.read_point_cloud(file_path)
    return pcd

def create_pcd(points):
    """
    Create a point cloud object.
    :param points:
    :return:
    """
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(points)
    return pcd

def write_pcd(file_path, pcd):
    """
    Write a point cloud to a PCD file.
    :param file_path:
    :param pcd:
    :return:
    """
    o3d.io.write_point_cloud(file_path, pcd)
    print(f'Saved point cloud to {file_path}')

def read_pkl(file_path):
    """
    Read a pickle file.
    :param file_path:
    :return:
    """
    with open(file_path, 'rb') as file:
        data = pickle.load(file)
    return data

def write_pkl(file_path, data):
    """
    Write a pickle file.
    :param file_path:
    :param data:
    :return:
    """
    with open(file_path, 'wb') as file:
        pickle.dump(data, file)
    print(f'Saved data to {file_path}')