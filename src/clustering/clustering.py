# clustering.py

import numpy as np
from sklearn.cluster import DBSCAN

from ..utils.utils import create_pcd

# Constants
VOXEL_SIZE = 0.05
CLUSTER_EPS = 1.25
CLUSTER_MIN_SAMPLES = 80
MIN_POINTS_IN_CLUSTER = 10
MAX_CLUSTER_DIMENSION = 10.0
MIN_CLUSTER_DIMENSION = 0.2

def cluster_and_filter_points(pcd, cluster_eps=CLUSTER_EPS,cluster_min_samples=CLUSTER_MIN_SAMPLES,voxel_size=VOXEL_SIZE):
    """
    Cluster points and filter out small clusters.
    :param pcd:
    :param cluster_eps:
    :param cluster_min_samples:
    :param voxel_size:
    :return:
    """
    # Downsample the point cloud
    pcd_down = pcd.voxel_down_sample(voxel_size=voxel_size)
    points = np.asarray(pcd_down.points)
    if len(points) < cluster_min_samples:
        return pcd_down, [], []
    # Cluster the points
    db = DBSCAN(eps=cluster_eps,
                min_samples=cluster_min_samples,
                algorithm='auto',
                n_jobs=-1)
    labels = db.fit_predict(points)
    mask = labels != -1
    core_points = points[mask]
    core_labels = labels[mask]
    if len(core_points) == 0:
        return pcd_down, [], []
    core_pcd = create_pcd(core_points)
    return core_pcd, core_points, core_labels

def get_oriented_bounding_box(points):
    """
    Compute oriented bounding box for a set of points.
    :param points:
    :return:
    """
    pcd = create_pcd(points)
    bbox = pcd.get_oriented_bounding_box()
    center = np.asarray(bbox.center)
    size = np.asarray(bbox.extent)
    rotation = np.asarray(bbox.R)
    # Check if the bounding box is valid
    if (np.any(size > MAX_CLUSTER_DIMENSION) or
            np.any(size < MIN_CLUSTER_DIMENSION)):
        return None
    yaw = np.arctan2(rotation[1, 0], rotation[0, 0])
    return [float(center[0]), float(center[1]), float(center[2]),
            float(size[0]), float(size[1]), float(size[2]),
            float(yaw)]

def is_valid_bbox(bbox):
    """
    Check if the bounding box is valid.
    :param bbox:
    :return:
    """
    if bbox is None or len(bbox) != 7:
        return False
    size = np.array(bbox[3:6])
    return (np.all(size < MAX_CLUSTER_DIMENSION) and np.all(size > MIN_CLUSTER_DIMENSION))

def filter_bounding_boxes(bboxes):
    """
    Filter out invalid bounding boxes.
    :param bboxes:
    :return:
    """
    return [bbox for bbox in bboxes if is_valid_bbox(bbox)]

def main(pcd):
    """
    Main function.
    :param pcd:
    :return:
    """
    # Cluster and filter points
    core_pcd, core_points, core_labels = cluster_and_filter_points(pcd)
    if len(core_labels) == 0:
        return []
    # Get oriented bounding boxes
    bboxes = []
    unique_labels = np.unique(core_labels)
    for label in unique_labels:
        cluster_points = core_points[core_labels == label]
        if len(cluster_points) < MIN_POINTS_IN_CLUSTER:
            continue
        bbox = get_oriented_bounding_box(cluster_points)
        if bbox is not None:
            bboxes.append(bbox)
    # Filter out invalid bounding boxes
    filtered_bboxes = filter_bounding_boxes(bboxes)
    return filtered_bboxes