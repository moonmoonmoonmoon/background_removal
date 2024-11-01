import numpy as np

from ..utils.utils import create_pcd

# Constants
# moving scene
MIN_DISTANCE = 0.3
# static scene
# MIN_DISTANCE = 3
GROUND_LEVEL = -5.0
MAX_HEIGHT = 2.5

def filter_points(pcd, min_distance=MIN_DISTANCE, ground_level=GROUND_LEVEL, max_height=MAX_HEIGHT):
    """
    Filter points based on minimum distance, ground level, and maximum height.
    :param pcd:
    :param min_distance:
    :param ground_level:
    :param max_height:
    :return:
    """
    points = np.asarray(pcd.points)
    # print(np.min(points[:,2]),np.max(points[:,2]))
    distances = np.linalg.norm(points, axis=1)
    height_mask = (points[:, 2] >= ground_level) & (points[:, 2] <= max_height)
    filtered_points = points[(distances > min_distance) & height_mask]
    filtered_pcd = create_pcd(filtered_points)
    return filtered_pcd