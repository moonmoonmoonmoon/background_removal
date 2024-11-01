# process_point_cloud.py

import numpy as np

from ..utils.utils import create_pcd
from ..visualization.visualization import visualize_dynamic_points
from ..find_background.find_spherical import point_cloud_to_grid
from .utils import filter_points

# Constants
FILTER_THRESHOLD = 1.0
VISUALIZE = True
BG_GRID_SIZE = [1080, 360]

def filter_background(pcd, bg_grid, grid_size, threshold=FILTER_THRESHOLD, visualize=VISUALIZE):
    """
    Remove background points.
    :param pcd:
    :param background_grid:
    :param filter_threshold:
    :param grid_size:
    :return:
    """
    points = np.asarray(pcd.points)
    # Convert Cartesian coordinates to spherical coordinates
    range_values = np.linalg.norm(points, axis=1)
    azimuth = np.arctan2(points[:, 1], points[:, 0])
    elevation = np.arctan2(points[:, 2], np.linalg.norm(points[:, :2], axis=1))
    # Normalize angles to fit into grid and ensure bins are within valid range
    azimuth_bins = np.clip(np.floor((azimuth + np.pi) / (2 * np.pi) * grid_size[0]).astype(int), 0, grid_size[0] - 1)
    elevation_bins = np.clip(np.floor((elevation + np.pi / 2) / np.pi * grid_size[1]).astype(int), 0, grid_size[1] - 1)
    # Ensure bins are within valid range
    azimuth_bins = np.clip(azimuth_bins, 0, grid_size[0] - 1)
    elevation_bins = np.clip(elevation_bins, 0, grid_size[1] - 1)
    dynamic_points = []
    static_points = []
    for i in range(len(range_values)):
        az_bin = azimuth_bins[i]
        el_bin = elevation_bins[i]
        is_dynamic = True
        for daz in [-1, 0, 1]:
            for delv in [-1, 0, 1]:
                neighbor_az_bin = az_bin + daz
                neighbor_el_bin = el_bin + delv
                if 0 <= neighbor_az_bin < grid_size[0] and 0 <= neighbor_el_bin < grid_size[1]:
                    neighbor_range = bg_grid[neighbor_az_bin, neighbor_el_bin]
                    if neighbor_range != -1 and np.abs(range_values[i] - neighbor_range) <= threshold:
                        is_dynamic = False
                        break
            if not is_dynamic:
                break
        if is_dynamic:
            dynamic_points.append(points[i])
        else:
            static_points.append(points[i])
    dynamic_pcd = create_pcd(np.array(dynamic_points))
    static_pcd = create_pcd(np.array(static_points))
    if visualize:
        visualize_dynamic_points(dynamic_pcd, static_pcd)
    return dynamic_pcd

def main(pcd, bg_pcd, trans_matrix, grid_size=BG_GRID_SIZE):
    """
    Main function.
    :param pcd:
    :param background_grid:
    :param transformation_matrix:
    :param grid_size:
    :return:
    """
    # Build background grid
    bg_grid = point_cloud_to_grid(bg_pcd, grid_size)
    # Filter background points
    filtered_pcd = filter_background(pcd, bg_grid, grid_size)
    # Transform the point cloud based on the transformation matrix of ground plane
    filtered_pcd.transform(trans_matrix)
    pcd.transform(trans_matrix)
    # Filter points based on minimum distance, ground level, and maximum height
    processed_pcd = filter_points(filtered_pcd)
    return processed_pcd, pcd


# def filter_background_1(pcd, background_grid, threshold=0.5, grid_size=(1080, 512)):
#     # Load all PCD files
#     all_pcd_files = load_random_pcd_files_from_folder(folder_path, 1)
#
#     # Randomly select a PCD file for dynamic point visualization
#     random_pcd_file = random.choice(all_pcd_files)
#     random_pcd_file = r'E:\Amherst Bus Stop LiDAR\otherside\PCD\Amherst_1st_Bus_stop__Frame_000105.pcd'
#     original_points = load_pcd(random_pcd_file)
#     print(f"Visualizing dynamic points from: {random_pcd_file}")
#
#     # Calculate the range of the points
#     range_values = np.linalg.norm(original_points, axis=1)
#
#     # Convert Cartesian coordinates to spherical coordinates
#     azimuth = np.arctan2(original_points[:, 1], original_points[:, 0])  # Horizontal angle
#     elevation = np.arctan2(original_points[:, 2], np.linalg.norm(original_points[:, :2], axis=1))  # Vertical angle
#
#     # Normalize angles to fit into grid
#     azimuth_bins = np.floor((azimuth + np.pi) / (2 * np.pi) * grid_size[0]).astype(int)
#     elevation_bins = np.array([np.argmin(np.abs(beam_angles - ele)) for ele in elevation])
#     elevation_bins = np.floor(elevation_bins * 4).astype(int)  # Scale to 512 bins
#
#     # Ensure bins are within valid range
#     azimuth_bins = np.clip(azimuth_bins, 0, grid_size[0] - 1)
#     elevation_bins = np.clip(elevation_bins, 0, grid_size[1] - 1)
#
#     # Helper function to check if a point matches the background in its own or neighboring cells
#     def is_background_point(az_bin, el_bin, range_value, threshold):
#         for daz in [-1, 0, 1]:
#             for delv in [-1, 0, 1]:
#                 neighbor_az_bin = az_bin + daz
#                 neighbor_el_bin = el_bin + delv
#                 if 0 <= neighbor_az_bin < grid_size[0] and 0 <= neighbor_el_bin < grid_size[1]:
#                     neighbor_range = background_grid[neighbor_az_bin, neighbor_el_bin]
#                     if neighbor_range != -1 and np.abs(range_value - neighbor_range) <= threshold:
#                         return True
#         return False
#
#     # Separate dynamic and static points
#     dynamic_points = []
#     static_points = []
#     for i in range(len(range_values)):
#         az_bin = azimuth_bins[i]
#         el_bin = elevation_bins[i]
#         if is_background_point(az_bin, el_bin, range_values[i], threshold):
#             static_points.append(original_points[i])
#         else:
#             dynamic_points.append(original_points[i])
#
#     # Convert to point clouds
#     dynamic_points = np.array(dynamic_points)
#     static_points = np.array(static_points)
#
#     dynamic_pcd = o3d.geometry.PointCloud()
#     dynamic_pcd.points = o3d.utility.Vector3dVector(dynamic_points)
#
#     static_pcd = o3d.geometry.PointCloud()
#     static_pcd.points = o3d.utility.Vector3dVector(static_points)
#
#     # Assign colors to the points for visualization
#     dynamic_colors = np.array([[1, 0, 0] for _ in range(dynamic_points.shape[0])])  # Red for dynamic points
#     static_colors = np.array([[0.5, 0.5, 0.5] for _ in range(static_points.shape[0])])  # Gray for static points
#
#     dynamic_pcd.colors = o3d.utility.Vector3dVector(dynamic_colors)
#     static_pcd.colors = o3d.utility.Vector3dVector(static_colors)
#
#     # Visualize the dynamic and static points
#     o3d.visualization.draw_geometries([static_pcd, dynamic_pcd])
#
# def filter_background_2(pcd, background_grid, threshold=0.5, grid_size=(1080, 360)):
#     points = np.asarray(pcd.points)
#     range_values = np.linalg.norm(points, axis=1)
#     azimuth = np.arctan2(points[:, 1], points[:, 0])
#     elevation = np.arctan2(points[:, 2], np.linalg.norm(points[:, :2], axis=1))
#     azimuth_bins = np.floor((azimuth + np.pi) / (2 * np.pi) * grid_size[0]).astype(int)
#     elevation_bins = np.floor((elevation + np.pi / 2) / np.pi * grid_size[1]).astype(int)
#     azimuth_bins = np.clip(azimuth_bins, 0, grid_size[0] - 1)
#     elevation_bins = np.clip(elevation_bins, 0, grid_size[1] - 1)
#
#     dynamic_points = []
#     for i in range(len(range_values)):
#         az_bin = azimuth_bins[i]
#         el_bin = elevation_bins[i]
#         is_dynamic = True
#         for daz in [-1, 0, 1]:
#             for delv in [-1, 0, 1]:
#                 neighbor_az_bin = az_bin + daz
#                 neighbor_el_bin = el_bin + delv
#                 if 0 <= neighbor_az_bin < grid_size[0] and 0 <= neighbor_el_bin < grid_size[1]:
#                     neighbor_range = background_grid[neighbor_az_bin, neighbor_el_bin]
#                     if neighbor_range != -1 and np.abs(range_values[i] - neighbor_range) <= threshold:
#                         is_dynamic = False
#                         break
#             if not is_dynamic:
#                 break
#
#         if is_dynamic:
#             dynamic_points.append(points[i])
#
#     dynamic_pcd = o3d.geometry.PointCloud()
#     dynamic_pcd.points = o3d.utility.Vector3dVector(np.array(dynamic_points))
#     return dynamic_pcd