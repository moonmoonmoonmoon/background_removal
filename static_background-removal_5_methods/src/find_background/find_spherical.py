# find_background.py

import numpy as np
import os
import random
from tqdm import tqdm
from glob import glob

from ..utils.utils import read_pcd,write_pcd,create_pcd
from ..visualization.visualization import visualize_pcd

# Constants
UPDATE_THRESHOLD = 0.01
SAMPLE_SIZE = 2
VISUALIZE = True
BG_GRID_SIZE = [1080, 360]

def point_cloud_to_grid(pcd, bg_grid_size):
    """
    Convert point cloud to grid.
    :param points:
    :param grid_size:
    :return:grid
    """
    points = np.asarray(pcd.points, dtype=np.float64)
    range_values = np.linalg.norm(points, axis=1)
    azimuth = np.arctan2(points[:, 1], points[:, 0])
    elevation = np.arctan2(points[:, 2], np.sqrt(np.sum(points[:, :2]**2, axis=1)))
    azimuth_bins = np.clip(np.floor((azimuth + np.pi) / (2 * np.pi) * bg_grid_size[0]).astype(int), 0, bg_grid_size[0] - 1)
    elevation_bins = np.clip(np.floor((elevation + np.pi / 2) / np.pi * bg_grid_size[1]).astype(int), 0, bg_grid_size[1] - 1)
    grid = np.full(bg_grid_size, -1.0, dtype=np.float64)
    np.maximum.at(grid, (azimuth_bins, elevation_bins), range_values)
    return grid

def update_grid(grid1, grid2, count_grid, update_threshold=UPDATE_THRESHOLD):
    """
    Update grid and count grid.
    :param grid1:
    :param grid2:
    :param count_grid:
    :param threshold:
    :return:updated_grid, count_grid
    """
    update_mask = (np.abs(grid1 - grid2) >= update_threshold) & (grid1 != -1) & (grid2 != -1)
    updated_grid = np.where(update_mask, np.maximum(grid1, grid2), grid1)
    count_grid[update_mask] += 1
    return updated_grid, count_grid

def grid_to_point_cloud(grid):
    """
    Convert grid to point cloud.
    :param grid:
    :return:pcd
    """
    az_bins, el_bins = np.where(grid != -1)
    range_vals = grid[az_bins, el_bins]
    azimuth = (az_bins / grid.shape[0]) * 2 * np.pi - np.pi
    elevation = (el_bins / grid.shape[1]) * np.pi - np.pi / 2
    x = range_vals * np.cos(elevation) * np.cos(azimuth)
    y = range_vals * np.cos(elevation) * np.sin(azimuth)
    z = range_vals * np.sin(elevation)
    points = np.column_stack((x, y, z))
    valid_mask = ~np.isnan(points).any(axis=1) & ~np.isinf(points).any(axis=1)
    print(f"Total points before filtering: {len(points)}")
    print(f"Valid points: {np.sum(valid_mask)}")
    pcd = create_pcd(points[valid_mask])
    return pcd

def process_pcd_files(pcd_files, bg_grid_size,sample_size=SAMPLE_SIZE):
    """
    Process PCD files and generate background grid.
    :param pcd_files:
    :param sample_size:
    :return:bg_grid, count_grid
    """
    # print(sample_size, len(pcd_files))
    sample_pcd_files = random.sample(pcd_files, min(sample_size, len(pcd_files)))
    pcd = read_pcd(sample_pcd_files[0])
    bg_grid = point_cloud_to_grid(pcd,bg_grid_size)
    count_grid = np.zeros_like(bg_grid)
    for pcd_file in tqdm(sample_pcd_files[1:], desc="Processing PCD files"):
        current_grid = point_cloud_to_grid(read_pcd(pcd_file),bg_grid_size)
        bg_grid, count_grid = update_grid(bg_grid, current_grid, count_grid)
    return bg_grid, count_grid

def main(pcd_folder, bg_pcd_path, bg_grid_size=BG_GRID_SIZE, visualize=VISUALIZE):
    """
    Main function to generate background grid from PCD files.
    :param folder_path:
    :param bg_grid_path:
    :param bg_grid_size:
    :param visualize:
    :return:
    """
    # Read PCD files and generate background grid
    pcd_file_paths = glob(os.path.join(pcd_folder, '*.pcd'))
    bg_grid, count_grid = process_pcd_files(pcd_file_paths, bg_grid_size)
    # Convert grid to point cloud
    final_pcd = grid_to_point_cloud(bg_grid)
    print(f"Final point cloud size: {len(final_pcd.points)}")
    # Save the final point cloud
    write_pcd(bg_pcd_path, final_pcd)
    # Visualize the final point cloud
    if visualize:
        visualize_pcd(final_pcd)
    return final_pcd