# find_background.py

import numpy as np
from glob import glob
import os
from tqdm import tqdm

from ..utils.utils import read_pcd, write_pcd, create_pcd
from ..visualization.visualization import visualize_pcd

# Constants
VOXEL_SIZE = 0.1  # Voxel size for downsampling
DISTANCE_THRESHOLD = 0.1  # Distance threshold to consider a point as background
SAMPLE_SIZE = 50  # Number of frames to build the background model
VISUALIZE = True

def process_single_frame(pcd, voxel_size=VOXEL_SIZE):
    """
    Process a single frame of point cloud data.
    :param pcd:
    :param voxel_size:
    :return:
    """
    # Downsample the point cloud
    downsampled_pcd = pcd.voxel_down_sample(voxel_size=voxel_size)
    # Remove statistical outliers
    cl, ind = downsampled_pcd.remove_statistical_outlier(nb_neighbors=20, std_ratio=2.0)
    return cl

def accumulate_background(pcd_files, voxel_size=VOXEL_SIZE, sample_size=SAMPLE_SIZE):
    """
    Accumulate multiple frames of point cloud data to build the background model.
    :param pcd_files:
    :param voxel_size:
    :param sample_size:
    :return:
    """
    # Randomly sample frames
    sample_files = np.random.choice(pcd_files, min(sample_size, len(pcd_files)), replace=False)
    # Initialize the merged point cloud
    points_dict = {}
    # Process each frame
    for file in tqdm(sample_files, desc="Processing frames for background"):
        current_pcd = read_pcd(file)
        # print(current_pcd)
        processed_pcd = process_single_frame(current_pcd, voxel_size)
        points = np.asarray(processed_pcd.points)
        # Accumulate points
        for point in points:
            # Convert point to voxel coordinates
            voxel_coord = tuple(np.round(point/voxel_size))
            if voxel_coord in points_dict:
                points_dict[voxel_coord]['count'] += 1
                points_dict[voxel_coord]['sum'] += point
            else:
                points_dict[voxel_coord] = {
                    'count': 1,
                    'sum': point
                }
    # Filter points based on frequency
    min_frequency = len(sample_files) * 0.3
    background_points = []
    for voxel_coord, data in points_dict.items():
        if data['count'] >= min_frequency:
            # Calculate the average point
            average_point = data['sum'] / data['count']
            background_points.append(average_point)
    # print(background_points)
    # Create the background point cloud
    background_pcd = create_pcd(np.array(background_points))
    return background_pcd

def main(pcd_folder, bg_pcd_path, visualize=VISUALIZE):
    """
    Main function to generate background point cloud from PCD files.
    :param folder_path:
    :param bg_path:
    :param visualize:
    :return:
    """
    # Get all PCD files
    pcd_files = glob(os.path.join(pcd_folder, '*.pcd'))
    # Generate the background point cloud
    bg_pcd = accumulate_background(pcd_files)
    # Save the background point cloud
    write_pcd(bg_pcd_path, bg_pcd)
    # Visualize the final point cloud
    if visualize:
        visualize_pcd(bg_pcd)
    return bg_pcd