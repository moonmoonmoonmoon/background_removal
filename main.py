# main.py
import os
import argparse
from glob import glob
from tqdm import tqdm
import pandas as pd
import numpy as np
from natsort import natsorted

from src.process_point_cloud.process_kdtree import main as process_point_cloud_kdtree
from src.process_point_cloud.process_octree import main as process_point_cloud_octree
from src.process_point_cloud.process_spherical import main as process_point_cloud_spherical
from src.process_point_cloud.process_brute import main as process_point_cloud_brute
from src.process_point_cloud.process_voxel import main as process_point_cloud_voxel
from src.clustering.clustering import main as cluster_and_bbox
from src.utils.utils import read_pcd,read_pcd_custom,ensure_directory
from src.find_ground.find_ground import main as find_ground
from src.find_background.find_cartesian import main as find_background_cartesian
from src.find_background.find_spherical import main as find_background_spherical
from src.visualization.visualization import visualize_pcd_with_bbox

# Constants
FINDING_BG_METHOD = {'function':{0: find_background_cartesian,1: find_background_spherical},'name':{0: 'Cartesian',1: 'Spherical'}}
PROCESSING_METHOD = {'function':{
    0: process_point_cloud_spherical,
    1: process_point_cloud_brute,
    2: process_point_cloud_voxel,
    3: process_point_cloud_kdtree,
    4: process_point_cloud_octree},
    'name':{0: 'Spherical',1: 'Brute Force',2: 'Voxel',3: 'KD Tree',4: 'Octree'}
}

def main(args):
    """
    Main function to process PCD files and generate bounding boxes.

    Parameters:
    args: argparse.Namespace
        Command line arguments
    """
    print("=== Starting Point Cloud Processing Pipeline ===")
    # Step 1: Generate or load background model
    print("\n1. Preparing background model...")
    ensure_directory(args.output_folder)
    coordinate_type = FINDING_BG_METHOD['name'][args.finding_bg_method]
    bg_pcd_path = os.path.join(args.output_folder, f'background_{coordinate_type}.pcd')
    find_background = FINDING_BG_METHOD['function'][args.finding_bg_method]
    print(f"Using {coordinate_type} coordinates")
    if os.path.exists(bg_pcd_path):
        print(f"Loading existing background from {bg_pcd_path}")
        bg_pcd = read_pcd(bg_pcd_path)
    else:
        print(f"Generating new background model...")
        bg_pcd = find_background(pcd_folder=args.pcd_folder,bg_pcd_path=bg_pcd_path)
    # Step 2: Get transformation matrix for ground plane alignment
    print("\n2. Preparing transformation matrix...")
    trans_matrix_path = os.path.join(args.output_folder, "trans_matrix.npy")
    if os.path.exists(trans_matrix_path):
        print(f"Loading existing transformation matrix")
        trans_matrix = np.load(trans_matrix_path)
    else:
        print(f"Calculating new transformation matrix...")
        trans_matrix = find_ground(pcd_folder=args.pcd_folder, trans_matrix_path=trans_matrix_path)
    # Step 3: Process each PCD file
    print("\n3. Processing PCD files...")
    pcd_file_paths = natsorted(glob(os.path.join(args.pcd_folder, '*.pcd')))
    all_bboxes = []
    column_names = ["frame_id", "label", "x", "y", "z", "w", "l", "h", "yaw"]
    comparing_method = PROCESSING_METHOD['name'][args.processing_method]
    print(f'Using {comparing_method} to process point cloud')
    for frame, pcd_file_path in tqdm(enumerate(pcd_file_paths), desc="Processing frames"):
        if frame % args.frame_step == 0:  # Process every nth frame
            # Read point cloud
            pcd = read_pcd(pcd_file_path)
            # Process point cloud (background removal and ground transformation)
            print(f'Processing point cloud {frame+1}')
            process_point_cloud = PROCESSING_METHOD['function'][args.processing_method]
            processed_pcd,transformed_pcd = process_point_cloud(pcd=pcd, bg_pcd=bg_pcd,trans_matrix=trans_matrix)
            # Generate bounding boxes for clusters
            print(f'Clustering and generating bounding boxes for frame {frame+1}')
            bboxes = cluster_and_bbox(pcd=processed_pcd)
            # Save bounding box information
            for bbox in bboxes:
                all_bboxes.append({
                    "frame_id": frame + 1,
                    "label": 0,
                    "x": bbox[0], "y": bbox[1], "z": bbox[2],
                    "w": bbox[3], "l": bbox[4], "h": bbox[5],
                    "yaw": bbox[6]
                })
            # Visualize if required
            if args.visualize:
                visualize_pcd_with_bbox(processed_pcd, bboxes)
    # Save results
    print("\n4. Saving results...")
    bbox_path = os.path.join(args.output_folder, f'bboxes_{coordinate_type}_{comparing_method}.csv')
    bbox_df = pd.DataFrame(all_bboxes, columns=column_names)
    bbox_df.to_csv(bbox_path, index=False)
    print(f"Saved bounding box information to {bbox_path}")
    print("\n=== Processing Complete ===")

if __name__ == "__main__":
    # dataset = 'lowell'
    dataset = 'boston_vehicle_mount'
    parser = argparse.ArgumentParser(description="Remove background and detect objects in PCD files")
    parser.add_argument("--output_folder", default=f"output/{dataset}", help="Folder to save output files")
    parser.add_argument("--pcd_folder", default=f"dataset/{dataset}",help="Path to folder containing PCD files")
    parser.add_argument("--visualize", type=bool, default=True,help="Visualize the results")
    parser.add_argument("--finding_bg_method", type=int, default=1,help="Method to find background, 0 for Cartesian, 1 for Spherical")
    parser.add_argument("--processing_method", type=int, default=0,help="Method to process point cloud, 0 for spherical, 1 for brute force, 2 for voxel, 3 for kd tree, 4 for octree")
    parser.add_argument("--frame_step", type=int, default=1, help="Process every nth frame")
    args = parser.parse_args()
    main(args)