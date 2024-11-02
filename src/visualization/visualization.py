# visualization.py

import numpy as np
import open3d as o3d

def visualize_pcd(final_pcd):
    """
    Visualize the final point cloud in gray color.
    Args:
        final_pcd (o3d.geometry.PointCloud): The point cloud to visualize
    """
    final_pcd.paint_uniform_color([0.5, 0.5, 0.5])  # Gray color
    # Create a visualizer object
    vis = o3d.visualization.Visualizer()
    vis.create_window()
    vis.add_geometry(final_pcd)
    # Set the rendering options
    opt = vis.get_render_option()
    opt.background_color = np.array([1, 1, 1])  # White background
    opt.point_size = 1.0  # Set point size
    # Run the visualizer
    vis.run()
    vis.destroy_window()

def visualize_pcd_with_bbox(pcd, bboxes):
    """
    Visualize point cloud with bounding boxes.
    Args:
        pcd (o3d.geometry.PointCloud): Point cloud to visualize
        bboxes (list): List of bounding box parameters [x,y,z,w,l,h,yaw]
    """
    pcd.paint_uniform_color([0.5, 0.5, 0.5])  # Gray for point cloud
    vis = o3d.visualization.Visualizer()
    vis.create_window()
    vis.add_geometry(pcd)
    # Add bounding boxes
    for bbox in bboxes:
        center = bbox[:3]
        extent = bbox[3:6]
        R = o3d.geometry.get_rotation_matrix_from_xyz((0, 0, bbox[6]))
        obb = o3d.geometry.OrientedBoundingBox(center, R, extent)
        obb.color = (1, 0, 0)  # Red color for bounding box
        vis.add_geometry(obb)
    opt = vis.get_render_option()
    opt.background_color = np.array([1, 1, 1])  # 白色背景
    opt.point_size = 1.0
    vis.run()
    vis.destroy_window()

def visualize_ground(ground_points, non_ground_points):
    """
    Visualize the ground plane with colored points.
    Args:
        ground_points (o3d.geometry.PointCloud): Ground points
        non_ground_points (o3d.geometry.PointCloud): Non-ground points
    """
    ground_points.paint_uniform_color([1, 0, 0])  # Red for ground
    non_ground_points.paint_uniform_color([0.5, 0.5, 0.5])  # Gray for non-ground
    vis = o3d.visualization.Visualizer()
    vis.create_window()
    vis.add_geometry(ground_points)
    vis.add_geometry(non_ground_points)
    opt = vis.get_render_option()
    opt.background_color = np.array([1, 1, 1])
    opt.point_size = 1.0
    vis.run()
    vis.destroy_window()

def visualize_dynamic_points(dynamic_points, static_points):
    """
    Visualize dynamic and static points with different colors.
    Args:
        dynamic_points (o3d.geometry.PointCloud): Dynamic points
        static_points (o3d.geometry.PointCloud): Static points
    """
    dynamic_points.paint_uniform_color([1, 0, 0])  # Red for dynamic points
    static_points.paint_uniform_color([0.5, 0.5, 0.5])  # Gray for static points
    vis = o3d.visualization.Visualizer()
    vis.create_window()
    vis.add_geometry(dynamic_points)
    vis.add_geometry(static_points)
    opt = vis.get_render_option()
    opt.background_color = np.array([1, 1, 1])
    opt.point_size = 1.0
    vis.run()
    vis.destroy_window()