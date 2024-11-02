# Background Removal

A Python-based project for processing Point Cloud Data (PCD) files, featuring background removal and point cloud clustering capabilities.

## Features

- Background removal from point cloud data
- Point cloud clustering (2D and 3D)
- Multiple processing methods:
  - Brute force
  - KD-tree
  - Octree
  - Spherical coordinates
  - Voxel-based
- Ground plane detection
- Visualization tools

## Demo
### Background Removal Result
https://github.com/moonmoonmoonmoon/background_removal/blob/main/Screenshot%20from%202024-11-02%2000-03-51.png

*Demonstration of the background removal process*

## Setup

1. Clone the repository:
   ```bash
   git clone https://github.com/moonmoonmoonmoon/static_background_removal.git
   cd background-removal
   ```

2. Install required packages:
   ```bash
   pip install -r requirements.txt
   ```

## Detailed Usage Guide

### Basic Usage

Run the main script with default parameters:
```bash
python main.py
```

### Configuration Options

Key parameters that can be adjusted in the processing pipeline:

1. Background Removal:
- `distance_threshold`: Distance threshold for background point detection
- `angle_threshold`: Angle threshold for background point detection

2. Clustering:
- `eps`: The maximum distance between two samples for them to be considered neighbors
- `min_samples`: The minimum number of samples in a cluster

3. Ground Detection:
- `distance_threshold`: Maximum distance from point to plane for ground classification
- `max_iterations`: Number of RANSAC iterations

## Project Structure

```
background-removal/
├── src/
│   ├── clustering/
│   │   ├── __init__.py
│   │   ├── cluster_2d.py
│   │   └── cluster_3d.py
│   ├── find_background/
│   │   ├── __init__.py
│   │   ├── find_cartesian.py
│   │   └── find_spherical.py
│   ├── find_ground/
│   │   ├── __init__.py
│   │   └── find_ground.py
│   ├── process_point_cloud/
│   │   ├── __init__.py
│   │   ├── process_brute.py
│   │   ├── process_kdtree.py
│   │   ├── process_octree.py
│   │   ├── process_spherical.py
│   │   ├── process_voxel.py
│   │   └── utils.py
│   ├── utils/
│   │   ├── __init__.py
│   │   └── utils.py
│   ├── visualization/
│   │   ├── __init__.py
│   │   └── visualization.py
│   └── __init__.py
├── LICENSE
├── README.md
├── main.py
├── requirements.txt
└── tree_generator.py
```

## Module Description

- `clustering/`: Implements 2D and 3D point cloud clustering algorithms
- `find_background/`: Contains algorithms for background detection in both Cartesian and spherical coordinates
- `find_ground/`: Implements ground plane detection algorithms and get transformation martix
- `process_point_cloud/`: Core processing algorithms including various optimization methods
- `utils/`: Common utility functions
- `visualization/`: Tools for visualizing point cloud data and results

## Contributing Guidelines

We welcome contributions to improve the project! Here's how you can contribute:

### Setting up Development Environment

1. Fork the repository
2. Create a virtual environment (using conda):
   ```bash
    conda create -n background-removal python=3.11
    conda activate background-removal
   ```
3. Install development dependencies:
   ```bash
   pip install -r requirements.txt
   ```

### Code Style

- Follow PEP 8 coding standards
- Use meaningful variable and function names
- Add docstrings to functions and classes
- Keep functions focused and single-purpose

### Making Contributions

1. Create a new branch for your feature:
   ```bash
   git checkout -b your-branch-name
   ```

2. Make your changes and commit:
   ```bash
   git add .
   git commit -m "Meaningful commit message"
   ```

3. Push to your fork:
   ```bash
   git push origin your-branch-name
   ```

4. Create a Pull Request (PR) from your fork to our main repository

### Pull Request Guidelines

- Provide a clear description of the changes
- Include any relevant issue numbers
- Ensure all tests pass
- Add unit tests for new features
- Update documentation if needed

### Reporting Issues

When reporting issues, please include:
- A clear description of the problem
- Steps to reproduce the issue
- Expected vs actual behavior
- System information (OS, Python version, etc.)

## License

This project is licensed under the MIT License - see the [LICENSE](LICENSE) file for details.

