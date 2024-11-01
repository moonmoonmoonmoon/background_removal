# background_removal
# Background Removal and Clustering for Point Cloud Data

This project processes Point Cloud Data (PCD) files by removing background and clustering foreground points.

## Setup

1. Clone the repository:
git clone https://github.com/siyuanmengmax/background-removal.git
cd background_removal
2. Install required packages:
pip install -r requirements.txt

## Usage

python main.py

## File Structure
    
    ```
    .
    ├── README.md
    ├── LICENSE
    ├── requirements.txt
    ├── main.py
    ├── background_removal
    │   ├── __init__.py
    │   ├── background_removal.py
    │   ├── clustering.py
    │   ├── io.py
    │   ├── visualization.py
    │   └── utils.py
    ├── data
    │   ├── input
    │   │   └── 0000000000.pcd
    │   ├── output
    │   │   ├── 0000000000_background_removed.pcd
    │   │   └── 0000000000_clustered.pcd
    │   └── visualization
    │       ├── 0000000000_background_removed.png
    │       └── 0000000000_clustered.png
    └── tests
        ├── test_background_removal.py
        ├── test_clustering.py
        ├── test_io.py
        ├── test_visualization.py
        └── test_utils.py
    ```


## License

This project is licensed under the MIT License - see the LICENSE file for details.
