# Background Removal and Clustering for Point Cloud Data

This project provides tools for processing Point Cloud Data (PCD) files, focusing on background removal and clustering of foreground points. It supports different methods to detect and eliminate static backgrounds, enabling efficient analysis of dynamic foreground objects within the point cloud.

## Setup

1. **Clone the Repository**  
   ```bash
   git clone https://github.com/moonmoonmoonmoon/static_background_removal.git
   cd background_removal
2. **Install Required Packages**
   pip install -r requirements.txt
3. **Run the main script**
   python main.py
   
## File Structure
.
├── README.md              # Project documentation
├── LICENSE                # License information
├── requirements.txt       # List of required packages
├── background_removal     # Main project folder
│   ├── src                # Source code for background removal and clustering
│   ├── dataset            # Contains datasets for testing
│   │   ├── static         # Static background datasets
│   │   │   └── lowell     # Lowell dataset for static scenes
│   │   ├── moving         # Moving background datasets
│   │   │   └── boston_vehicle_mount # Boston vehicle-mounted dataset for dynamic scenes
│   ├── output             # Output folder for processed data
│   └── main.py            # Main script to run background removal and clustering


## License
This README provides a clear overview of the project, instructions for setup, usage, and a detailed breakdown of the file structure and datasets.

