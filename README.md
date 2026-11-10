# MSOL
Multi-Sector Overlap Loss: A Universal Framework for 6DoF Global Localization Across Heterogeneous LiDARs.


### Installation

#### Prerequisites

Install required dependencies:

```bash
sudo apt-get install nlohmann-json3-dev
sudo apt-get install libeigen3-dev
sudo apt install libgtsam-dev libgtsam-unstable-dev
```

> **Note:** Please install ceres-solver (version > 2.1.0) by following the guide at [ceres Installation](http://ceres-solver.org/installation.html).

#### Setup & Usage

```bash
# Clone the repository
git clone https://github.com/Hfx-J/MSOL.git

# Navigate to project directory
cd MSOL 

# Build the project
catkin_make -DPYTHON_EXECUTABLE=/usr/bin/python3

# Setup environment
source ./devel/setup.bash

# Launch the application
roslaunch msol heliplace_v2.launch
```
### DataSet
We provide a example dataset in [BaiDuDrive](https://pan.baidu.com/s/1WVywzNPgzUt4VT0o5HGHhQ?pwd=f5i5).
### Citation

If you find our work useful in your research, please consider citing:

```
@ARTICLE{11122650,
  author={Gao, Wang and Huang, Feixuan and Liu, Hong and Pan, Shuguo and Zhao, Heng},
  journal={IEEE Robotics and Automation Letters}, 
  title={Multi-Sector Overlap Loss: A Universal Framework for One-Shot 6DoF Global Localization Across Heterogeneous LiDARs}, 
  year={2025},
  volume={10},
  number={10},
  pages={9862-9869},
  keywords={Laser radar;Location awareness;Point cloud compression;Semantics;Accuracy;Pose estimation;Windows;Data mining;Computational modeling;Training;Localization;SLAM;heterogeneous LiDAR;cylindrical coordinate;overlap loss strategy},
  doi={10.1109/LRA.2025.3597891}}

```

### Contact

For any questions or issues regarding MSOL, please open an issue on GitHub.


## Using MSOL from another C++ project

The core library is in `src/FAST_LIO/msol`. It requires PCL (common, filters,
and kdtree) and optionally uses OpenMP. It builds without ROS or CUDA.

```cmake
add_subdirectory("/path/to/MSOL-main/src/FAST_LIO/msol" msol-build)
target_link_libraries(your_target PRIVATE msol::msol)
```

```cpp
#include <msol/msol.hpp>

// map_cloud and query_cloud are msol::Cloud::ConstPtr in their sensor frames.
const auto map = msol::makeDescriptor(map_cloud);
const auto query = msol::makeDescriptor(query_cloud);
const auto match = msol::matchDescriptors(map, query);
const double yaw_degrees = match.yaw_shift_sectors * 360.0 / map.size();
```

Use the same sector count for both descriptors. The matcher compares the common
prefix of each pair of sector rows and returns the raw score for each window,
along with the best score and the signed sector shift used by the ROS node.
An all-empty query yields zero scores and zero shift.

After transforming a candidate query into the target frame, call
`msol::computeOverlap(aligned_query, target, distance_threshold, voxel_size)`
to get the fraction of source points with a nearby target point. Distances are
in meters. Pass a nonpositive voxel size to skip downsampling.
