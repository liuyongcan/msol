# MSOL

MSOL（Multi-Sector Overlap Loss）的独立 C++ 实现，提供点云扇区描述子生成、滑窗匹配和点云重叠率计算。外部项目可以用匹配结果估计候选偏航角，再对已对齐的点云计算重叠率。

## 构建

需要 CMake 3.10 或更高版本、支持 C++14 的编译器，以及 PCL 开发库（`common`、`filters`、`kdtree`）。OpenMP 可选。核心库无需 ROS 或 CUDA。

```bash
git clone https://github.com/liuyongcan/msol.git
cd msol
cmake -S . -B build
cmake --build build --config Release
```

若 CMake 找不到 PCL，可通过 `-DPCL_DIR=/path/to/pcl/cmake` 指定包含 `PCLConfig.cmake` 的目录。

## 接入 C++ 项目

将本仓库作为子目录引入，然后链接 `msol::msol`。下面假设仓库位于项目的 `third_party/msol` 目录，且 `your_target` 已定义。

```cmake
add_subdirectory(third_party/msol)
target_link_libraries(your_target PRIVATE msol::msol)
```

```cpp
#include <msol/msol.h>

double estimateYawDegrees(const msol::Cloud::ConstPtr& map_cloud,
                          const msol::Cloud::ConstPtr& query_cloud) {
    constexpr int sectors = 24;
    const auto map = msol::makeDescriptor(map_cloud, sectors);
    const auto query = msol::makeDescriptor(query_cloud, sectors);
    const auto match = msol::matchDescriptors(map, query);
    return match.yaw_shift_sectors * 360.0 / sectors;
}
```

## 接口

- `makeDescriptor(cloud, num_sectors, max_range, min_points_per_sector)` 生成二值扇区描述子。默认使用 24 个扇区、50 米最大距离，每扇区至少 3 个点。
- `matchDescriptors(map, query, weights)` 返回各滑窗的原始得分、最佳得分和带符号的偏航扇区偏移。地图和查询描述子应使用相同的扇区数。默认权重为 `{0, -1, 3}`，以两个二值栅格的占用值之和为索引。
- `computeOverlap(aligned_query, target, distance_threshold, voxel_size)` 返回源点云中能在目标点云里找到近邻的点所占的比例，范围为 0 到 1。两组点云应位于同一坐标系。默认近邻距离为 0.05 米，体素边长为 0.1 米。体素边长小于或等于 0 时跳过下采样。

## 论文

Gao, Wang, et al. *Multi-Sector Overlap Loss: A Universal Framework for One-Shot 6DoF Global Localization Across Heterogeneous LiDARs*. IEEE Robotics and Automation Letters, 2025.
