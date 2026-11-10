#pragma once

#include <array>
#include <vector>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace msol {

using Cloud = pcl::PointCloud<pcl::PointXYZ>;
using Descriptor = std::vector<std::vector<int>>;
using Weights = std::array<int, 3>;

struct MatchResult {
    std::vector<float> scores;
    float best_score = 0.0f;
    int yaw_shift_sectors = 0;
    int sector_offset = 0;
    int removed_sectors = 0;
    int compared_sectors = 0;
};

// Coordinates and max_range are in meters.
Descriptor makeDescriptor(const Cloud::ConstPtr& point_cloud, int num_sectors = 24,
                          float max_range = 50.0f, int min_points_per_sector = 3);

// Both descriptors must use the same sector count. Weights index the sum of two binary bins.
MatchResult matchDescriptors(const Descriptor& map, const Descriptor& query,
                             const Weights& weights = {{0, -1, 3}});

// Fraction of source points near the target after optional voxel downsampling.
double computeOverlap(const Cloud::ConstPtr& source, const Cloud::ConstPtr& target,
                      float distance_threshold = 0.05f, float voxel_size = 0.1f);

}  // namespace msol
