#include "msol/msol.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <numeric>
#include <stdexcept>
#include <utility>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>

namespace msol {
namespace {

constexpr double pi = 3.14159265358979323846;

/**
 * Find start and end positions of the unique continuous zero region in descriptor
 *
 * @param descriptor One-dimensional array, LiDAR descriptor
 * @param zero_threshold Float, values less than or equal to this threshold are considered "zero values"
 * @return Pair containing start index and end index of zero region (end index not included)
 *         Returns {-1, -1} if no zero values found
 */
std::pair<int, int> find_single_zero_region(const std::vector<float>& descriptor, float zero_threshold = 0.0f)
{
    // Return directly if descriptor is empty
    if (descriptor.empty()) {
        return {-1, -1};
    }

    // Create index array marking zero value positions
    std::vector<int> zero_indices;
    for (int i = 0; i < descriptor.size(); ++i) {
        if (descriptor[i] <= zero_threshold) {
            zero_indices.push_back(i);
        }
    }

    // Return directly if no zero values
    if (zero_indices.empty()) {
        return {-1, -1};
    }

    // If only single zero value
    if (zero_indices.size() == 1) {
        return {zero_indices[0], zero_indices[0] + 1};
    }

    // Check if it's a continuous region
    if (zero_indices.back() - zero_indices.front() == zero_indices.size() - 1) {
        return {zero_indices.front(), zero_indices.back() + 1};
    }

    // If zero values are not continuous, find the longest continuous segment
    std::vector<std::vector<int>> segments;
    std::vector<int> current_segment = {zero_indices[0]};

    for (int i = 1; i < zero_indices.size(); ++i) {
        if (zero_indices[i] == zero_indices[i-1] + 1) {
            // Current index is continuous with previous index
            current_segment.push_back(zero_indices[i]);
        } else {
            // Current index is not continuous with previous index, start new segment
            segments.push_back(current_segment);
            current_segment.clear();
            current_segment.push_back(zero_indices[i]);
        }
    }

    // Add the last segment
    if (!current_segment.empty()) {
        segments.push_back(current_segment);
    }

    // Find the longest segment
    auto longest_segment = std::max_element(
        segments.begin(), segments.end(),
        [](const std::vector<int>& a, const std::vector<int>& b) {
            return a.size() < b.size();
        }
    );

    // Return start and end indices of longest segment
    if (((*longest_segment).back() + 1 - (*longest_segment).front()) <= descriptor.size()/3) {
        return {-1, -1};
    } else {
        return {(*longest_segment).front(), (*longest_segment).back() + 1};
    }
}

}  // namespace

MatchResult matchDescriptors(const Descriptor& map, const Descriptor& query,
                             const Weights& weights) {
    if (map.empty() || map.size() != query.size()) {
        throw std::invalid_argument("MSOL descriptors must have the same nonzero sector count");
    }
    const auto& a = query;
    const auto& b = map;
    MatchResult match;
    auto& offset_sector = match.sector_offset;
    auto& zero_len = match.removed_sectors;
    // Calculate sum of each row
    std::vector<float> row_sums(a.size(), 0.0f);
    for (size_t i = 0; i < a.size(); ++i) {
        row_sums[i] = std::accumulate(a[i].begin(), a[i].end(), 0.0f);
    }

    // Find zero region
    const auto zero_region = find_single_zero_region(row_sums);
    const int zero_start = zero_region.first;
    const int zero_end = zero_region.second;
    // Create final_CLD, removing zero region
    Descriptor final_CLD;

    // Add part from zero_end to end
    if (zero_start!=zero_end)
    {
        for (size_t i = zero_end; i < a.size(); ++i) {
            final_CLD.push_back(a[i]);
        }

        // Add part from beginning to zero_start
        zero_len = zero_end - zero_start;
        for (size_t i = 0; i < zero_start; ++i) {
            final_CLD.push_back(a[i]);
        }
        if (final_CLD.empty()) {
            match.scores.assign(map.size(), 0.0f);
            return match;
        }
        if (zero_len <= a.size()/2)
            offset_sector = zero_end - 1; // - 2
        else
            offset_sector = int(final_CLD.size()/2) -1; // - 2

    }
    else{
        final_CLD = a;
    }
    // Create final_map, expanding b
    Descriptor final_map;

    // Add rear part of b
    size_t start_idx = (b.size() > final_CLD.size() - 1) ?
                      (b.size() - final_CLD.size() + 1) : 0;
    for (size_t i = start_idx; i < b.size(); ++i) {
        final_map.push_back(b[i]);
    }

    // Add all parts of b
    for (const auto& row : b) {
        final_map.push_back(row);
    }
    // Initialize result array
    size_t result_size = final_map.size() - final_CLD.size() + 1;
    std::vector<float> result_cosine(result_size, 0.0f);
    match.compared_sectors = final_CLD.size();
    // Efficient sliding window calculation
    for (size_t i = 0; i < result_size; ++i) {
        float sum = 0.0f;

        // Calculate similarity for current window
        for (size_t j = 0; j < final_CLD.size(); ++j) {
            const auto& window_row = final_map[i + j];
            const auto& cld_row = final_CLD[j];

            // Ensure both rows have same length
            size_t col_size = std::min(window_row.size(), cld_row.size());

            for (size_t k = 0; k < col_size; ++k) {
                // Calculate index
                if ((window_row[k] != 0 && window_row[k] != 1) ||
                    (cld_row[k] != 0 && cld_row[k] != 1)) {
                    throw std::invalid_argument("MSOL descriptor bins must be binary");
                }
                sum += weights[window_row[k] + cld_row[k]];
            }
        }

        result_cosine[i] = sum;
    }

    match.scores = std::move(result_cosine);
    const auto best = std::max_element(match.scores.begin(), match.scores.end());
    match.best_score = *best;
    match.yaw_shift_sectors = static_cast<int>(best - match.scores.begin()) - offset_sector;
    return match;
}

Descriptor makeDescriptor(const Cloud::ConstPtr& point_cloud, int num_sectors,
                          float max_range, int min_points_per_sector) {
    if (!point_cloud || num_sectors <= 0 || min_points_per_sector <= 0 ||
        !std::isfinite(max_range) || max_range <= 0 ||
        max_range / 2.0 >= std::numeric_limits<int>::max() / 13) {
        throw std::invalid_argument("Invalid MSOL point cloud or descriptor parameters");
    }
    // Basic preprocessing and validity check
    if (point_cloud->points.empty()) {
        // Return all-zero descriptor
        return Descriptor(num_sectors, std::vector<int>(0, 0));
    }

    // Height filtering parameters
    const float z_min = -1.0f, z_max = 12.0f;
    const float interval_size = 1.0f;
    const float interval_size_r = 2.0f;

    // Store filtered points
    std::vector<pcl::PointXYZ> valid_points;

    // Filter points based on height and distance
    for (const auto& point : point_cloud->points) {
        float r_xy = std::sqrt(point.x * point.x + point.y * point.y);

        if (point.z >= z_min && point.z < z_max && r_xy < max_range) {
            valid_points.push_back(point);
        }
    }

    // Check if there are enough points after filtering
    if (valid_points.size() < min_points_per_sector) {
        return Descriptor (num_sectors, std::vector<int>(0, 0));
    }

    // Calculate sector size
    const float sector_size = 2.0f * pi / num_sectors;

    // Initialize descriptor
    int rz_dim = int(max_range / interval_size_r) * int((z_max - z_min) / interval_size);
    Descriptor sector_vector(num_sectors, std::vector<int>(rz_dim, 0));

    // Count points in each sector
    std::vector<int> sector_points_count(num_sectors, 0);
    std::vector<std::vector<std::pair<int, int>>> sector_rz(num_sectors);

    // Assign points to sectors
    for (size_t i = 0; i < valid_points.size(); i++) {
        const auto& point = valid_points[i];

        // Calculate polar coordinates
        float r = std::sqrt(point.x * point.x + point.y * point.y);
        float theta = std::atan2(point.y, point.x);

        // Ensure theta is in [0, 2��) range
        if (theta < 0) theta += 2.0f * pi;

        // Determine sector index
        int sector_idx = int(std::floor(theta / sector_size)) % num_sectors;

        // Store r and z values for this sector
        sector_rz[sector_idx].push_back(std::make_pair(r, point.z));
        sector_points_count[sector_idx]++;
    }

    // Process each sector
    for (int sector_idx = 0; sector_idx < num_sectors; sector_idx++) {
        // Skip sectors with insufficient points
        if (sector_points_count[sector_idx] < min_points_per_sector) {
            continue;
        }

        // Create 2D grid representation (rz plane)
        Descriptor new_rz(
            int((z_max - z_min) / interval_size),
            std::vector<int>(int(max_range / interval_size_r), 0)
        );

        // Fill grid with point data
        for (const auto& rz_pair : sector_rz[sector_idx]) {
            float r_value = rz_pair.first;
            float z_value = rz_pair.second;

            int interval_idz = int((z_value - z_min) / interval_size);
            int interval_idr = int(r_value / interval_size_r);

            // Boundary check
            if (interval_idz >= 0 && interval_idz < new_rz.size() &&
                interval_idr >= 0 && interval_idr < new_rz[0].size()) {
                new_rz[interval_idz][interval_idr] = 1;
            }
        }

        // Flatten 2D grid to 1D array
        std::vector<int>& sector_data = sector_vector[sector_idx];
        for (int iz = 0; iz < new_rz.size(); iz++) {
            for (int ir = 0; ir < new_rz[iz].size(); ir++) {
                int flat_idx = iz * new_rz[iz].size() + ir;
                if (flat_idx < sector_data.size()) {
                    sector_data[flat_idx] = new_rz[iz][ir];
                }
            }
        }
    }

    return sector_vector;
}

double computeOverlap(
    const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& small_cloud,
    const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& large_cloud,
    float distance_threshold,
    float voxel_size) {

    if (!small_cloud || !large_cloud || !std::isfinite(distance_threshold) ||
        distance_threshold < 0 || !std::isfinite(voxel_size)) {
        throw std::invalid_argument("Invalid MSOL overlap parameters");
    }
    if (small_cloud->empty() || large_cloud->empty()) {
        return 0.0;
    }
    Cloud::ConstPtr small_filtered = small_cloud;
    Cloud::ConstPtr large_filtered = large_cloud;

    // Downsample to improve efficiency (if voxel_size > 0)
    if (voxel_size > 0) {
        pcl::VoxelGrid<pcl::PointXYZ> vg;

        vg.setInputCloud(small_cloud);
        vg.setLeafSize(voxel_size, voxel_size, voxel_size);
        pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_source(new pcl::PointCloud<pcl::PointXYZ>);
        vg.filter(*filtered_source);
        small_filtered = filtered_source;

        vg.setInputCloud(large_cloud);
        vg.setLeafSize(voxel_size, voxel_size, voxel_size);
        pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_target(new pcl::PointCloud<pcl::PointXYZ>);
        vg.filter(*filtered_target);
        large_filtered = filtered_target;
    }

    if (small_filtered->empty() || large_filtered->empty()) {
        return 0.0;
    }

    // Build KD tree (using large point cloud)
    pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
    kdtree.setInputCloud(large_filtered);

    // Statistics variables
    int overlap_count = 0;
    const int small_cloud_size = small_filtered->size();

    // Parallel processing for efficiency
    #pragma omp parallel
    {
        // Local counter for each thread
        int local_count = 0;
        std::vector<int> pointIdxRadiusSearch;
        std::vector<float> pointRadiusSquaredDistance;

        // Parallel block processing
        #pragma omp for nowait
        for (int i = 0; i < small_cloud_size; ++i) {
            // Use radius search instead of K-nearest neighbor to avoid unnecessary distance sorting
            if (kdtree.radiusSearch(small_filtered->points[i], distance_threshold,
                                   pointIdxRadiusSearch, pointRadiusSquaredDistance, 1) > 0) {
                local_count++;
            }
        }

        // Merge thread local results
        #pragma omp atomic
        overlap_count += local_count;
    }

    // Calculate overlap ratio
    return static_cast<double>(overlap_count) / small_cloud_size;
}

}  // namespace msol
