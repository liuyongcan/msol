#include <msol/msol.h>

#include <exception>
#include <iostream>

int main() {
    try {
        msol::Cloud::Ptr map(new msol::Cloud);
        for (float x : {4.0f, 4.1f, 4.2f}) {
            map->push_back(pcl::PointXYZ(x, 1.0f, 0.5f));
        }

        // Simulate a query whose sensor frame is rotated 90 degrees.
        msol::Cloud::Ptr query(new msol::Cloud);
        for (const auto& point : *map) {
            query->push_back(pcl::PointXYZ(-point.y, point.x, point.z));
        }

        constexpr int sectors = 24;
        const auto map_descriptor = msol::makeDescriptor(map, sectors);
        const auto query_descriptor = msol::makeDescriptor(query, sectors);
        const auto match = msol::matchDescriptors(map_descriptor, query_descriptor);
        std::cout << "Best descriptor score: " << match.best_score << '\n'
                  << "Yaw shift (degrees): " << match.yaw_shift_sectors * 360.0 / sectors
                  << '\n';

        // Use the known inverse rotation to put the query in the map frame.
        msol::Cloud::Ptr aligned_query(new msol::Cloud);
        for (const auto& point : *query) {
            aligned_query->push_back(pcl::PointXYZ(point.y, -point.x, point.z));
        }

        constexpr float radius = 0.05f;
        constexpr float voxel_size = 0.0f;
        std::cout << "Overlap before alignment: "
                  << msol::computeOverlap(query, map, radius, voxel_size) << '\n'
                  << "Overlap after alignment: "
                  << msol::computeOverlap(aligned_query, map, radius, voxel_size) << '\n';
        return 0;
    } catch (const std::exception& error) {
        std::cerr << "MSOL demo failed: " << error.what() << '\n';
        return 1;
    }
}
