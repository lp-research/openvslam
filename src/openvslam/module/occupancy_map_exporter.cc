#include "openvslam/config.h"
#include "openvslam/data/keyframe.h"
#include "openvslam/data/landmark.h"
#include "openvslam/data/map_database.h"
#include "openvslam/module/occupancy_map_exporter.h"

#include <spdlog/spdlog.h>
#include <spdlog/fmt/ostr.h>
#include <iostream>
#include <algorithm>

namespace {
    // ROS coords are: x fordward, y left, z up
    // VSLAM coords are: x right, y down, z forward
    // this method convert from vslam to ROS so the outputted
    // occupancy map is in proper ROS coordinate frame
    inline openvslam::Vec3_t vslam_to_ros(openvslam::Vec3_t const& ros) {
        return openvslam::Vec3_t(ros.z(), -ros.x(), -ros.y());
    }
}

namespace openvslam {
namespace module {

occupancy_map_exporter::occupancy_map_exporter(const std::shared_ptr<config>& cfg,
    data::map_database* map_db) : cfg_(cfg), map_db_(map_db) {
}

unsigned long occupancy_map_exporter::estimate_buffer_size() const {
    auto params = compute_parameters();
    return params.height * params.width;
}

data::occupancy_map_info occupancy_map_exporter::compute_parameters() const {

    auto kfs = map_db_->get_all_keyframes();

    if (kfs.size() == 0) {
        return {
            // width, height
            0, 0,
            cell_size_,
            0.0, 0.0
        };
    }

    // todo: check what happens when some keyframe gets deleted ?
    double min_x = std::numeric_limits<double>::max();
    double max_x = std::numeric_limits<double>::min();
    double min_y = std::numeric_limits<double>::max();
    double max_y = std::numeric_limits<double>::min();

    // get extend of all laser data
    for ( auto kf: kfs) {
        const auto cam_center = vslam_to_ros(kf->get_cam_center());
        // this is now in ROS coords, so use x and y to be on the
        // laser scanner plane
        min_x = std::min(cam_center.x() - laser_range_, min_x);
        max_x = std::max(cam_center.x() + laser_range_, max_x);
        min_y = std::min(cam_center.y() - laser_range_, min_y);
        max_y = std::max(cam_center.y() + laser_range_, max_y);
    }
    
    auto delta_x = max_x - min_x;
    auto delta_y = max_y - min_y;

    unsigned long cells_x = std::ceil(delta_x / cell_size_);
    unsigned long cells_y = std::ceil(delta_y / cell_size_);

    spdlog::info("Occupancy Map: Computed needed grid size to [{0}, {1}] to [{2}, {3}], size [{4}, {5}]",
        min_x, min_y, max_x, max_y,
        cells_x, cells_y);

    return { 
        // width, height
        cells_x, cells_y,
        cell_size_,
        min_x, min_y
        };
}

data::occupancy_map_info occupancy_map_exporter::map_export(int8_t * map_data, unsigned long map_data_size) const {
    data::occupancy_map_info inf = compute_parameters();

    // mark all entries as unknown in th beginning
    std::memset(map_data, -1, map_data_size);

    // insert all keyframe's laser data with the oldest keyframes first
    auto kfs = map_db_->get_all_keyframes();
    // sort by age
    std::sort(kfs.begin(), kfs.end(), [](auto * kf1, auto * kf2) { return kf1->timestamp_ < kf2->timestamp_; });

    int i = 0;
    for (data::keyframe * kf: kfs) {
        // compute laser ray
        const auto rays = kf->laser_landmark_.get_laser_rays_in_world();

        // draw this is occupancy map
        for (auto const & ray: rays) {
            set_line_of_sight(inf, vslam_to_ros(ray.first), vslam_to_ros(ray.second),
                map_data, map_data_size);
        }
        i++;
    }

    return inf;
}

long occupancy_map_exporter::to_occ_map_index(data::occupancy_map_info const& map_info,
    Vec3_t world_pos, unsigned long map_data_size) const {
    return occ_map_index(map_info, to_occ_map(map_info, world_pos, map_data_size),
        map_data_size);
}

long occupancy_map_exporter::occ_map_index(data::occupancy_map_info const& map_info, occ_map_coords occ_coords,
    unsigned long map_data_size) const {
    if (occ_coords.x < 0 || occ_coords.y < 0) {
        return -1;
    }

    // project into array
    long i = occ_coords.x + occ_coords.y * map_info.width;

    return i >= map_data_size ? -1 : i;
}

occupancy_map_exporter::occ_map_coords occupancy_map_exporter::to_occ_map(data::occupancy_map_info const& map_info, Vec3_t world_pos,
    unsigned long map_data_size) const {

    // use x and y here as world_pos is in ROS coordinates already
    const long x_grid = std::round((world_pos.x() - map_info.origin_x) / map_info.resolution);
    const long y_grid = std::round((world_pos.y() - map_info.origin_y) / map_info.resolution);

    return {x_grid, y_grid};
}

void occupancy_map_exporter::set_line_of_sight(data::occupancy_map_info const& map_info, Vec3_t start, Vec3_t target,
    int8_t * map_data, unsigned long map_data_size) const {

    // get start box
    auto start_pt = to_occ_map(map_info, start, map_data_size);
    auto target_pt = to_occ_map(map_info, target, map_data_size);

    int8_t pTarget = 100;
    int8_t pStart = 0;

    // move from start to end
    auto ix_distance = target_pt.x - start_pt.x;
    auto iy_distance = target_pt.y - start_pt.y;

    auto x_offset = start_pt.x;
    auto y_offset = start_pt.y;

    auto ix_direction = ix_distance >= 0 ? 1 : -1;
    auto iy_direction = iy_distance >= 0 ? 1 : -1;

    float y_slope = std::abs(float(iy_distance) / float(ix_distance));
    
    // will be true if the movement in y is larger than in x
    bool is_steep = std::abs(iy_distance) > std::abs(ix_distance);

    if (is_steep) {
        std::swap(ix_distance, iy_distance);
        std::swap(x_offset, y_offset);
        std::swap(ix_direction, iy_direction);
        y_slope = 1.0 / y_slope;
    }

    Eigen::Index iy = 0;

    for (auto ix = 0; ix <= std::abs(ix_distance); ix++) {
        // check if we need to increase y
        iy = std::ceil(y_slope * float(ix));

        const auto p = (ix == std::abs(ix_distance)) && ( iy == std::abs(iy_distance))
            ? pTarget : pStart;

        Eigen::Index gridPosX = x_offset + ix_direction * ix;
        Eigen::Index gridPosY = y_offset + iy_direction * iy;

        if (is_steep) {
            std::swap(gridPosX, gridPosY);
        }

        const auto index_to_update = occ_map_index(map_info, occ_map_coords{gridPosX, gridPosY}, map_data_size);

        if (index_to_update >= 0 ) {
            // todo: does not use propablities at the moment
            map_data[index_to_update] = p;
        }
    }
}


} // namespace module
} // namespace openvslam
