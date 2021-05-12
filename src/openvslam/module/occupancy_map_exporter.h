#ifndef OPENVSLAM_MODULE_OCCUPANCY_MAP_EXPORTER_H
#define OPENVSLAM_MODULE_OCCUPANCY_MAP_EXPORTER_H

#include "openvslam/data/occupancy_map_info.h"

namespace openvslam {

class config;

namespace data {
class frame;
class map_database;
class bow_database;
} // namespace data

namespace module {

class occupancy_map_exporter {
public:
    occupancy_map_exporter() = delete;

    //! Constructor
    occupancy_map_exporter(const std::shared_ptr<config>& cfg, data::map_database* map_db );

    unsigned long estimate_buffer_size() const;

    data::occupancy_map_info map_export(int8_t * map_data, unsigned long map_data_size) const;

private:
    struct occ_map_coords {
        // +x is pointing right
        long x;
        // +y is pointing up
        long y;
    };

    // -1 if not possible to map
    // world_pos must be in ROS coords
    long to_occ_map_index(data::occupancy_map_info const& map_info, Vec3_t world_pos,
        unsigned long map_data_size) const;

    // occ_coords must be in ROS coords
    long occ_map_index(data::occupancy_map_info const& map_info, occ_map_coords occ_coords,
        unsigned long map_data_size) const;

    // world_pos must be in ROS coords
    occ_map_coords to_occ_map(data::occupancy_map_info const& map_info, Vec3_t world_pos,
        unsigned long map_data_size) const;

    data::occupancy_map_info compute_parameters() const;

    // start and stop must be in ROS coords
    void set_line_of_sight(data::occupancy_map_info const& map_info, Vec3_t start, Vec3_t target,
        int8_t * map_data, unsigned long map_data_size) const;

    std::shared_ptr<config> cfg_;
    data::map_database * map_db_ = nullptr;

    const double laser_range_ = 5.0;
    const double cell_size_ = 0.05;
};

} // namespace module
} // namespace openvslam

#endif // OPENVSLAM_MODULE_OCCUPANCY_MAP_EXPORTER_H
