#ifndef OPENVSLAM_DATA_OCCUPANCY_MAP_INFO_H
#define OPENVSLAM_DATA_OCCUPANCY_MAP_INFO_H

namespace openvslam {
namespace data {
    
struct occupancy_map_info {
    unsigned long width, height;
    // cell size in meters
    float resolution;
    // The origin of the map [m, m, rad].  This is the real-world pose of the
    // bottom left corner of cell (0,0) in the map.
    float origin_x, origin_y;
};

}
}

#endif