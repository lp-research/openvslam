#ifndef OPENVSLAM_LASER_SCANNER_BASE_H
#define OPENVSLAM_LASER_SCANNER_BASE_H

#include "openvslam/type.h"

#include <string>
#include <limits>

#include <opencv2/core.hpp>
#include <yaml-cpp/yaml.h>
#include <nlohmann/json_fwd.hpp>

namespace openvslam {

namespace data{
    class laser2d;
}

namespace laser {

class laser_scanner_base {
public:
    virtual std::vector<Vec3_t> laser2d_in_world(data::laser2d const& /*data*/, Mat44_t const& /*cam_pose_cw*/) const {
        return {};
    }

    virtual std::vector<std::pair<Vec3_t, Vec3_t>> get_laser_rays_in_world(data::laser2d const& /*data*/, Mat44_t const& /*cam_pose_cw*/) const {
        return {};
    }
};

}
}

#endif