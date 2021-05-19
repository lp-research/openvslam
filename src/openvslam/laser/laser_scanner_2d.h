

#ifndef OPENVSLAM_LASER_SCANNER_2D_H
#define OPENVSLAM_LASER_SCANNER_2D_H

#include "openvslam/type.h"
#include "openvslam/laser/laser_scanner_base.h"

#include <string>
#include <limits>
#include <functional>

#include <yaml-cpp/yaml.h>
#include <nlohmann/json_fwd.hpp>

namespace openvslam {
namespace laser {

class laser_scanner_2d : public laser_scanner_base {
public:
    std::vector<Vec3_t> laser2d_in_world(data::laser2d const& data, Mat44_t const& cam_pose_cw) const override;

    std::vector<std::pair<Vec3_t, Vec3_t>> get_laser_rays_in_world(data::laser2d const& data, Mat44_t const& cam_pose_cw) const override;
private:
    void internal_in_world(data::laser2d const& data, Mat44_t const& cam_pose_cw,
        std::function<void(Vec3_t, Vec3_t)> lmdAdder) const;
};

}
}

#endif