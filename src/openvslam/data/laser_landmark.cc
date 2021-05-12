#include "openvslam/data/laser_landmark.h"

#include "openvslam/data/keyframe.h"

namespace openvslam {
namespace data {

std::atomic<unsigned int> laser_landmark::next_id_{0};

laser_landmark::laser_landmark(laser2d const& l2d, keyframe * kfrm,
    laser::laser_scanner_base * scanner) : laser2d_(l2d),
        ref_keyfrm_(kfrm),
        laser_scanner_(scanner) {

}

std::vector<Vec3_t> laser_landmark::get_points_in_world() const {
    // we might have no information on the laser or no measurements
    if (laser_scanner_ == nullptr || !laser2d_.is_valid()) {
        return {};
    }

    return laser_scanner_->laser2d_in_world(laser2d_, ref_keyfrm_->get_cam_pose());
}

std::vector<std::pair<Vec3_t, Vec3_t>> laser_landmark::get_laser_rays_in_world() const {
    return laser_scanner_->get_laser_rays_in_world(laser2d_, ref_keyfrm_->get_cam_pose());
}

}
}