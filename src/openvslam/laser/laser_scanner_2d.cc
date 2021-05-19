#include "openvslam/laser/laser_scanner_2d.h"

#include "openvslam/data/laser2d.h"
#include "openvslam/util/transformation.h"

#include <spdlog/spdlog.h>
#include <spdlog/fmt/ostr.h>

#include <iostream>

namespace openvslam {
namespace laser {

void laser_scanner_2d::internal_in_world(data::laser2d const& data, Mat44_t const& cam_pose_cw,
    std::function<void(Vec3_t, Vec3_t)> lmdAdder) const {

    const auto [cam_rot, cam_pos] = util::cameraPoseToNavRotTrans(cam_pose_cw);

    // add to our occupancy map
    float this_angle = data.angle_max_;
    // ROS laser scan ranges entries goes from angle_max to angle_min !
    for(size_t i = 0; i < data.get_ranges().size(); i++ ) {
        const auto tRange = data.get_ranges()[i];

        if ((tRange < data.range_min_) || (tRange > data.range_max_) || std::isnan(tRange)) {
            // ignore values outside of the range
            // we also don't insert ranges that are outside of the end range
            // of the lidar. In that case space stays "unknown" (grey) if there
            // is no open space detected to on obstacle laying behind.
        } else {
            // compute point in the occupancy map
            // laser scan values are in the coordinate system of the
            // laser scanner

            // 0 degree is in the center of the lidar
            // -90 degrees is to the left of the scanner
            // laser scanners forward direction
            float z_rel = cos(this_angle) * tRange;
            float x_rel = sin(this_angle) * tRange; 

            // rotate the laser points into our map frame and
            // shift it to the laser position in map
            const Vec3_t target_point_l = Vec3_t(x_rel, 0.0, z_rel);

            // shift laser points into camera frame
            const Vec3_t target_point_c = data.rot_lc_ * target_point_l + data.trans_lc_;

            // this is the point from where the laser beam was sent out
            // todo: ignores rotation for now ...
            const Vec3_t source_point_w = cam_pos + data.trans_lc_;

            // rotate into keyframe orientation:
            const Vec3_t target_point_w = cam_rot.transpose() * target_point_c + cam_pos;
            lmdAdder(source_point_w, target_point_w);
        }
        this_angle -= data.angle_increment_;
    }
}

std::vector<std::pair<Vec3_t, Vec3_t>> laser_scanner_2d::get_laser_rays_in_world(data::laser2d const& data, Mat44_t const& cam_pose_cw) const {
    std::vector<std::pair<Vec3_t, Vec3_t>> pts;
    pts.reserve(data.get_ranges().size());

    internal_in_world(data, cam_pose_cw, [&pts](Vec3_t cam, Vec3_t target ) {
        pts.push_back({cam, target});
    });

    return pts;
}

std::vector<Vec3_t> laser_scanner_2d::laser2d_in_world(data::laser2d const& data, Mat44_t const& cam_pose_cw) const {
    std::vector<Vec3_t> pts(data.get_ranges().size());
    pts.reserve(data.get_ranges().size());

    internal_in_world(data, cam_pose_cw, [&pts](Vec3_t, Vec3_t target ) {
        pts.push_back(target);
    });

    return pts;
}

}
}