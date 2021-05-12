#ifndef OPENVSLAM_LASER_LANDMARK_H
#define OPENVSLAM_LASER_LANDMARK_H

#include "openvslam/type.h"
#include "openvslam/data/laser2d.h"
#include "openvslam/laser/laser_scanner_base.h"

#include <vector>

namespace openvslam {

namespace data {

class keyframe;

/*
Stores all the measurement of a laser2d scan and some utility methods to use them.
This is works a bit different than the optical landmarks. Laser_landmarks are not
obsevered by multiple keyframes but only used to move from keyframe to the following
one by using a matching of the laser data.
*/

class laser_landmark {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    laser_landmark() = default;

    laser_landmark(laser2d const& l2d, keyframe * kfrm,
        laser::laser_scanner_base * scanner);

    //! return the position of the laser measurement points
    //! in world coordinates, y coordinate (going down) is always 0
    std::vector<Vec3_t> get_points_in_world() const;
    std::vector<std::pair<Vec3_t, Vec3_t>> get_laser_rays_in_world() const;

private:
    unsigned int id_;
    // todo: make sure this works after loading map from file !!
    static std::atomic<unsigned int> next_id_;

    laser2d laser2d_;

    //! reference keyframe
    keyframe* ref_keyfrm_ = nullptr;

    //! laser model
    laser::laser_scanner_base * laser_scanner_ = nullptr;
};

} // namespace data
} // namespace openvslam

#endif // OPENVSLAM_LASER2D_H
