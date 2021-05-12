#ifndef OPENVSLAM_DATA_NAVIGATION_STATE_H
#define OPENVSLAM_DATA_NAVIGATION_STATE_H

#include "openvslam/type.h"

#include <array>

namespace openvslam {
namespace data {

struct imu_measurement {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    // length of one sample in seconds
    double dt;

    double timestamp;

    // Acceleration in m/s^2
    Vec3_t acceleration;

    // Angular velocity in rad/s
    Vec3_t angular_velocity;
};
}
/**
 * Navigation state is always in cartensian coordinates
 * and not in homegenous ones like the camera pose.
 * 
 * Coordinate system is:
 * -z forward 
 * +x right
 * +y up
 */
struct navigation_state {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    bool valid = false;
    // Rotation of the camera from world coordinates to the
    // current rotation
    Mat33_t cam_rotation = Mat33_t::Identity();

    Mat33_t get_rotation_inverse() const {
        return cam_rotation.inverse();
    }

    // Translation of the camera from world coordinates to
    // current position
    Vec3_t cam_translation = Vec3_t::Zero();
    //std::array<double, 4 * 4> cam_pose;

    // velocity in world frame
    Vec3_t velocity = Vec3_t::Zero();

    // true if a valid velocity is contained
    bool velocity_valid = false;
};
}

#endif