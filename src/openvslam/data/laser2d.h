#ifndef OPENVSLAM_LASER2D_H
#define OPENVSLAM_LASER2D_H

#include "openvslam/type.h"
#include "openvslam/openvslam_export.h"

#include <vector>

namespace openvslam {

namespace data {

class keyframe;

class OPENVSLAM_EXPORT laser2d {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    laser2d() = default;
    
    // range measurements in meters
    std::vector<float> ranges_;

    //! Pose of laser scanner in relation to the camera frame related to
    //! this laser scan.

    //! rotation: camera -> laser
    Mat33_t rot_lc_;
    //! translation: camera -> laser
    Vec3_t trans_lc_;

    // this is contained in the ROS message
    // so potenially it could change by every call from the laser!

    //! start angle of the scan [rad]
    float angle_min_ = 0.0;
    //! end angle of the scan [rad]
    float angle_max_ = 0.0;
    //! 200 measurements over the rangen [rad]
    float angle_increment_ = 0.;

    //! minimum range value [m]
    float range_min_ = 0.0;
    //! maximum range value [m]
    float range_max_ = 0.0;

    //! we assume valid if we have at least one entry
    bool is_valid() const {
        return ranges_.size() > 0;
    }

    std::vector<float> const& get_ranges() const {
        return ranges_;
    }
};

} // namespace data
} // namespace openvslam

#endif // OPENVSLAM_LASER2D_H
