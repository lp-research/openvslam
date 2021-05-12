#ifndef OPENVSLAM_UTIL_TRANSFORMATION_H
#define OPENVSLAM_UTIL_TRANSFORMATION_H

#include "openvslam/type.h"

#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <utility>

namespace openvslam {
namespace util {

    inline Mat44_t toPose(Mat33_t const &rot, Vec3_t const& trans) {
        Mat44_t pose = Mat44_t::Identity();
        pose.block<3, 3>(0, 0) = rot;
        pose.block<3, 1>(0, 3) = trans;

        return pose;
    }

    inline Mat44_t navDataToCameraPose( Mat33_t const & rot_cw, Vec3_t const t_w) {
        const Vec3_t t_c = - (rot_cw * t_w);
        return toPose(rot_cw, t_c);
    }

    // returns rotation and orientation in the convention of the
    // navigation data
    inline std::pair<Mat33_t, Vec3_t> cameraPoseToNavRotTrans(Mat44_t const& pose) {
        const Mat33_t rot_cw = pose.block<3, 3>(0, 0);
        const Vec3_t trans_cw = pose.block<3, 1>(0, 3);
        const Vec3_t cam_center = -rot_cw.transpose() * trans_cw;

        return {rot_cw, cam_center};
    }

    inline std::pair<Mat33_t, Vec3_t> toRotTrans(Mat44_t const& pose) {
        return {pose.block<3, 3>(0, 0), pose.block<3, 1>(0, 3)};
    }

    /**
     * Computes the relative rotation from one quaternion to the another.
     * The returnerd quaternion fullfils this condition:
     * to = returned * from
     */
    inline Mat33_t relativeRotation(Mat33_t const& from, Mat33_t const& to) {
        const Quat_t qFrom(from);
        const Quat_t qTo(to);

        // rotate our target quaternion back by the from quaternion
        // equivalent to: diff = To - From
        const Quat_t qRotationTo = qFrom.inverse() * qTo;
        return qRotationTo.toRotationMatrix();
    }

}
}

#endif