#include "openvslam/optimize/g2o/se3/navigation_pose_opt_edge.h"

namespace openvslam {
namespace optimize {
namespace g2o {
namespace se3 {

navigation_pose_opt_edge::navigation_pose_opt_edge()
    : ::g2o::BaseBinaryEdge<6, // 3 d for rotation, 3d for position of the computed error
        navigation_state, // contains the relative change from frame 1 to frame 2
        shot_vertex, shot_vertex>() {}

bool navigation_pose_opt_edge::read(std::istream& is) {
    for (unsigned int i = 0; i < 3; ++i) {
        is >> _measurement.cam_translation[i];
    }

    for (unsigned int i = 0; i < 3; ++i) {
        for (unsigned int j = 0; j < 3; ++j) {
            is >> _measurement.cam_rotation(i, j);
        }
    }

    for (unsigned int i = 0; i < 3; ++i) {
        for (unsigned int j = i; j < 3; ++j) {
            is >> information()(i, j);
            if (i != j) {
                information()(j, i) = information()(i, j);
            }
        }
    }

    return true;
}

bool navigation_pose_opt_edge::write(std::ostream& os) const {
    for (unsigned int i = 0; i < 3; ++i) {
        os << _measurement.cam_translation[i] << " ";

    }

    for (unsigned int i = 0; i < 3; ++i) {
        for (unsigned int j = 0; j < 3; ++j) {
            os << " " << _measurement.cam_rotation(i, j);
        }
    }

    for (unsigned int i = 0; i < 3; ++i) {
        for (unsigned int j = i; j < 3; ++j) {
            os << " " << information()(i, j);
        }
    }

    return os.good();
}

void navigation_pose_opt_edge::computeError() {
    // fitted vertex keyframe 1
    const auto v1_cw = static_cast<const shot_vertex*>(_vertices.at(0));
    // fitted vertex keyframe 2
    const auto v2_cw = static_cast<const shot_vertex*>(_vertices.at(1));

    // residual rotation between fitted vertices (in map coordinates)
    auto residualRot_cw = v1_cw->estimate().inverse() * v2_cw->estimate();

    // formulat in g2o terms
    ::g2o::SE3Quat nav_delta_cw(_measurement.cam_rotation,
        _measurement.cam_translation);

    // V1^T * V2 = Res | * Res^T
    // Res^T * V1^T * V2 = 0
    auto res = nav_delta_cw.inverse() * residualRot_cw;
    res.normalizeRotation();

    // for some reason I don't understand, the transformation of the translation is not working properly
    // with the above formula, probably because the translation in nav_delta_cw is not initialized with the
    // proper coordinate frame.
    // In the meantime, just compute the translation error in the global frame by hand
    ::g2o::Vector3 v1_t_wc = - (v1_cw->estimate().rotation().inverse() * v1_cw->estimate().translation());
    ::g2o::Vector3 v2_t_wc = - (v2_cw->estimate().rotation().inverse() * v2_cw->estimate().translation());

    // contains the position and x,y,z components of the quaternion which
    // should be close to zero if fitted vertex is close to the correct
    // position and orientation
    _error = res.toMinimalVector();

    ::g2o::Vector3 trans_res =
        // transform navigation delta translation into the global frame (by using the v1 pose as anchor point)
        -(v1_cw->estimate().rotation().inverse() *_measurement.cam_translation)
        // add delta of fitted vertices
        + (v2_t_wc - v1_t_wc);

    _error[0] = trans_res.x();
    _error[1] = trans_res.y();
    _error[2] = trans_res.z();
}

} // namespace se3
} // namespace g2o
} // namespace optimize
} // namespace openvslam
