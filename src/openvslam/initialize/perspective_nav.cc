#include "openvslam/camera/perspective.h"
#include "openvslam/camera/fisheye.h"
#include "openvslam/data/frame.h"
#include "openvslam/initialize/perspective_nav.h"
#include "openvslam/solve/homography_solver.h"
#include "openvslam/solve/fundamental_solver.h"
#include "openvslam/util/transformation.h"
#include "openvslam/util/logging.h"

#include <thread>

#include <spdlog/spdlog.h>
#include <spdlog/fmt/ostr.h>

namespace openvslam {
namespace initialize {

perspective_nav::perspective_nav(const data::frame& ref_frm,
                         const unsigned int num_ransac_iters, const unsigned int min_num_triangulated,
                         const float parallax_deg_thr, const float reproj_err_thr,
                         const float min_movement)
    : base(ref_frm, num_ransac_iters, min_num_triangulated, parallax_deg_thr, reproj_err_thr),
      ref_cam_matrix_(get_camera_matrix(ref_frm.camera_)), ref_nav_(ref_frm.nav_state_),
      min_movement_(min_movement) {
    spdlog::debug("CONSTRUCT: initialize::perspective_nav");
}

perspective_nav::~perspective_nav() {
    spdlog::debug("DESTRUCT: initialize::perspective_nav");
}

bool perspective_nav::initialize(const data::frame& cur_frm, const std::vector<int>& ref_matches_with_cur) {

    if (!cur_frm.nav_state_.valid) {
        spdlog::warn("Current frame has no navigation data, can't use for inialization");
        return false;
    }

    // set the current camera model
    cur_camera_ = cur_frm.camera_;
    // store the keypoints and bearings
    cur_undist_keypts_ = cur_frm.undist_keypts_;
    cur_bearings_ = cur_frm.bearings_;
    // align matching information
    ref_cur_matches_.clear();
    ref_cur_matches_.reserve(cur_frm.undist_keypts_.size());
    for (unsigned int ref_idx = 0; ref_idx < ref_matches_with_cur.size(); ++ref_idx) {
        const auto cur_idx = ref_matches_with_cur.at(ref_idx);
        if (0 <= cur_idx) {
            ref_cur_matches_.emplace_back(std::make_pair(ref_idx, cur_idx));
        }
    }

    // set the current camera matrix
    cur_cam_matrix_ = get_camera_matrix(cur_frm.camera_);

    eigen_alloc_vector<Mat33_t> init_rots;
    eigen_alloc_vector<Vec3_t> init_transes;
    eigen_alloc_vector<Vec3_t> init_normals;
    std::vector<bool> is_inlier(ref_cur_matches_.size(), true);

    // get the translation between the ref_frame
    // and our current frame

    // verified that its the curr to ref transformation direction
    // which needs to be input here

    Vec3_t rel_trans = cur_frm.nav_state_.cam_translation -
        ref_nav_.cam_translation;

    SPDLOG_DEBUG("This frame pos: \n{0} \n ref frame pos: \n{1}",
        cur_frm.nav_state_.cam_translation,
        ref_nav_.cam_translation);

    const double distance = rel_trans.norm();
    if (distance < min_movement_) {
        SPDLOG_DEBUG("Only move {0} m so far, not enough movement for initialization",
            distance);
        return false;
    }

    // needs to be rotated into the camera frame !
    openvslam::Quat_t cam_rot(ref_nav_.cam_rotation);

    // translation in camera frame
    // not sure why the has to be done here because
    // the variables inside find_most_plausible_pose()
    // are named trans_ref_to_cur ...
    Vec3_t trans_cw = - (cam_rot * rel_trans);

    init_transes.push_back(trans_cw);

    Mat33_t rotationRefToCur = util::relativeRotation(ref_nav_.cam_rotation,
        cur_frm.nav_state_.cam_rotation);
    init_rots.push_back(rotationRefToCur);

    assert(init_rots.size() == 1);
    assert(init_transes.size() == 1);

    SPDLOG_DEBUG("Used navigation information to compute initial rotation and translation:\n{0}\n{1}",
        util::to_string(init_rots[0]), util::to_string(init_transes[0]));

    const auto pose_is_found = find_most_plausible_pose(init_rots, init_transes, is_inlier, true);
    if (!pose_is_found) {
        spdlog::warn("Can't find pose for initial rotation and translation:\n{0}\n{1}",
            util::to_string(init_rots[0]), util::to_string(init_transes[0]));
        return false;
    }

    SPDLOG_DEBUG("initialization succeeded with navigation information");
    return true;
}

Mat33_t perspective_nav::get_camera_matrix(camera::base* camera) {
    switch (camera->model_type_) {
        case camera::model_type_t::Perspective: {
            auto c = static_cast<camera::perspective*>(camera);
            return c->eigen_cam_matrix_;
        }
        case camera::model_type_t::Fisheye: {
            auto c = static_cast<camera::fisheye*>(camera);
            return c->eigen_cam_matrix_;
        }
        default: {
            throw std::runtime_error("Cannot get a camera matrix from the camera model");
        }
    }
}

} // namespace initialize
} // namespace openvslam
