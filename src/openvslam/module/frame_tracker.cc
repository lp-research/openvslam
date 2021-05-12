#include "openvslam/camera/base.h"
#include "openvslam/data/frame.h"
#include "openvslam/data/keyframe.h"
#include "openvslam/data/landmark.h"
#include "openvslam/match/bow_tree.h"
#include "openvslam/match/projection.h"
#include "openvslam/match/robust.h"
#include "openvslam/module/frame_tracker.h"
#include "openvslam/util/transformation.h"
#include "openvslam/navigation/navigation.h"
#include "openvslam/util/logging.h"

#include <spdlog/spdlog.h>
#include <spdlog/fmt/ostr.h>

namespace openvslam {
namespace module {

frame_tracker::frame_tracker(camera::base* camera, const unsigned int num_matches_thr)
    : camera_(camera), num_matches_thr_(num_matches_thr), pose_optimizer_(), inertial_pose_optimizer_() {}

bool frame_tracker::navigation_based_track(data::frame& curr_frm, const data::frame& last_frm, const Mat44_t& velocity) const {
    if (!curr_frm.nav_state_.valid || !last_frm.nav_state_.valid) {
        return false;
    }
    Mat44_t motionBased = velocity * last_frm.cam_pose_cw_;
    SPDLOG_DEBUG("Motion based prediction : \n{0}", motionBased);

    SPDLOG_DEBUG("Pose of last frame: \n{0}", last_frm.cam_pose_cw_);
    Mat44_t cam_pose_from_nav = navigation::applyRelativeMotionToPose(last_frm.nav_state_, curr_frm.nav_state_,
        last_frm.cam_pose_cw_);

    curr_frm.set_cam_pose(cam_pose_from_nav);
    SPDLOG_DEBUG("VS: set frame pose to {0}", util::to_string(curr_frm.cam_pose_cw_));

    // 2D-3D対応を初期化
    std::fill(curr_frm.landmarks_.begin(), curr_frm.landmarks_.end(), nullptr);

    // last frameで見えている3次元点を再投影して2D-3D対応を見つける
    match::projection projection_matcher(0.9, true);
    const float margin = (camera_->setup_type_ != camera::setup_type_t::Stereo) ? 20 : 10;
    auto num_matches = projection_matcher.match_current_and_last_frames(curr_frm, last_frm, margin);

    SPDLOG_DEBUG("Found {0} matches in current frame with {1} landmarks available",
        num_matches,
        last_frm.landmarks_.size());

    // be more relaxed with mathches because we have nav data
    if (num_matches < num_matches_thr_ * 0.5) {
        // marginを広げて再探索
        std::fill(curr_frm.landmarks_.begin(), curr_frm.landmarks_.end(), nullptr);
        num_matches = projection_matcher.match_current_and_last_frames(curr_frm, last_frm, 2 * margin);
    }

    if (num_matches < num_matches_thr_ * 0.5) {
        SPDLOG_DEBUG("navigation based tracking failed: {} matches < {}", num_matches, num_matches_thr_);
        return false;
    }

    // pose optimization using the navigation data to got from last frame to curr_frm
    SPDLOG_DEBUG("Camera pose before optimization: \n{0}", curr_frm.get_cam_pose());
    pose_optimizer_.optimize(curr_frm, last_frm);
    SPDLOG_DEBUG("Camera pose after optimization: \n{0}", curr_frm.get_cam_pose());

    // outlierを除く
    const auto num_valid_matches = discard_outliers(curr_frm);

    if (num_valid_matches < num_matches_thr_ * 0.5) {
        SPDLOG_DEBUG("navigation based tracking failed: {} inlier matches < {}", num_valid_matches, num_matches_thr_);
        return false;
    }

    return true;
}

bool frame_tracker::motion_based_track(data::frame& curr_frm, const data::frame& last_frm, const Mat44_t& velocity) const {
    match::projection projection_matcher(0.9, true);

    SPDLOG_DEBUG("motion based tracking before update: {0}",
        curr_frm.cam_pose_cw_);

    // motion modelを使って姿勢の初期値を設定
    curr_frm.set_cam_pose(velocity * last_frm.cam_pose_cw_);

    SPDLOG_DEBUG("motion based tracking after update: {0}",
        curr_frm.cam_pose_cw_);

    // 2D-3D対応を初期化
    std::fill(curr_frm.landmarks_.begin(), curr_frm.landmarks_.end(), nullptr);

    // last frameで見えている3次元点を再投影して2D-3D対応を見つける
    const float margin = (camera_->setup_type_ != camera::setup_type_t::Stereo) ? 20 : 10;
    auto num_matches = projection_matcher.match_current_and_last_frames(curr_frm, last_frm, margin);

    if (num_matches < num_matches_thr_) {
        // marginを広げて再探索
        std::fill(curr_frm.landmarks_.begin(), curr_frm.landmarks_.end(), nullptr);
        num_matches = projection_matcher.match_current_and_last_frames(curr_frm, last_frm, 2 * margin);
    }

    if (num_matches < num_matches_thr_) {
        SPDLOG_DEBUG("motion based tracking failed: {} matches < {}", num_matches, num_matches_thr_);
        return false;
    }

    // pose optimization
    pose_optimizer_.optimize(curr_frm);

    // outlierを除く
    const auto num_valid_matches = discard_outliers(curr_frm);

    if (num_valid_matches < num_matches_thr_) {
        SPDLOG_DEBUG("motion based tracking failed: {} inlier matches < {}", num_valid_matches, num_matches_thr_);
        return false;
    }

    curr_frm.nav_state_.cam_rotation = curr_frm.get_rotation_inv();
    curr_frm.nav_state_.cam_translation = curr_frm.get_cam_center();
    // don't set to valid by default. Otherwise we will use it for downstream
    // optimizations without having actual navigation data
    //curr_frm.nav_state_.valid = false;

    return true;
}

bool frame_tracker::bow_match_based_track(data::frame& curr_frm, const data::frame& last_frm, data::keyframe* ref_keyfrm) const {
    match::bow_tree bow_matcher(0.7, true);

    // BoW matchを行うのでBoWを計算しておく
    curr_frm.compute_bow();

    // keyframeとframeで2D対応を探して，frameの特徴点とkeyframeで観測している3次元点の対応を得る
    std::vector<data::landmark*> matched_lms_in_curr;
    auto num_matches = bow_matcher.match_frame_and_keyframe(ref_keyfrm, curr_frm, matched_lms_in_curr);

    if (num_matches < num_matches_thr_) {
        SPDLOG_DEBUG("bow match based tracking failed: {} matches < {}", num_matches, num_matches_thr_);
        return false;
    }

    // 2D-3D対応情報を更新
    curr_frm.landmarks_ = matched_lms_in_curr;

    // pose optimization
    // 初期値は前のフレームの姿勢
    curr_frm.set_cam_pose(last_frm.cam_pose_cw_);
    pose_optimizer_.optimize(curr_frm);

    // outlierを除く
    const auto num_valid_matches = discard_outliers(curr_frm);

    if (num_valid_matches < num_matches_thr_) {
        SPDLOG_DEBUG("bow match based tracking failed: {} inlier matches < {}", num_valid_matches, num_matches_thr_);
        return false;
    }
    else {
        return true;
    }
}

bool frame_tracker::robust_match_based_track(data::frame& curr_frm, const data::frame& last_frm, data::keyframe* ref_keyfrm) const {
    match::robust robust_matcher(0.8, false);

    // Search 2D-2D matches between the ref keyframes and the current frame
    // to acquire 2D-3D matches between the frame keypoints and 3D points observed in the ref keyframe
    std::vector<data::landmark*> matched_lms_in_curr;
    auto num_matches = robust_matcher.match_frame_and_keyframe(curr_frm, ref_keyfrm, matched_lms_in_curr);

    if (num_matches < num_matches_thr_) {
        spdlog::debug("robust match based tracking failed: {} matches < {}", num_matches, num_matches_thr_);
        return false;
    }

    // Update the 2D-3D matches
    curr_frm.landmarks_ = matched_lms_in_curr;

    // Pose optimization
    // The initial value is the pose of the previous frame
    curr_frm.set_cam_pose(last_frm.cam_pose_cw_);
    pose_optimizer_.optimize(curr_frm);

    // Discard the outliers
    const auto num_valid_matches = discard_outliers(curr_frm);

    if (num_valid_matches < num_matches_thr_) {
        spdlog::debug("robust match based tracking failed: {} inlier matches < {}", num_valid_matches, num_matches_thr_);
        return false;
    }
    else {
        return true;
    }
}

unsigned int frame_tracker::discard_outliers(data::frame& curr_frm) const {
    unsigned int num_valid_matches = 0;

    for (unsigned int idx = 0; idx < curr_frm.num_keypts_; ++idx) {
        if (!curr_frm.landmarks_.at(idx)) {
            continue;
        }

        auto lm = curr_frm.landmarks_.at(idx);

        if (curr_frm.outlier_flags_.at(idx)) {
            curr_frm.landmarks_.at(idx) = nullptr;
            curr_frm.outlier_flags_.at(idx) = false;
            lm->is_observable_in_tracking_ = false;
            lm->identifier_in_local_lm_search_ = curr_frm.id_;
            continue;
        }

        ++num_valid_matches;
    }

    return num_valid_matches;
}

} // namespace module
} // namespace openvslam
