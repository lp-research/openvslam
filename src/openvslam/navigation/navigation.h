
#ifndef OPENVSLAM_NAVIGATION_NAVIGATION_H
#define OPENVSLAM_NAVIGATION_NAVIGATION_H

#include "openvslam/camera/base.h"
#include "openvslam/data/navigation_state.h"
#include "openvslam/data/keyframe.h"
#include "openvslam/util/transformation.h"
#include "openvslam/optimize/internal/se3/navigation_pose_opt_edge.h"

#include <g2o/core/optimizable_graph.h>
#include <g2o/core/hyper_graph.h>

#include <opencv2/core.hpp>

#include <sstream>

namespace openvslam {
namespace navigation {

    // compute the relative motion between two navigation frames
    // output is in the coordinate frame of the first navigation_state
    // this method assumes that both navigation states are in the same coordinate frame !
    inline navigation_state computeRelativeRotPos(navigation_state const& state1,
        navigation_state const& state2) {

        // this is in the (global) odometry coordinate frame !
        const Vec3_t rel_trans_odom = state2.cam_translation -
            state1.cam_translation;
        const Mat33_t rotationRefToCur_odom = util::relativeRotation(state1.cam_rotation,
            state2.cam_rotation);

        // transform into the coordinate frame of the first state (s1)
        // so we are independant of the original navigation coordinate system
        const Vec3_t rel_trans_s1 = state1.cam_rotation * rel_trans_odom;
        // not need to rotate it in state1 frame because its already
        // the relative
        const Mat33_t rotationRefToCur_s1 = rotationRefToCur_odom;

        navigation_state out_nav;
        out_nav.valid = true;

        out_nav.cam_translation = rel_trans_s1;
        out_nav.cam_rotation = rotationRefToCur_s1;

        // todo: velocities

        return out_nav;
    }

    // applies the relative motion (rotation and translation) between two navigation
    // states to a pose
    inline Mat44_t applyRelativeMotionToPose_direct_matrix_not_working(navigation_state const& state1,
        navigation_state const& state2, Mat44_t const& pose_cw) {

        Vec3_t diff_trans_world = -(state2.cam_translation - state1.cam_translation);
        Mat33_t diff_rot_world = util::relativeRotation(state2.cam_rotation,
            state1.cam_rotation);

        // compute differential pose (in world frame)
        Mat44_t diff_pose_wc = util::toPose(diff_rot_world,
            diff_rot_world * diff_trans_world);

        Mat44_t cam_pose_from_nav = diff_pose_wc * pose_cw;

        return cam_pose_from_nav;
    }

    // This method uses quaternions to combine the transformation.
    // Takes a bit longer to compute but works
    inline Mat44_t applyRelativeMotionToPose(navigation_state const& state1,
        navigation_state const& state2, Mat44_t const& pose_cw) {

        Vec3_t diff_trans_world = state2.cam_translation - state1.cam_translation;

        // rototian from last to current nav state in world frame
        Mat33_t diff_rot_world = util::relativeRotation(state1.cam_rotation,
            state2.cam_rotation);

        // convert pose to world coords
        auto [pose_rot, pose_pos] = util::cameraPoseToNavRotTrans(pose_cw);

        Eigen::Quaterniond quat_pose_rot(pose_rot);
        Eigen::Quaterniond quat_diff_rot(diff_rot_world);

        Vec3_t pose_pos_new = pose_pos + diff_trans_world;
        Eigen::Quaterniond quat_pose_rot_new = quat_diff_rot * quat_pose_rot;

        Mat44_t cam_pose_from_nav = util::navDataToCameraPose(quat_pose_rot_new.toRotationMatrix(),
            pose_pos_new);

        return cam_pose_from_nav;
    }

    inline std::pair<double, data::keyframe *> findClosestKeyframeBefore(std::vector<data::keyframe *> const& keyfrms,
        data::keyframe * keyfrm) {
        // check which other keyframe is closest in time
        data::keyframe * keyfrm_before = nullptr;
        double time_distance = std::numeric_limits<double>::max();
        for (const auto other_keyfrm: keyfrms) {
            if (other_keyfrm == keyfrm) {
                continue;
            }

            // only use keyframes which actually have navigation data
            if (!other_keyfrm->nav_state_.valid) {
                continue;
            }

            const double time_distance_this = keyfrm->timestamp_ - other_keyfrm->timestamp_;
            if (time_distance_this < 0.0) {
                // this keyframe is after our current one
                continue;
            }
            if (time_distance_this < time_distance){
                time_distance = time_distance_this;
                keyfrm_before = other_keyfrm;
            }
        }

        return {time_distance, keyfrm_before};
    }

    template <class TFrameA, class TFrameB>
    inline auto createNavigationEdge(TFrameA const* keyfrm_before, TFrameB const* keyfrm,
        g2o::HyperGraph::Vertex* v_before, g2o::HyperGraph::Vertex* v) {
        // add edge linking last and current frame
        auto nav_edge = new optimize::internal::se3::navigation_pose_opt_edge();

        if (!keyfrm_before->nav_state_.valid ||
            !keyfrm->nav_state_.valid ) {
                SPDLOG_ERROR("Keyframe does not have valid navigation state, but still navigation edge is computed");
        }

        auto rel_navigation = computeRelativeRotPos(keyfrm_before->nav_state_, keyfrm->nav_state_);
        //rel_navigation.cam_translation(0,0) = 200.0;
        const double dt = keyfrm->timestamp_ - keyfrm_before->timestamp_;
        nav_edge->setMeasurement(rel_navigation);
        // todo: make correct
        const double distance = rel_navigation.cam_translation.norm();
        SPDLOG_DEBUG("Distance between navigation frames in meters: {0} and time {1}",
            distance, dt);
        const double absError = 0.005;

        const double distError = 0.005; // per meter
        const double timeError = 0.0005; // per second

        const double absErrorTime = 0.0005;
        const double rotError = 0.0001; // per meter
        const double rotErrorTime = 0.0001; // per second

        // 5 cm error per meter
        const double sigma_sq_dist = std::pow(distance * distError +
            dt * timeError +
            absError, 2);

        const double sigma_sq_rot = std::pow(distance * rotError +
            dt * rotErrorTime +
            absErrorTime, 2);

        // check how many landmarks are shared between these keyframes
        //data::keyframe * kf = nullptr;
      /*  unsigned long shared_obs = 0;
        for( auto * lm: keyfrm_before->get_landmarks()) {
            if (lm == nullptr) {
                continue;
            }
            auto const& obs = lm->get_observations();
            // check if 2nd keyframe is observed
            if (obs.find(const_cast<TFrameA*>(keyfrm)) != obs.end()) {
                shared_obs++;
            }
        }

        SPDLOG_DEBUG("Found {0} shared landmarks between kf {1} and {2}",
            shared_obs, keyfrm_before->id_, keyfrm->id_);

        double noLandmarkBonus = 1.0;
        if (shared_obs == 0) {
            noLandmarkBonus = 5.0;
        }*/

        // information matrix needs 1/sigma_sq

        Mat66_t info_matrix = Mat66_t::Identity();

        // translations
        info_matrix(0,0) = 1.0/sigma_sq_dist;
        info_matrix(1,1) = 1.0/sigma_sq_dist;
        info_matrix(2,2) = 1.0/sigma_sq_dist;

        // rotations
        info_matrix(3,3) = 1.0/sigma_sq_rot;
        info_matrix(4,4) = 1.0/sigma_sq_rot;
        info_matrix(5,5) = 1.0/sigma_sq_rot;

        nav_edge->setInformation(info_matrix);
        nav_edge->setVertex(0, v_before);
        nav_edge->setVertex(1, v);

        return nav_edge;
    }
}
}

#endif