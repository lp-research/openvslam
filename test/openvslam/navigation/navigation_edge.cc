
#include "helper/keypoint.h"
#include "helper/landmark.h"

#include "openvslam/type.h"
#include "openvslam/navigation/navigation.h"
#include "openvslam/data/navigation_state.h"
#include "openvslam/util/transformation.h"
#include "openvslam/util/trigonometric.h"
#include "openvslam/util/converter.h"

#include "openvslam/type.h"
#include "openvslam/optimize/internal/landmark_vertex.h"
#include "openvslam/optimize/internal/se3/shot_vertex.h"
#include "openvslam/data/navigation_state.h"
#include "openvslam/util/transformation.h"

#include <g2o/core/base_binary_edge.h>
#include <g2o/core/base_unary_edge.h>


#include <gtest/gtest.h>

#include <random>

using namespace openvslam;
using namespace openvslam::optimize::internal::se3;

namespace {
    g2o::Vector6 computeErr(g2o::SE3Quat const& v1_cw, g2o::SE3Quat const& v2_cw, navigation_state const& measurement) {
        auto residualRot_cw = v1_cw.inverse() * v2_cw;

        // convert navigation data into the camera frame
        // because the pose vertices are in the camera frame
        ::g2o::SE3Quat nav_delta_cw(measurement.cam_rotation,
            // the next line probably wrong
            - (residualRot_cw.rotation().inverse() * measurement.cam_translation));

        // V1^T * V2 = Res | * Res^T
        // Res^T * V1^T * V2 = 0
        auto res = (nav_delta_cw.inverse()) * (v1_cw.inverse() * v2_cw);
        res.normalizeRotation();

        // for some reason I don't understand, the transformation of the translation is not working properly
        // with the above formula, probably because the translation in nav_delta_cw is not initialized with the
        // proper coordinate frame.
        // In the meantime, just compute the translation error in the global frame by hand
        ::g2o::Vector3 v1_t_wc = - (v1_cw.rotation().inverse() * v1_cw.translation());
        ::g2o::Vector3 v2_t_wc = - (v2_cw.rotation().inverse() * v2_cw.translation());
        ::g2o::Vector3 trans_res = -measurement.cam_translation + (v2_t_wc - v1_t_wc);

        // contains the position and x,y,z components of the quaternion which
        // should be close to zero if fitted vertex is close to the correct
        // position and orientation
        g2o::Vector6 error = res.toMinimalVector();
        error[0] = trans_res.x();
        error[1] = trans_res.y();
        error[2] = trans_res.z();

        return error;
    }
}

TEST(navigation_edge, no_movement) {

    Mat44_t pose1_cw = Mat44_t::Identity();
    pose1_cw(0, 3) = 4.0;
    Mat44_t pose2_cw = pose1_cw;

    auto pose1_shot = util::converter::to_g2o_SE3(pose1_cw);
    auto pose2_shot = util::converter::to_g2o_SE3(pose2_cw);

    navigation_state nav_state;

    auto err = computeErr(pose1_shot, pose2_shot, nav_state);

    // we expect no error here
    ASSERT_NEAR(0, err.norm(), 0.01);
}

TEST(navigation_edge, movement_forward) {
    Mat44_t pose1_cw = Mat44_t::Identity();
    pose1_cw(0, 3) = 4.0;
    // moved forward by 1 meter
    Mat44_t pose2_cw = pose1_cw;
    // homongenouse coords, so this is 1 meter right
    pose1_cw(0, 3) = 3.0;

    auto pose1_shot = util::converter::to_g2o_SE3(pose1_cw);
    auto pose2_shot = util::converter::to_g2o_SE3(pose2_cw);

    navigation_state nav_state;

    auto err = computeErr(pose1_shot, pose2_shot, nav_state);

    // we expect an error here
    std::cout << "err " << err << std::endl;
    ASSERT_GE(err.norm(), 0.1);
}

TEST(navigation_edge, movement_forward_also_nav) {
    Mat44_t pose1_cw = Mat44_t::Identity();
    pose1_cw(0, 3) = 4.0;
    // moved forward by 1 meter
    Mat44_t pose2_cw = pose1_cw;
    // homongenouse coords, so this is 1 meter right
    pose2_cw(0, 3) = 3.0;

    auto pose1_shot = util::converter::to_g2o_SE3(pose1_cw);
    auto pose2_shot = util::converter::to_g2o_SE3(pose2_cw);

    navigation_state nav_state;
    nav_state.cam_translation(0) = 1.0;

    auto err = computeErr(pose1_shot, pose2_shot, nav_state);

    // we expect no error here
    std::cout << "err " << err << std::endl;
    ASSERT_NEAR(0.0, err.norm(), 0.1);
}

TEST(navigation_edge, movement_rotation_also_nav) {

    Vec3_t camPos_nav1 = Vec3_t(5.0, 6.0, 7.0);
    Vec3_t camPos_nav2 = Vec3_t(5.0, 6.0, 7.0);

    Mat33_t rot_cw_90;
    rot_cw_90 = Eigen::AngleAxis(-double(util::toRad(90.0f)),
        // this is 90 degrees
     -Vec3_t::UnitY());

    Mat33_t rot_cw_180;
    rot_cw_180 = Eigen::AngleAxis(-double(util::toRad(180.0f)),
        // this is 180 degrees to the right
     -Vec3_t::UnitY());

    Mat44_t pose1_cw = util::navDataToCameraPose(rot_cw_90, camPos_nav1);
    Mat44_t pose2_cw = util::navDataToCameraPose(rot_cw_180, camPos_nav2);

    navigation_state nav_state;
    nav_state.cam_rotation = rot_cw_90;
    nav_state.cam_translation = camPos_nav2 - camPos_nav1;

    auto pose1_shot = util::converter::to_g2o_SE3(pose1_cw);
    auto pose2_shot = util::converter::to_g2o_SE3(pose2_cw);

    auto err = computeErr(pose1_shot, pose2_shot, nav_state);

    // we expect no error here
    std::cout << "err " << err << std::endl;
    ASSERT_NEAR(0.0, err.norm(), 0.1);
}


TEST(navigation_edge, movement_rotation_translation_also_nav) {

    Vec3_t camPos_nav1 = Vec3_t(5.0, 6.0, 7.0);
    Vec3_t camPos_nav2 = Vec3_t(5.0, 6.0, 5.0);

    Mat33_t rot_cw_90;
    rot_cw_90 = Eigen::AngleAxis(-double(util::toRad(90.0f)),
        // this is 90 degrees
     -Vec3_t::UnitY());

    Mat33_t rot_cw_180;
    rot_cw_180 = Eigen::AngleAxis(-double(util::toRad(180.0f)),
        // this is 180 degrees to the right
     -Vec3_t::UnitY());

    Mat44_t pose1_cw = util::navDataToCameraPose(rot_cw_90, camPos_nav1);
    Mat44_t pose2_cw = util::navDataToCameraPose(rot_cw_180, camPos_nav2);

    navigation_state nav_state;
    nav_state.cam_rotation = rot_cw_90;
    nav_state.cam_translation = camPos_nav2 - camPos_nav1;

    auto pose1_shot = util::converter::to_g2o_SE3(pose1_cw);
    auto pose2_shot = util::converter::to_g2o_SE3(pose2_cw);

    auto err = computeErr(pose1_shot, pose2_shot, nav_state);

    // we expect no error here
    std::cout << "err " << err << std::endl;
    ASSERT_NEAR(0.0, err.norm(), 0.1);
}
