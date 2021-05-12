#include "helper/keypoint.h"
#include "helper/landmark.h"

#include "openvslam/type.h"
#include "openvslam/navigation/navigation.h"
#include "openvslam/data/navigation_state.h"
#include "openvslam/util/transformation.h"
#include "openvslam/util/trigonometric.h"

#include <gtest/gtest.h>

#include <random>

using namespace openvslam;

TEST(diff_poses, from_nav_pose) {
    navigation_state nav1;
    nav1.cam_translation = Vec3_t(2.0, 3.0, 1.0);

    navigation_state nav2;
    // moved by 10 com fordward in z axis !
    nav2.cam_translation = Vec3_t(2.0, 3.0, 1.1);

    Mat44_t camPose_cw = Mat44_t::Identity();
    Mat44_t camPosAfter_cw = navigation::applyRelativeMotionToPose(nav1, nav2, camPose_cw);

    auto rot_trans_after = util::cameraPoseToNavRotTrans(camPosAfter_cw);

    // should have been moved by 10 cm
    ASSERT_NEAR(0.1, rot_trans_after.second.z(), 0.001);
}

TEST(diff_poses, rotated_pose) {
    navigation_state nav1;
    nav1.cam_translation = Vec3_t(2.0, 3.0, 1.0);

    navigation_state nav2;
    // moved by 1 meter fordward in z axis !
    nav2.cam_translation = Vec3_t(2.0, 3.0, 2.0);

    Vec3_t camPos_global = Vec3_t(5.0, 6.0, 7.0);
    Mat33_t rot_cw;
    // rotated by 90 degrees to the left
    rot_cw = Eigen::AngleAxis(double(util::toRad(90.0f)), -Vec3_t::UnitY());
    Mat44_t camPose_cw = Mat44_t::Identity();

    camPose_cw.block<3, 3>(0, 0) = rot_cw;
    camPose_cw.block<3, 1>(0, 3) = -rot_cw * camPos_global;

    Mat44_t camPosAfter_cw = navigation::applyRelativeMotionToPose(nav1, nav2, camPose_cw);
    //Mat44_t camPosAfter_cw = camPose_cw;

    auto rot_trans_after = util::cameraPoseToNavRotTrans(camPosAfter_cw);

    std::cout << "rot after " << rot_trans_after.first << std::endl;
    std::cout << "trans after " << rot_trans_after.second << std::endl;

    // should have been moved by 1 meter in z
    ASSERT_NEAR(5.0, rot_trans_after.second.x(), 0.01);
    ASSERT_NEAR(6.0, rot_trans_after.second.y(), 0.01);
    ASSERT_NEAR(7.0 + 1.0, rot_trans_after.second.z(), 0.01);
}

TEST(diff_poses, rotated_pose_rotated_nav) {

    // rotated by 90 degrees to the right
    Vec3_t camPos_wc = Vec3_t(5.0, 6.0, 7.0);
    Mat33_t rot_cw;
    rot_cw = Eigen::AngleAxis( double(util::toRad(90.0f)), -Vec3_t::UnitY());
    Mat44_t camPose_cw = Mat44_t::Identity();

    navigation_state nav1;
    nav1.cam_rotation = rot_cw;
    nav1.cam_translation = Vec3_t(2.0, 3.0, 1.0);

    navigation_state nav2;
    // moved by 10 com fordward in z axis !
    nav1.cam_rotation = rot_cw;
    nav2.cam_translation = Vec3_t(2.0, 3.0, 1.1);

    camPose_cw.block<3, 3>(0, 0) = rot_cw;
    camPose_cw.block<3, 1>(0, 3) = -rot_cw * camPos_wc;

    Mat44_t camPosAfter_cw = navigation::applyRelativeMotionToPose(nav1, nav2, camPose_cw);
    //Mat44_t camPosAfter_cw = camPose_cw;

    auto rot_trans_after = util::cameraPoseToNavRotTrans(camPosAfter_cw);

    // should have been moved by 10 cm
    ASSERT_NEAR(5.0, rot_trans_after.second.x(), 0.01);
    ASSERT_NEAR(6.0, rot_trans_after.second.y(), 0.01);
    ASSERT_NEAR(7.0 + 0.1, rot_trans_after.second.z(), 0.01);
}

TEST(diff_poses, rotated_pose_rotated_nav_diff) {

    // rotated by 90 degrees to the right
    Vec3_t camPos_wc = Vec3_t(5.0, 6.0, 7.0);
    Mat33_t rot_cw_90;
    rot_cw_90 = Eigen::AngleAxis(-double(util::toRad(90.0f)),
        // this is 90 degrees
     -Vec3_t::UnitY());
    
    // points to +x: rotated right by 90 degrees
    std::cout << "unit z rot "<< rot_cw_90 * Vec3_t::UnitZ() << std::endl;

    Mat33_t rot_cw_180;
    rot_cw_180 = Eigen::AngleAxis(-double(util::toRad(180.0f)),
        // this is 180 degrees to the right
     -Vec3_t::UnitY());

    Mat44_t camPose_cw = Mat44_t::Identity();

    navigation_state nav1;
    nav1.cam_rotation = rot_cw_90;
    nav1.cam_translation = Vec3_t(2.0, 3.0, 1.0);

    navigation_state nav2;
    // moved by 4 meter fordward in z axis !
    nav2.cam_rotation = rot_cw_180;
    nav2.cam_translation = Vec3_t(2.0, 3.0, 5.0);

    camPose_cw.block<3, 3>(0, 0) = rot_cw_90;
    camPose_cw.block<3, 1>(0, 3) = -rot_cw_90 * camPos_wc;

    Mat44_t camPosAfter_cw = navigation::applyRelativeMotionToPose(nav1, nav2, camPose_cw);
    //Mat44_t camPosAfter_cw = camPose_cw;

    auto rot_trans_after = util::cameraPoseToNavRotTrans(camPosAfter_cw);

    // should have been moved by 4 meter
    ASSERT_NEAR(5.0, rot_trans_after.second.x(), 0.3);
    ASSERT_NEAR(6.0, rot_trans_after.second.y(), 0.3);
    ASSERT_NEAR(7.0 + 4.0, rot_trans_after.second.z(), 0.3);

    std::cout << "after rot " << rot_trans_after.first << std::endl;

    const Vec3_t rotatedUnitZ = rot_trans_after.first * Vec3_t::UnitZ();
    std::cout << "rotated unit z " << rotatedUnitZ << std::endl;

    // cam pose should now look 180* back so this should be -z direction
    ASSERT_NEAR(0.0,(rotatedUnitZ - (-Vec3_t::UnitZ())).norm(), 0.01);
/*
    ASSERT_NEAR(5.0, rot_trans_after.first(0,0), 0.3);
    ASSERT_NEAR(6.0, rot_trans_after.first(0,0), 0.3);
    ASSERT_NEAR(7.0 + 4.0, rot_trans_after.second.z(), 0.3);*/
}