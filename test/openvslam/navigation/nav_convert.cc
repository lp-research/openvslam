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
    Vec3_t camPos_nav1 = Vec3_t(5.0, 6.0, 7.0);

    Mat33_t rot_cw_70;
    rot_cw_70 = Eigen::AngleAxis(-double(util::toRad(70.0f)),
        // this is 90 degrees
     (Vec3_t::UnitY() + Vec3_t::UnitZ()).normalized());

    Mat44_t pose1_cw;
    pose1_cw << 0.0, 1.0, 0.0, 3.0,
                -1.0, 0.0, 0.0, 4.0,
                0.0, 0.0, 1.0, 5.0,
                0.0, 0.0, 0.0, 1.0;

    auto [nav_rot, nav_pos] = util::cameraPoseToNavRotTrans(pose1_cw);
    Mat44_t pose1_cw_back = util::navDataToCameraPose(nav_rot, nav_pos);

    std::cout << "Orig" << std::endl << pose1_cw << std::endl;
    std::cout << "Back convert" << std::endl << pose1_cw_back << std::endl;

    //util::navDataToCameraPose(rot_cw_70, camPos_nav1);

}