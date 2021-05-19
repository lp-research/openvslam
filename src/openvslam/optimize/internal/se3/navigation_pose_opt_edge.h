#ifndef OPENVSLAM_OPTIMIZER_G2O_SE3_NAVIGATION_POSE_OPT_EDGE_H
#define OPENVSLAM_OPTIMIZER_G2O_SE3_NAVIGATION_POSE_OPT_EDGE_H

#include "openvslam/type.h"
#include "openvslam/optimize/internal/landmark_vertex.h"
#include "openvslam/optimize/internal/se3/shot_vertex.h"
#include "openvslam/data/navigation_state.h"
#include "openvslam/util/transformation.h"

#include <g2o/core/base_binary_edge.h>
#include <g2o/core/base_unary_edge.h>

#include <spdlog/spdlog.h>
#include <spdlog/fmt/ostr.h>

namespace openvslam {
namespace optimize {
namespace internal {
namespace se3 {

class navigation_pose_opt_edge final : public 

    ::g2o::BaseBinaryEdge<
        6, // 3d for position of the computed error, 3d for rotation
        navigation_state, // contains the relative change from frame 1 to frame 2
        shot_vertex, // vertex of last frame
        shot_vertex // vertex of this frame
        > {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    navigation_pose_opt_edge();

    bool read(std::istream& is) override;

    bool write(std::ostream& os) const override;

    void computeError() override;

    // todo: implement for faster processing
    //void linearizeOplus() override;
};

} // namespace se3
} // namespace g2o
} // namespace optimize
} // namespace openvslam

#endif // OPENVSLAM_OPTIMIZER_G2O_SE3_NAVIGATION_POSE_OPT_EDGE_H
