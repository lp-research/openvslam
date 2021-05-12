#ifndef OPENVSLAM_INITIALIZE_PERSPECTIVE_NAV_H
#define OPENVSLAM_INITIALIZE_PERSPECTIVE_NAV_H

#include "openvslam/type.h"
#include "openvslam/initialize/base.h"
#include "openvslam/data/navigation_state.h"

#include <opencv2/core.hpp>

namespace openvslam {

namespace data {
class frame;
} // namespace data

namespace initialize {

/* triangulates the 3d position
   with the help of the navigation input
*/

class perspective_nav final : public base {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    perspective_nav() = delete;

    //! Constructor
    perspective_nav(const data::frame& ref_frm,
                const unsigned int num_ransac_iters, const unsigned int min_num_triangulated,
                const float parallax_deg_thr, const float reproj_err_thr,
                // minium movement between the two navigation frames used for initialization
                // in meters
                const float min_movement);

    //! Destructor
    ~perspective_nav() override;

    //! Initialize with the current frame
    bool initialize(const data::frame& cur_frm, const std::vector<int>& ref_matches_with_cur) override;

private:

    //! Get the camera matrix from the camera object
    static Mat33_t get_camera_matrix(camera::base* camera);

    //! camera matrix of the reference frame
    const Mat33_t ref_cam_matrix_;
    //! camera matrix of the current frame
    Mat33_t cur_cam_matrix_;

    navigation_state ref_nav_;

    //! movement between frames used for initialization
    //! in meters
    const double min_movement_;
};

} // namespace initialize
} // namespace openvslam

#endif // OPENVSLAM_INITIALIZE_PERSPECTIVE_H
