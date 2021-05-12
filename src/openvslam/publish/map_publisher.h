#ifndef OPENVSLAM_PUBLISH_MAP_PUBLISHER_H
#define OPENVSLAM_PUBLISH_MAP_PUBLISHER_H

#include "openvslam/type.h"
#include "openvslam/openvslam_export.h"
#include "openvslam/data/occupancy_map_info.h"

#include <functional>
#include <mutex>
#include <memory>
#include <unordered_map>

namespace openvslam {

class config;

namespace module {
class occupancy_map_exporter;
}

namespace data {
class keyframe;
class landmark;
class map_database;
} // namespace data

namespace publish {

class OPENVSLAM_EXPORT map_publisher {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    typedef std::function<void(std::unordered_map<unsigned int, data::landmark*> const&)> LambdaLandmark;
    typedef std::vector<Mat44_t,Eigen::aligned_allocator<Mat44_t>> nav_poses_vector;

    /**
     * Constructor
     * @param cfg
     * @param map_db
     */
    map_publisher(const std::shared_ptr<config>& cfg, std::shared_ptr<module::occupancy_map_exporter>& map_exporter,
        data::map_database* map_db);

    /**
     * Destructor
     */
    virtual ~map_publisher();

    /**
     * Set current camera pose
     * NOTE: should be accessed from tracker thread
     * @param cam_pose_cw
     */
    void set_current_cam_pose(const Mat44_t& cam_pose_cw);

    /**
     * Get current camera pose
     * NOTE: should be accessed from viewer thread
     * @return
     */
    Mat44_t get_current_cam_pose();

    void set_current_nav_pose(const Mat44_t& nav_pose_cw);
    Mat44_t get_current_nav_pose();

    void add_intermediate_nav_pose(const Mat44_t& nav_pose_cw);
    nav_poses_vector get_intermediate_nav_poses();

    void set_current_ref_pose(const Mat44_t& ref_pose_cw);
    void set_current_ref_pose(const navigation_state & nav_state);
    Mat44_t get_current_ref_pose();

    /**
     * Get all keyframes
     * @param all_keyfrms
     * @return number of keyframes in map
     */
    unsigned int get_keyframes(std::vector<data::keyframe*>& all_keyfrms);

    /**
     * Get all landmarks and local landmarks
     * @param all_landmarks
     * @param local_landmarks
     * @return number of landmarks in map
     */
    unsigned int get_landmarks(std::vector<data::landmark*>& all_landmarks,
                               std::set<data::landmark*>& local_landmarks);

    void execute_on_landmarks( LambdaLandmark & lmd );

    // todo: add anchors

    unsigned long get_landmarks_count();

    unsigned long get_keyframe_count();

    // todo: can also select bounding box
    // todo: get buffer size needed
    // todo: fill buffer and return parameters like map center, offset, cell size etc

    unsigned long occupancy_map_export_size_required();

    data::occupancy_map_info occupancy_map_export(int8_t * map_data, unsigned long map_data_size);

    bool export_to_csv(std::string csv_filename);

private:
    //! config
    std::shared_ptr<config> cfg_;
    //! map database
    data::map_database* map_db_;

    std::shared_ptr<module::occupancy_map_exporter> map_exporter_;
    
    // list of navigation points which are not linked to any image
    // which are used for smoother visualization
    nav_poses_vector intermediate_nav_poses_;
    std::mutex intermediate_nav_poses_lock_;

    // -------------------------------------------
    //! mutex to access camera pose
    std::mutex mtx_cam_pose_;
    Mat44_t cam_pose_cw_ = Mat44_t::Identity();

    // -------------------------------------------
    //! mutex to access navigation pose
    std::mutex mtx_nav_pose_;
    Mat44_t nav_pose_cw_ = Mat44_t::Identity();

    std::mutex mtx_ref_pose_;
    Mat44_t ref_pose_cw_ = Mat44_t::Identity();
};

} // namespace publish
} // namespace openvslam

#endif // OPENVSLAM_PUBLISH_MAP_PUBLISHER_H
