#include "openvslam/config.h"
#include "openvslam/data/keyframe.h"
#include "openvslam/data/landmark.h"
#include "openvslam/data/map_database.h"
#include "openvslam/initialize/bearing_vector.h"
#include "openvslam/initialize/perspective.h"
#include "openvslam/initialize/perspective_nav.h"
#include "openvslam/match/area.h"
#include "openvslam/module/initializer.h"
#include "openvslam/optimize/global_bundle_adjuster.h"
#include "openvslam/util/transformation.h"
#include "openvslam/laser/laser_scanner_base.h"

#include <spdlog/spdlog.h>
#include <spdlog/fmt/ostr.h>
#include <iostream>

namespace openvslam {
namespace module {

initializer::initializer(const camera::setup_type_t setup_type,
                         laser::laser_scanner_base * laser_scanner,
                         data::map_database* map_db, data::bow_database* bow_db,
                         const YAML::Node& yaml_node)
    : setup_type_(setup_type), map_db_(map_db), bow_db_(bow_db),
      num_ransac_iters_(yaml_node["num_ransac_iterations"].as<unsigned int>(100)),
      min_num_triangulated_(yaml_node["num_min_triangulated_pts"].as<unsigned int>(50)),
      parallax_deg_thr_(yaml_node["parallax_deg_threshold"].as<float>(1.0)),
      reproj_err_thr_(yaml_node["reprojection_error_threshold"].as<float>(4.0)),
      num_ba_iters_(yaml_node["num_ba_iterations"].as<unsigned int>(20)),
      scaling_factor_(yaml_node["scaling_factor"].as<float>(1.0)),
      use_fixed_seed_(yaml_node["use_fixed_seed"].as<bool>(false)),
      min_movement_(yaml_node["min_movement"].as<float>(0.1)) {
    spdlog::debug("CONSTRUCT: module::initializer");
}

initializer::~initializer() {
    spdlog::debug("DESTRUCT: module::initializer");
}

void initializer::reset() {
    initializer_.reset(nullptr);
    state_ = initializer_state_t::NotReady;
    init_frm_id_ = 0;
}

initializer_state_t initializer::get_state() const {
    return state_;
}

std::vector<cv::KeyPoint> initializer::get_initial_keypoints() const {
    return init_frm_.keypts_;
}

std::vector<int> initializer::get_initial_matches() const {
    return init_matches_;
}

unsigned int initializer::get_initial_frame_id() const {
    return init_frm_id_;
}

bool initializer::initialize(data::frame& curr_frm, bool reinitialize, bool createNewMap) {
    switch (setup_type_) {
        case camera::setup_type_t::Monocular: {
            // construct an initializer if not constructed
            if (state_ == initializer_state_t::NotReady) {
                create_initializer(curr_frm);
                return false;
            }

            // try to initialize
            if (!try_initialize_for_monocular(curr_frm)) {
                // failed
                return false;
            }

            // create new map if succeeded
            if (createNewMap) {
                create_map_for_monocular(curr_frm, reinitialize);
            }
            break;
        }
        case camera::setup_type_t::Stereo:
        case camera::setup_type_t::RGBD: {
            state_ = initializer_state_t::Initializing;

            // try to initialize
            if (!try_initialize_for_stereo(curr_frm)) {
                // failed
                return false;
            }

            // create new map if succeeded
            if (createNewMap) {
                create_map_for_stereo(curr_frm, reinitialize);
            }
            break;
        }
        default: {
            throw std::runtime_error("Undefined camera setup");
        }
    }

    // check the state is succeeded or not
    if (state_ == initializer_state_t::Succeeded) {
        // don't change the initial frame if we just re-initialized
        if (!reinitialize) {
            init_frm_id_ = curr_frm.id_;
        }
        return true;
    }
    else {
        return false;
    }
}

void initializer::create_initializer(data::frame& curr_frm) {
    // set the initial frame
    init_frm_ = data::frame(curr_frm);

    // initialize the previously matched coordinates
    prev_matched_coords_.resize(init_frm_.undist_keypts_.size());
    for (unsigned int i = 0; i < init_frm_.undist_keypts_.size(); ++i) {
        prev_matched_coords_.at(i) = init_frm_.undist_keypts_.at(i).pt;
    }

    // initialize matchings (init_idx -> curr_idx)
    std::fill(init_matches_.begin(), init_matches_.end(), -1);

    // build a initializer
    initializer_.reset(nullptr);
    switch (init_frm_.camera_->model_type_) {
        case camera::model_type_t::Perspective:
        case camera::model_type_t::Fisheye: {

            if (curr_frm.nav_state_.valid) {
                spdlog::debug("Will intialize with navigation data input");
                initializer_ = std::unique_ptr<initialize::perspective_nav>(new initialize::perspective_nav(init_frm_,
                                                                                                num_ransac_iters_, min_num_triangulated_,
                                                                                                parallax_deg_thr_, reproj_err_thr_,
                                                                                                min_movement_));
            } else {
                spdlog::debug("Will intialize with perspective camera-only method");
                initializer_ = std::unique_ptr<initialize::perspective>(new initialize::perspective(init_frm_,
                                                                                                num_ransac_iters_, min_num_triangulated_,
                                                                                                parallax_deg_thr_, reproj_err_thr_,
                                                                                                use_fixed_seed_));
            }
            break;
        }
        case camera::model_type_t::Equirectangular: {
            initializer_ = std::unique_ptr<initialize::bearing_vector>(new initialize::bearing_vector(init_frm_,
                                                                                                      num_ransac_iters_, min_num_triangulated_,
                                                                                                      parallax_deg_thr_, reproj_err_thr_,
                                                                                                      use_fixed_seed_));
            break;
        }
    }

    state_ = initializer_state_t::Initializing;
}

bool initializer::try_initialize_for_monocular(data::frame& curr_frm) {
    assert(state_ == initializer_state_t::Initializing);

    match::area matcher(0.9, true);
    const auto num_matches = matcher.match_in_consistent_area(init_frm_, curr_frm, prev_matched_coords_, init_matches_, 100);

    if (num_matches < min_num_triangulated_) {
        // rebuild the initializer with the next frame
        reset();
        return false;
    }

    // try to initialize with the current frame
    assert(initializer_);
    return initializer_->initialize(curr_frm, init_matches_);
}

bool initializer::create_map_for_monocular(data::frame& curr_frm, bool reinitialize) {
    assert(state_ == initializer_state_t::Initializing);

    eigen_alloc_vector<Vec3_t> init_triangulated_pts;
    {
        assert(initializer_);
        init_triangulated_pts = initializer_->get_triangulated_pts();
        const auto is_triangulated = initializer_->get_triangulated_flags();

        // make invalid the matchings which have not been triangulated
        for (unsigned int i = 0; i < init_matches_.size(); ++i) {
            if (init_matches_.at(i) < 0) {
                continue;
            }
            if (is_triangulated.at(i)) {
                continue;
            }
            init_matches_.at(i) = -1;
        }

        // set the camera poses
        /*
        Quat_t quatRotMeas;
        quatRotMeas = Eigen::AngleAxis<double>((10.0 / 180.0) /3.1415,
            Eigen::Vector3d(1.0, 0.0, 0.0).normalized());
            */

        // todo: use navigation position of this frame
        Mat44_t init_pose_cw = Mat44_t::Identity();

        // check if nav data is available
        if (init_frm_.nav_state_.valid) {
            SPDLOG_DEBUG("Nav data for initial frame, pos: \n{0}\n rot:\n {1}\n",
                init_frm_.nav_state_.cam_rotation,
                init_frm_.nav_state_.cam_translation);
            init_pose_cw = util::toPose(init_frm_.nav_state_.cam_rotation,
                -init_frm_.nav_state_.cam_rotation * init_frm_.nav_state_.cam_translation);
            SPDLOG_DEBUG("Using nav data for initial frame: \n{0}", init_pose_cw);
        }

        init_frm_.set_cam_pose(init_pose_cw);

        Mat44_t cam_pose_cw = Mat44_t::Identity();
        if (curr_frm.nav_state_.valid) {
            SPDLOG_DEBUG("Nav data for current frame, pos: \n{0}\n rot:\n {1}\n",
                curr_frm.nav_state_.cam_rotation,
                curr_frm.nav_state_.cam_translation);
            cam_pose_cw = util::toPose(curr_frm.nav_state_.cam_rotation,
                -curr_frm.nav_state_.cam_rotation * curr_frm.nav_state_.cam_translation);

            SPDLOG_DEBUG("Using nav data for current keyframe: \n{0}",
                cam_pose_cw);
        } else {
            cam_pose_cw.block<3, 3>(0, 0) = //quatRotMeas.toRotationMatrix();
                initializer_->get_rotation_ref_to_cur();
            cam_pose_cw.block<3, 1>(0, 3) = initializer_->get_translation_ref_to_cur();
        }
        
        curr_frm.set_cam_pose(cam_pose_cw);

        // destruct the initializer
        initializer_.reset(nullptr);
    }

    // create initial keyframes
    auto init_keyfrm = new data::keyframe(init_frm_, map_db_, bow_db_);
    auto curr_keyfrm = new data::keyframe(curr_frm, map_db_, bow_db_);

    init_keyfrm->relocalized_ = reinitialize;

    // compute BoW representations
    init_keyfrm->compute_bow();
    curr_keyfrm->compute_bow();

    SPDLOG_DEBUG("Initial keyframe id {0} position: \n{1}\n rot_cw: \n{2}",
        init_keyfrm->id_, init_keyfrm->get_cam_center(), init_keyfrm->get_rotation());
    SPDLOG_DEBUG("Second keyframe id {0} position: \n{1}\n rot_cw: \n{2}",
        curr_keyfrm->id_, curr_keyfrm->get_cam_center(), curr_keyfrm->get_rotation());

    // add the keyframes to the map DB
    map_db_->add_keyframe(init_keyfrm);
    map_db_->add_keyframe(curr_keyfrm);

    if (reinitialize && init_keyfrm->nav_state_.valid
        && curr_keyfrm->nav_state_.valid) {
        // update connections so the reinitialized keyframes link to their
        // previous keyframes in time via the navigation data
        init_keyfrm->graph_node_->update_connections(map_db_);
        curr_keyfrm->graph_node_->update_connections(map_db_);
    }

    // update the frame statistics
    init_frm_.ref_keyfrm_ = init_keyfrm;
    curr_frm.ref_keyfrm_ = curr_keyfrm;

    if (!reinitialize) {
        // only call on first initialization
        // otherwise it will be called twice because
        // also tracking module calls it
        map_db_->update_frame_statistics(init_frm_, false);
        map_db_->update_frame_statistics(curr_frm, false);
    }

    // assign 2D-3D associations
    for (unsigned int init_idx = 0; init_idx < init_matches_.size(); init_idx++) {
        const auto curr_idx = init_matches_.at(init_idx);
        if (curr_idx < 0) {
            continue;
        }

        // shift all tringulated points into the navigation frame
        Vec3_t shiftedTriangluatedPoint = init_triangulated_pts.at(init_idx);
        if (init_frm_.nav_state_.valid) {

            Vec3_t transLm = /*curr_frm.nav_state_.cam_translation -*/
                init_frm_.nav_state_.cam_translation;
            Mat33_t transRot = /*util::relativeRotation(curr_frm.nav_state_.cam_rotation,*/
                init_frm_.nav_state_.cam_rotation;

            // rotate first back into world nav coordinate system (from cam frame)
            // and then shift by our world nav coordinate system
            // not sure why we have to tahe the transpose here !?!
            const Vec3_t shiftTmp = transRot.transpose() * shiftedTriangluatedPoint + transLm;
            shiftedTriangluatedPoint = shiftTmp;
        }

        // construct a landmark
        auto lm = new data::landmark(shiftedTriangluatedPoint, curr_keyfrm, map_db_);

        // set the assocications to the new keyframes
        init_keyfrm->add_landmark(lm, init_idx);
        curr_keyfrm->add_landmark(lm, curr_idx);
        lm->add_observation(init_keyfrm, init_idx);
        lm->add_observation(curr_keyfrm, curr_idx);

        // update the descriptor
        lm->compute_descriptor();
        // update the geometry
        lm->update_normal_and_depth();

        // set the 2D-3D assocications to the current frame
        curr_frm.landmarks_.at(curr_idx) = lm;
        curr_frm.outlier_flags_.at(curr_idx) = false;

        // add the landmark to the map DB
        map_db_->add_landmark(lm);
    }

    SPDLOG_DEBUG("Initial keyframe before global bundle adjuster: \n{0}", init_keyfrm->get_cam_pose());
    SPDLOG_DEBUG("Current keyframe before global bundle adjuster: \n{0}", curr_keyfrm->get_cam_pose());

    if (map_db_->get_all_landmarks().size() == 0) {
        spdlog::error("Database does not contain any landmarks after initialization. Considering initialization as failed");
        return false;
    }

    // global bundle adjustment
    // right now, this messes up our keyframe orientation and position
    // but it will also take some time to run if we already have many keyframes
    // as it fits *all* keyframes in the data base !
    // so probably better not to run it after initialization
    if (!reinitialize) {
        SPDLOG_DEBUG("Running bundle adjuster for initialization keyframes");
        const auto global_bundle_adjuster = optimize::global_bundle_adjuster(map_db_, num_ba_iters_, true);
        global_bundle_adjuster.optimize();
    }

    SPDLOG_DEBUG("Initial keyframe after global bundle adjuster: \n{0}", init_keyfrm->get_cam_pose());
    SPDLOG_DEBUG("Current keyframe after global bundle adjuster: \n{0}", curr_keyfrm->get_cam_pose());

    // scale the map so that the median of depths is 1.0
    const auto median_depth = init_keyfrm->compute_median_depth(init_keyfrm->camera_->model_type_ == camera::model_type_t::Equirectangular);

    if (!median_depth.has_value()) {
        spdlog::warn("Median depth cannot be computed");
        return false;
    }

    const auto inv_median_depth = 1.0 / median_depth.value();
    if (curr_keyfrm->get_num_tracked_landmarks(1) < min_num_triangulated_ && median_depth.value() < 0) {
        SPDLOG_DEBUG("seems to be wrong initialization, resetting");
        state_ = initializer_state_t::Wrong;
        return false;
    }

    // dont scale map when we have scale coming from INS system
    if (!init_frm_.nav_state_.valid && !curr_frm.nav_state_.valid) {
        scale_map(init_keyfrm, curr_keyfrm, inv_median_depth * scaling_factor_);
    }

    // update the current frame pose
    curr_frm.set_cam_pose(curr_keyfrm->get_cam_pose());

    // set the origin keyframe
    if (!reinitialize) {
        map_db_->origin_keyfrm_ = init_keyfrm;
    }

    SPDLOG_DEBUG("new map created with {} landmarks: frame {} to frame {}",
        map_db_->get_num_landmarks(), init_frm_.id_, curr_frm.id_);
    state_ = initializer_state_t::Succeeded;
    return true;
}

void initializer::scale_map(data::keyframe* init_keyfrm, data::keyframe* curr_keyfrm, const double scale) {
    // scaling keyframes
    Mat44_t cam_pose_cw = curr_keyfrm->get_cam_pose();
    cam_pose_cw.block<3, 1>(0, 3) *= scale;
    curr_keyfrm->set_cam_pose(cam_pose_cw);

    // scaling landmarks
    const auto landmarks = init_keyfrm->get_landmarks();
    for (auto lm : landmarks) {
        if (!lm) {
            continue;
        }
        lm->set_pos_in_world(lm->get_pos_in_world() * scale);
    }
}

bool initializer::try_initialize_for_stereo(data::frame& curr_frm) {
    assert(state_ == initializer_state_t::Initializing);
    // count the number of valid depths
    unsigned int num_valid_depths = std::count_if(curr_frm.depths_.begin(), curr_frm.depths_.end(),
                                                  [](const float depth) {
                                                      return 0 < depth;
                                                  });
    SPDLOG_DEBUG("Tried initialization for stereo image and got {0} valid depth matches, Required are {1}",
        num_valid_depths, min_num_triangulated_);
    return min_num_triangulated_ <= num_valid_depths;
}

bool initializer::create_map_for_stereo(data::frame& curr_frm, bool reinitialize) {
    assert(state_ == initializer_state_t::Initializing);

    Mat44_t init_pose_cw = Mat44_t::Identity();

    navigation_state nav_data;

    // cannot reinitialize if there is no nav data !
    if (reinitialize && !curr_frm.nav_state_map_.valid) {
        SPDLOG_DEBUG("Cannot reinit without nav map data\n");
        return false;
    }

    // only use the map coordinate frame for re-initialization
    if (reinitialize) {
        nav_data = curr_frm.nav_state_map_;
    }

    if (nav_data.valid) {
        SPDLOG_DEBUG("Nav data for initial stereo frame, rot: \n{0}\n pos:\n {1}\n",
            nav_data.cam_rotation,
            nav_data.cam_translation);
        init_pose_cw = util::toPose(nav_data.cam_rotation,
            -nav_data.cam_rotation * nav_data.cam_translation);
        SPDLOG_DEBUG("Using nav data for initial stereo frame: \n{0}", init_pose_cw);
    }

    // create an initial keyframe
    curr_frm.set_cam_pose(init_pose_cw);
    auto curr_keyfrm = new data::keyframe(curr_frm, map_db_, bow_db_);

    // add laser data
    if (curr_frm.laser2d_.is_valid()) {
        curr_keyfrm->set_laser_landmark(
            data::laser_landmark(curr_frm.laser2d_, curr_keyfrm, laser_scanner_)
         );

        spdlog::info("Added {0} laser measurements to initial keyframe", curr_frm.laser2d_.get_ranges().size());
    }

    // compute BoW representation
    curr_keyfrm->compute_bow();

    curr_keyfrm->relocalized_ = reinitialize;

    // add to the map DB
    map_db_->add_keyframe(curr_keyfrm);

    if (reinitialize && nav_data.valid) {
        // update connections so the reinitialized keyframe link to their
        // previous keyframe in time via the navigation data
        curr_keyfrm->graph_node_->update_connections(map_db_);
    }

    // update the frame statistics
    curr_frm.ref_keyfrm_ = curr_keyfrm;

    if (!reinitialize) {
        // only call on first initialization
        // otherwise it will be called twice because
        // also tracking module calls it
        map_db_->update_frame_statistics(curr_frm, false);
    }

    for (unsigned int idx = 0; idx < curr_frm.num_keypts_; ++idx) {
        // add a new landmark if tht corresponding depth is valid
        const auto z = curr_frm.depths_.at(idx);
        if (z <= 0) {
            continue;
        }

        // build a landmark
        // these landmark positions are already returned in world coordinates
        // and don't need to be transformed using navigation information
        Vec3_t pos_w = curr_frm.triangulate_stereo(idx);
        auto lm = new data::landmark(pos_w, curr_keyfrm, map_db_);

        // set the associations to the new keyframe
        lm->add_observation(curr_keyfrm, idx);
        curr_keyfrm->add_landmark(lm, idx);

        // update the descriptor
        lm->compute_descriptor();
        // update the geometry
        lm->update_normal_and_depth();

        // set the 2D-3D associations to the current frame
        curr_frm.landmarks_.at(idx) = lm;
        curr_frm.outlier_flags_.at(idx) = false;

        // add the landmark to the map DB
        map_db_->add_landmark(lm);
    }

    if (map_db_->get_all_landmarks().size() == 0) {
        spdlog::error("Database does not contain any landmarks after initialization. Considering initialization as failed");
        return false;
    }

    // set the origin keyframe
    if (!reinitialize) {
        map_db_->origin_keyfrm_ = curr_keyfrm;
    }

    SPDLOG_DEBUG("new map created with {} points: frame {}", map_db_->get_num_landmarks(), curr_frm.id_);
    state_ = initializer_state_t::Succeeded;
    return true;
}

} // namespace module
} // namespace openvslam
