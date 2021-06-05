#include "openvslam/config.h"
#include "openvslam/system.h"
#include "openvslam/tracking_module.h"
#include "openvslam/mapping_module.h"
#include "openvslam/global_optimization_module.h"
#include "openvslam/camera/base.h"
#include "openvslam/data/landmark.h"
#include "openvslam/data/laser2d.h"
#include "openvslam/data/map_database.h"
#include "openvslam/data/bow_database.h"
#include "openvslam/feature/orb_extractor.h"
#include "openvslam/match/projection.h"
#include "openvslam/util/image_converter.h"
#include "openvslam/util/yaml.h"
#include "openvslam/util/logging.h"
#include "openvslam/util/transformation.h"

#include <chrono>
#include <iterator>
#include <iostream>
#include <algorithm>
#include <unordered_map>

#include <spdlog/spdlog.h>

namespace {
using namespace openvslam;

feature::orb_params get_orb_params(const YAML::Node& yaml_node) {
    spdlog::debug("load ORB parameters");
    try {
        return feature::orb_params(yaml_node);
    }
    catch (const std::exception& e) {
        spdlog::error("failed in loading ORB parameters: {}", e.what());
        throw;
    }
}

double get_true_depth_thr(const camera::base* camera, const YAML::Node& yaml_node) {
    spdlog::debug("load depth threshold");
    double true_depth_thr = 40.0;
    if (camera->setup_type_ == camera::setup_type_t::Stereo || camera->setup_type_ == camera::setup_type_t::RGBD) {
        const auto depth_thr_factor = yaml_node["depth_threshold"].as<double>(40.0);
        true_depth_thr = camera->true_baseline_ * depth_thr_factor;
    }
    return true_depth_thr;
}

double get_depthmap_factor(const camera::base* camera, const YAML::Node& yaml_node) {
    spdlog::debug("load depthmap factor");
    double depthmap_factor = 1.0;
    if (camera->setup_type_ == camera::setup_type_t::RGBD) {
        depthmap_factor = yaml_node["depthmap_factor"].as<double>(depthmap_factor);
    }
    if (depthmap_factor < 0.) {
        throw std::runtime_error("depthmap_factor must be greater than 0");
    }
    return depthmap_factor;
}
} // unnamed namespace

namespace openvslam {

tracking_module::tracking_module(const std::shared_ptr<config>& cfg, system* system, data::map_database* map_db,
                                 data::bow_vocabulary* bow_vocab, data::bow_database* bow_db)
    : camera_(cfg->camera_), true_depth_thr_(get_true_depth_thr(camera_, cfg->yaml_node_)),
      depthmap_factor_(get_depthmap_factor(camera_, cfg->yaml_node_)),
      system_(system), map_db_(map_db), bow_vocab_(bow_vocab), bow_db_(bow_db),
      initializer_(cfg->camera_->setup_type_, cfg->laser_scanner_, map_db, bow_db, util::yaml_optional_ref(cfg->yaml_node_, "Initializer")),
      frame_tracker_(camera_, 10), relocalizer_(bow_db_), pose_optimizer_(),
      keyfrm_inserter_(cfg->camera_->setup_type_, true_depth_thr_, cfg->laser_scanner_, map_db, bow_db, 0, cfg->camera_->fps_),
      relocalize_with_navdata_(cfg->relocalize_with_navdata_),
      time_to_relocalize_(cfg->time_to_relocalize_) {
    spdlog::debug("CONSTRUCT: tracking_module");

    feature::orb_params orb_params = get_orb_params(cfg->yaml_node_["Feature"]);
    extractor_left_ = new feature::orb_extractor(orb_params);
    if (camera_->setup_type_ == camera::setup_type_t::Monocular) {
        ini_extractor_left_ = new feature::orb_extractor(orb_params);
        ini_extractor_left_->set_max_num_keypoints(orb_params.ini_max_num_keypts_);
    }
    if (camera_->setup_type_ == camera::setup_type_t::Stereo) {
        extractor_right_ = new feature::orb_extractor(orb_params);
    }
}

tracking_module::~tracking_module() {
    delete extractor_left_;
    extractor_left_ = nullptr;
    delete extractor_right_;
    extractor_right_ = nullptr;
    delete ini_extractor_left_;
    ini_extractor_left_ = nullptr;

    spdlog::debug("DESTRUCT: tracking_module");
}

void tracking_module::set_mapping_module(mapping_module* mapper) {
    mapper_ = mapper;
    keyfrm_inserter_.set_mapping_module(mapper);
}

void tracking_module::set_global_optimization_module(global_optimization_module* global_optimizer) {
    global_optimizer_ = global_optimizer;
}

void tracking_module::set_mapping_module_status(const bool mapping_is_enabled) {
    std::lock_guard<std::mutex> lock(mtx_mapping_);
    mapping_is_enabled_ = mapping_is_enabled;
}

bool tracking_module::get_mapping_module_status() const {
    std::lock_guard<std::mutex> lock(mtx_mapping_);
    return mapping_is_enabled_;
}

std::vector<cv::KeyPoint> tracking_module::get_initial_keypoints() const {
    return initializer_.get_initial_keypoints();
}

std::vector<int> tracking_module::get_initial_matches() const {
    return initializer_.get_initial_matches();
}

Mat44_t tracking_module::track_monocular_image(const cv::Mat& img, const double timestamp, const cv::Mat& mask,
    const navigation_state & navState) {
    const auto start = std::chrono::system_clock::now();

    // color conversion
    img_gray_ = img;
    util::convert_to_grayscale(img_gray_, camera_->color_order_);

    // create current frame object
    if (tracking_state_ == tracker_state_t::NotInitialized || tracking_state_ == tracker_state_t::Initializing) {
        curr_frm_ = data::frame(img_gray_, timestamp, ini_extractor_left_, bow_vocab_, camera_, true_depth_thr_, mask);
    }
    else {
        curr_frm_ = data::frame(img_gray_, timestamp, extractor_left_, bow_vocab_, camera_, true_depth_thr_, mask);
    }
    curr_frm_.nav_state_ = navState;

    track();

    const auto end = std::chrono::system_clock::now();
    elapsed_ms_ = std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count();

    return curr_frm_.cam_pose_cw_;
}

Mat44_t tracking_module::track_stereo_image(const cv::Mat& left_img_rect, const cv::Mat& right_img_rect,
    const double timestamp,
    const cv::Mat& mask,
    const navigation_state & nav_state,
    const navigation_state & navState_map,
    const data::laser2d & laser2d_data) {
    const auto start = std::chrono::system_clock::now();

    // color conversion
    img_gray_ = left_img_rect;
    cv::Mat right_img_gray = right_img_rect;
    util::convert_to_grayscale(img_gray_, camera_->color_order_);
    util::convert_to_grayscale(right_img_gray, camera_->color_order_);

    // create current frame object
    curr_frm_ = data::frame(img_gray_, right_img_gray, timestamp, extractor_left_, extractor_right_, bow_vocab_, camera_, true_depth_thr_, mask);

    curr_frm_.nav_state_ = nav_state;
    curr_frm_.nav_state_map_ = navState_map;
    curr_frm_.laser2d_ = laser2d_data;
    track();

    const auto end = std::chrono::system_clock::now();
    elapsed_ms_ = std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count();

    return curr_frm_.cam_pose_cw_;
}

Mat44_t tracking_module::track_RGBD_image(const cv::Mat& img, const cv::Mat& depthmap, const double timestamp, const cv::Mat& mask,
    const navigation_state & navState, const navigation_state & navState_map) {
    const auto start = std::chrono::system_clock::now();

    // color and depth scale conversion
    img_gray_ = img;
    cv::Mat img_depth = depthmap;
    util::convert_to_grayscale(img_gray_, camera_->color_order_);
    util::convert_to_true_depth(img_depth, depthmap_factor_);

    curr_frm_.nav_state_ = navState;
    curr_frm_.nav_state_map_ = navState_map;
    // create current frame object
    curr_frm_ = data::frame(img_gray_, img_depth, timestamp, extractor_left_, bow_vocab_, camera_, true_depth_thr_, mask);

    track();

    const auto end = std::chrono::system_clock::now();
    elapsed_ms_ = std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count();

    return curr_frm_.cam_pose_cw_;
}

void tracking_module::reset() {
    spdlog::info("resetting system");

    initializer_.reset();
    keyfrm_inserter_.reset();

    mapper_->request_reset();
    global_optimizer_->request_reset();

    bow_db_->clear();
    map_db_->clear();

    data::frame::next_id_ = 0;
    data::keyframe::next_id_ = 0;
    data::landmark::next_id_ = 0;

    last_reloc_frm_id_ = 0;

    tracking_state_ = tracker_state_t::NotInitialized;
}

void tracking_module::track() {
    if (tracking_state_ == tracker_state_t::NotInitialized) {
        tracking_state_ = tracker_state_t::Initializing;
    }

    SPDLOG_DEBUG("Received frame with {0} keypoints found", curr_frm_.keypts_.size());
    SPDLOG_DEBUG("Frame has valid navigation data: {0}", curr_frm_.nav_state_.valid);
    if (curr_frm_.nav_state_.valid) {
        SPDLOG_DEBUG("Frame navigation translation:\n{0}\n rotation:\n{1}",
            curr_frm_.nav_state_.cam_translation,
            curr_frm_.nav_state_.cam_rotation);
    }

    last_tracking_state_ = tracking_state_;

    // check if pause is requested
    check_and_execute_pause();
    while (is_paused()) {
        std::this_thread::sleep_for(std::chrono::microseconds(5000));
    }

    // LOCK the map database
    std::lock_guard<std::mutex> lock(data::map_database::mtx_database_);

    if (tracking_state_ == tracker_state_t::Initializing) {
        if (!initialize()) {
            return;
        }

        //std::cout << "VS: got " << map_db_->get_all_keyframes().size() << " keyframes" << std::endl; 
        // update the reference keyframe, local keyframes, and local landmarks
        update_local_map();

        // pass all of the keyframes to the mapping module
        const auto keyfrms = map_db_->get_all_keyframes();
        for (const auto keyfrm : keyfrms) {
            mapper_->queue_keyframe(keyfrm);
        }

        // state transition to Tracking mode
        tracking_state_ = tracker_state_t::Tracking;
    }
    else {

        // apply replace of landmarks observed in the last frame
        apply_landmark_replace();
        // update the camera pose of the last frame
        // because the mapping module might optimize the camera pose of the last frame's reference keyframe
        update_last_frame();

        // set the reference keyframe of the current frame
        curr_frm_.ref_keyfrm_ = ref_keyfrm_;

        auto succeeded = track_current_frame();
        //std::cout << "VS: track_current_frame " << succeeded << std::endl;

        // update the local map and optimize the camera pose of the current frame
        if (succeeded) {
            update_local_map();
            // todo: nav data not used because
            // the last frame is not useful in case we got lost tracking
            // so don't use it if we are in lost tracking mode
            succeeded = optimize_current_frame_with_local_map(tracking_state_ != tracker_state_t::Lost);
        }

        // update the motion model
        if (succeeded) {
            update_motion_model();
        }
        //std::cout << "VS: update_motion_model " << succeeded << std::endl;

        // state transition
        tracking_state_ = succeeded ? tracker_state_t::Tracking : tracker_state_t::Lost;

        // update the frame statistics
        map_db_->update_frame_statistics(curr_frm_, tracking_state_ == tracker_state_t::Lost);

        // if tracking is failed within 5.0 sec after initialization, reset the system
        constexpr float init_retry_thr = 5.0;
        if (tracking_state_ == tracker_state_t::Lost
            && curr_frm_.id_ - initializer_.get_initial_frame_id() < camera_->fps_ * init_retry_thr) {
            //spdlog::info("tracking lost within {} sec after initialization", init_retry_thr);
            spdlog::info("tracking lost within {} sec after initialization", init_retry_thr);
            system_->request_reset();
            return;
        }

        // show message if tracking has been lost
        if (last_tracking_state_ != tracker_state_t::Lost && tracking_state_ == tracker_state_t::Lost) {
            spdlog::info("tracking lost: frame {}", curr_frm_.id_);
            time_since_lost_ = 0.0f;
        }

        // check to insert the new keyframe derived from the current frame
        if (succeeded && new_keyframe_is_needed()) {
            insert_new_keyframe();
        }

        // tidy up observations
        for (unsigned int idx = 0; idx < curr_frm_.num_keypts_; ++idx) {
            if (curr_frm_.landmarks_.at(idx) && curr_frm_.outlier_flags_.at(idx)) {
                curr_frm_.landmarks_.at(idx) = nullptr;
            }
        }
    }

    // store the relative pose from the reference keyframe to the current frame
    // to update the camera pose at the beginning of the next tracking process
    if (curr_frm_.cam_pose_cw_is_valid_) {
        last_cam_pose_from_ref_keyfrm_ = curr_frm_.cam_pose_cw_ * curr_frm_.ref_keyfrm_->get_cam_pose_inv();
    }

    // update last frame
    last_frm_ = curr_frm_;
}

bool tracking_module::initialize() {
    // try to initialize with the current frame
    initializer_.initialize(curr_frm_);
    //std::cout << "VS: tracking module init" << std::endl << curr_frm_.cam_pose_cw_ << std::endl;

    // if map building was failed -> reset the map database
    if (initializer_.get_state() == module::initializer_state_t::Wrong) {
        // reset
        system_->request_reset();
        return false;
    }

    // if initializing was failed -> try to initialize with the next frame
    if (initializer_.get_state() != module::initializer_state_t::Succeeded) {
        return false;
    }

    // reset so we are ready for possible re-initialaztions during the tracking
    // with navigation data
    initializer_.reset();

    // succeeded
    return true;
}

bool tracking_module::track_current_frame() {
    bool succeeded = false;
    if (tracking_state_ == tracker_state_t::Tracking) {
        // Tracking mode
        // disabled direction navigation mode for now
        if (curr_frm_.nav_state_.valid && last_frm_.nav_state_.valid &&
            last_frm_.cam_pose_cw_is_valid_){
            succeeded = frame_tracker_.navigation_based_track(curr_frm_, last_frm_, velocity_);
            SPDLOG_DEBUG("Tried navigation-based tracking. Successful = {0}", succeeded);
        }
        if (!succeeded && velocity_is_valid_ && last_reloc_frm_id_ + 2 < curr_frm_.id_) {
            // if the motion model is valid
            succeeded = frame_tracker_.motion_based_track(curr_frm_, last_frm_, velocity_);
            SPDLOG_DEBUG("Tried motien-based tracking. Successful = {0}", succeeded);
        }
        if (!succeeded) {
            succeeded = frame_tracker_.bow_match_based_track(curr_frm_, last_frm_, ref_keyfrm_);
            SPDLOG_DEBUG("Tried bow-based tracking. Successful = {0}", succeeded);
        }
        if (!succeeded) {
            succeeded = frame_tracker_.robust_match_based_track(curr_frm_, last_frm_, ref_keyfrm_);
            SPDLOG_DEBUG("Tried robust-match tracking. Successful = {0}", succeeded);
        }
    }
    else {
        // Lost mode
        // try to relocalize
        time_since_lost_ += 1.0 / camera_->fps_;
        // if we have navigation data, try to use it to initialize
        // this is not well tested ..
/*        if (curr_frm_.nav_state_.valid && last_frm_.nav_state_.valid) {

            SPDLOG_DEBUG("Trying to relocalize with navigation data");
            Mat44_t last_pose_cw = util::toPose(curr_frm_.nav_state_.cam_rotation,
                -curr_frm_.nav_state_.cam_rotation * curr_frm_.nav_state_.cam_translation);
            last_frm_.cam_pose_cw_ = last_pose_cw;

            succeeded = frame_tracker_.navigation_based_track(curr_frm_, last_frm_, velocity_);
        }*/
        if (!succeeded) {
            SPDLOG_DEBUG("Trying to relocalize with keyframes only");
            succeeded = relocalizer_.relocalize(curr_frm_);
            if (succeeded) {
                // see if we can also use the relocalized
                // positon to update the local map
                update_local_map();
                // don't use previous navigation data for refit
                succeeded = optimize_current_frame_with_local_map(false);
            }

        }
        if (succeeded) {
            last_reloc_frm_id_ = curr_frm_.id_;
        }

        // use frame to prepare reinalization
        // only try this if mapping mode is enabled
        if (last_frm_.nav_state_map_.valid &&
            system_->mapping_module_is_enabled() && relocalize_with_navdata_) {
            // forward frame to initializer to be ready to resume tracking
            // by reinitialzation
            SPDLOG_DEBUG("Trying reinitialization with current frame");
            bool tryReinitialize = !succeeded && (time_since_lost_ > time_to_relocalize_);

            // if relocating fails reinitialize
            if (tryReinitialize) {
                initializer_.initialize(curr_frm_, true, tryReinitialize);
                SPDLOG_DEBUG("Trying reinitialization of map with navigation data");

                // map building failed ...
                if (initializer_.get_state() == module::initializer_state_t::Wrong) {
                    // reset
                    SPDLOG_DEBUG("Resetting reinitializer because map is wrong");
                    initializer_.reset();
                    return false;
                }

                if (initializer_.get_state() == module::initializer_state_t::Succeeded) {
                    std::cout << "Reinitialization of map with navigation data successful" << std::endl;
                    // reset initializer so we are ready for the next relocalization
                    initializer_.reset();
                    first_keyframe_after_reinitializaiton_ = true;
                    return true;
                }
            }

            // in any case, reset the initializer for next usage if it is not ready
            // even though we did not use it yet
            if (initializer_.get_state() == module::initializer_state_t::Wrong ||
                initializer_.get_state() == module::initializer_state_t::Succeeded) {
                initializer_.reset();
            }
        }
    }
    return succeeded;
}

void tracking_module::update_motion_model() {
    if (last_frm_.cam_pose_cw_is_valid_) {
        Mat44_t last_frm_cam_pose_wc = Mat44_t::Identity();
        last_frm_cam_pose_wc.block<3, 3>(0, 0) = last_frm_.get_rotation_inv();
        last_frm_cam_pose_wc.block<3, 1>(0, 3) = last_frm_.get_cam_center();
        velocity_is_valid_ = true;
        velocity_ = curr_frm_.cam_pose_cw_ * last_frm_cam_pose_wc;
    }
    else {
        velocity_is_valid_ = false;
        velocity_ = Mat44_t::Identity();
    }
}

void tracking_module::apply_landmark_replace() {
    for (unsigned int idx = 0; idx < last_frm_.num_keypts_; ++idx) {
        auto lm = last_frm_.landmarks_.at(idx);
        if (!lm) {
            continue;
        }

        auto replaced_lm = lm->get_replaced();
        if (replaced_lm) {
            last_frm_.landmarks_.at(idx) = replaced_lm;
        }
    }
}

void tracking_module::update_last_frame() {
    auto last_ref_keyfrm = last_frm_.ref_keyfrm_;
    if (!last_ref_keyfrm) {
        return;
    }
    last_frm_.set_cam_pose(last_cam_pose_from_ref_keyfrm_ * last_ref_keyfrm->get_cam_pose());
}

bool tracking_module::optimize_current_frame_with_local_map(bool use_last_frame) {
    // acquire more 2D-3D matches by reprojecting the local landmarks to the current frame
    search_local_landmarks();

    // optimize the pose
    // does not use information from the last frame yet
    if (use_last_frame) {
        pose_optimizer_.optimize(curr_frm_, last_frm_);
    } else {
        pose_optimizer_.optimize(curr_frm_);
    }


    // count up the number of tracked landmarks
    num_tracked_lms_ = 0;
    //std::cout << "VS: Total number of keypoints in frame: " << curr_frm_.num_keypts_ << std::endl;
    for (unsigned int idx = 0; idx < curr_frm_.num_keypts_; ++idx) {
        auto lm = curr_frm_.landmarks_.at(idx);
        if (!lm) {
            continue;
        }

        if (!curr_frm_.outlier_flags_.at(idx)) {
            // the observation has been considered as inlier in the pose optimization
            assert(lm->has_observation());
            // count up
            ++num_tracked_lms_;
            // increment the number of tracked frame
            lm->increase_num_observed();
        }
        else {
            // the observation has been considered as outlier in the pose optimization
            // remove the observation
            curr_frm_.landmarks_.at(idx) = nullptr;
        }
    }

    constexpr unsigned int num_tracked_lms_thr = 20;

    // if recently relocalized, use the more strict threshold
    if (curr_frm_.id_ < last_reloc_frm_id_ + camera_->fps_ && num_tracked_lms_ < 2 * num_tracked_lms_thr) {
        SPDLOG_DEBUG("local map tracking failed: {} matches < {}", num_tracked_lms_, 2 * num_tracked_lms_thr);
        return false;
    }

    // check the threshold of the number of tracked landmarks
    if (num_tracked_lms_ < num_tracked_lms_thr) {
        SPDLOG_DEBUG("local map tracking failed: {} matches < {}", num_tracked_lms_, num_tracked_lms_thr);
        return false;
    }

    return true;
}

void tracking_module::update_local_map() {
    update_local_keyframes();
    update_local_landmarks();

    map_db_->set_local_landmarks(local_landmarks_);
}

void tracking_module::update_local_keyframes() {
    constexpr unsigned int max_num_local_keyfrms = 60;

    // count the number of sharing landmarks between the current frame and each of the neighbor keyframes
    // key: keyframe, value: number of sharing landmarks
    std::unordered_map<data::keyframe*, unsigned int> keyfrm_weights;
    for (unsigned int idx = 0; idx < curr_frm_.num_keypts_; ++idx) {
        auto lm = curr_frm_.landmarks_.at(idx);
        if (!lm) {
            continue;
        }
        if (lm->will_be_erased()) {
            curr_frm_.landmarks_.at(idx) = nullptr;
            continue;
        }

        const auto observations = lm->get_observations();
        for (auto obs : observations) {
            ++keyfrm_weights[obs.first];
        }
    }

    if (keyfrm_weights.empty()) {
        return;
    }

    // set the aforementioned keyframes as local keyframes
    // and find the nearest keyframe
    unsigned int max_weight = 0;
    data::keyframe* nearest_covisibility = nullptr;

    local_keyfrms_.clear();
    local_keyfrms_.reserve(4 * keyfrm_weights.size());

    for (auto& keyfrm_weight : keyfrm_weights) {
        auto keyfrm = keyfrm_weight.first;
        const auto weight = keyfrm_weight.second;

        if (keyfrm->will_be_erased()) {
            continue;
        }

        local_keyfrms_.push_back(keyfrm);

        // avoid duplication
        keyfrm->local_map_update_identifier = curr_frm_.id_;

        // update the nearest keyframe
        if (max_weight < weight) {
            max_weight = weight;
            nearest_covisibility = keyfrm;
        }
    }

    // add the second-order keyframes to the local landmarks
    auto add_local_keyframe = [this](data::keyframe* keyfrm) {
        if (!keyfrm) {
            return false;
        }
        if (keyfrm->will_be_erased()) {
            return false;
        }
        // avoid duplication
        if (keyfrm->local_map_update_identifier == curr_frm_.id_) {
            return false;
        }
        keyfrm->local_map_update_identifier = curr_frm_.id_;
        local_keyfrms_.push_back(keyfrm);
        return true;
    };
    for (auto iter = local_keyfrms_.cbegin(), end = local_keyfrms_.cend(); iter != end; ++iter) {
        if (max_num_local_keyfrms < local_keyfrms_.size()) {
            break;
        }

        auto keyfrm = *iter;

        // covisibilities of the neighbor keyframe
        const auto neighbors = keyfrm->graph_node_->get_top_n_covisibilities(10);
        for (auto neighbor : neighbors) {
            if (add_local_keyframe(neighbor)) {
                break;
            }
        }

        // children of the spanning tree
        const auto spanning_children = keyfrm->graph_node_->get_spanning_children();
        for (auto child : spanning_children) {
            if (add_local_keyframe(child)) {
                break;
            }
        }

        // parent of the spanning tree
        auto parent = keyfrm->graph_node_->get_spanning_parent();
        add_local_keyframe(parent);
    }

    // update the reference keyframe with the nearest one
    if (nearest_covisibility) {
        ref_keyfrm_ = nearest_covisibility;
        curr_frm_.ref_keyfrm_ = ref_keyfrm_;
    }
}

void tracking_module::update_local_landmarks() {
    local_landmarks_.clear();

    for (auto keyfrm : local_keyfrms_) {
        const auto lms = keyfrm->get_landmarks();

        for (auto lm : lms) {
            if (!lm) {
                continue;
            }
            if (lm->will_be_erased()) {
                continue;
            }

            // avoid duplication
            if (lm->identifier_in_local_map_update_ == curr_frm_.id_) {
                continue;
            }
            lm->identifier_in_local_map_update_ = curr_frm_.id_;

            local_landmarks_.push_back(lm);
        }
    }
}

void tracking_module::search_local_landmarks() {
    // select the landmarks which can be reprojected from the ones observed in the current frame
    //std::cout << "VS: landmarks in current frame: " << curr_frm_.landmarks_.size() << std::endl;

    for (auto lm : curr_frm_.landmarks_) {
        if (!lm) {
            continue;
        }
        if (lm->will_be_erased()) {
            continue;
        }

        // this landmark cannot be reprojected
        // because already observed in the current frame
        lm->is_observable_in_tracking_ = false;
        lm->identifier_in_local_lm_search_ = curr_frm_.id_;

        // this landmark is observable from the current frame
        lm->increase_num_observable();
    }

    bool found_proj_candidate = false;
    // temporary variables
    Vec2_t reproj;
    float x_right;
    unsigned int pred_scale_level;
    for (auto lm : local_landmarks_) {
        // avoid the landmarks which cannot be reprojected (== observed in the current frame)
        if (lm->identifier_in_local_lm_search_ == curr_frm_.id_) {
            continue;
        }
        if (lm->will_be_erased()) {
            continue;
        }

        // check the observability
        if (curr_frm_.can_observe(lm, 0.5, reproj, x_right, pred_scale_level)) {
            // pass the temporary variables
            lm->reproj_in_tracking_ = reproj;
            lm->x_right_in_tracking_ = x_right;
            lm->scale_level_in_tracking_ = pred_scale_level;

            // this landmark can be reprojected
            lm->is_observable_in_tracking_ = true;

            // this landmark is observable from the current frame
            lm->increase_num_observable();

            found_proj_candidate = true;
        }
        else {
            // this landmark cannot be reprojected
            lm->is_observable_in_tracking_ = false;
        }
    }

    if (!found_proj_candidate) {
        return;
    }

    // acquire more 2D-3D matches by projecting the local landmarks to the current frame
    match::projection projection_matcher(0.8);
    const float margin = (curr_frm_.id_ < last_reloc_frm_id_ + 2)
                             ? 20.0
                             : ((camera_->setup_type_ == camera::setup_type_t::RGBD)
                                    ? 10.0
                                    : 5.0);
    projection_matcher.match_frame_and_landmarks(curr_frm_, local_landmarks_, margin);
}

bool tracking_module::new_keyframe_is_needed() const {
    if (!mapping_is_enabled_) {
        return false;
    }

    // cannnot insert the new keyframe in a second after relocalization
    const auto num_keyfrms = map_db_->get_num_keyframes();
    if (camera_->fps_ < num_keyfrms && curr_frm_.id_ < last_reloc_frm_id_ + camera_->fps_) {
        return false;
    }

    // check the new keyframe is needed
    return keyfrm_inserter_.new_keyframe_is_needed(curr_frm_, num_tracked_lms_, *ref_keyfrm_, first_keyframe_after_reinitializaiton_);
}

void tracking_module::insert_new_keyframe() {
    // insert the new keyframe
    const auto ref_keyfrm = keyfrm_inserter_.insert_new_keyframe(curr_frm_);
    SPDLOG_DEBUG("Created new keyframe is valid: {0}", ref_keyfrm != nullptr);

    if (ref_keyfrm != nullptr) {
        first_keyframe_after_reinitializaiton_ = false;
    }

    // set the reference keyframe with the new keyframe
    ref_keyfrm_ = ref_keyfrm ? ref_keyfrm : ref_keyfrm_;
    curr_frm_.ref_keyfrm_ = ref_keyfrm_;

    SPDLOG_DEBUG("Inserted new keyframe with timestamp {0} with at pose:\n{1}\n pos:{2}\n rot_cw:\n {3}",
        ref_keyfrm->timestamp_,
        util::to_string(ref_keyfrm_->get_cam_pose()),
        ref_keyfrm_->get_cam_center(), ref_keyfrm_->get_rotation());
}

void tracking_module::request_pause() {
    std::lock_guard<std::mutex> lock1(mtx_pause_);
    pause_is_requested_ = true;
}

bool tracking_module::pause_is_requested() const {
    std::lock_guard<std::mutex> lock(mtx_pause_);
    return pause_is_requested_;
}

bool tracking_module::is_paused() const {
    std::lock_guard<std::mutex> lock(mtx_pause_);
    return is_paused_;
}

void tracking_module::resume() {
    std::lock_guard<std::mutex> lock(mtx_pause_);

    is_paused_ = false;
    pause_is_requested_ = false;

    spdlog::info("resume tracking module");
}

bool tracking_module::check_and_execute_pause() {
    std::lock_guard<std::mutex> lock(mtx_pause_);
    if (pause_is_requested_) {
        is_paused_ = true;
        spdlog::info("pause tracking module");
        return true;
    }
    else {
        return false;
    }
}

} // namespace openvslam
