#include "openvslam/data/landmark.h"
#include "openvslam/data/keyframe.h"
#include "openvslam/data/map_database.h"
#include "openvslam/publish/map_publisher.h"
#include "openvslam/util/transformation.h"
#include "openvslam/module/occupancy_map_exporter.h"

#include <spdlog/spdlog.h>

#include <iostream>
#include <fstream>

namespace openvslam {
namespace publish {

map_publisher::map_publisher(const std::shared_ptr<config>& cfg, std::shared_ptr<module::occupancy_map_exporter>& map_exporter,
    data::map_database* map_db)
    : cfg_(cfg), map_db_(map_db), map_exporter_(map_exporter) {
    spdlog::debug("CONSTRUCT: publish::map_publisher");
}

map_publisher::~map_publisher() {
    spdlog::debug("DESTRUCT: publish::map_publisher");
}

void map_publisher::set_current_cam_pose(const Mat44_t& cam_pose_cw) {
    std::lock_guard<std::mutex> lock(mtx_cam_pose_);
    cam_pose_cw_ = cam_pose_cw;
}

void map_publisher::set_current_nav_pose(const Mat44_t& nav_pose_cw) {
    std::lock_guard<std::mutex> lock(mtx_nav_pose_);
    nav_pose_cw_ = nav_pose_cw;
}

Mat44_t map_publisher::get_current_cam_pose() {
    std::lock_guard<std::mutex> lock(mtx_cam_pose_);
    return cam_pose_cw_;
}

Mat44_t map_publisher::get_current_nav_pose() {
    std::lock_guard<std::mutex> lock(mtx_nav_pose_);
    return nav_pose_cw_;
}

unsigned int map_publisher::get_keyframes(std::vector<data::keyframe*>& all_keyfrms) {
    all_keyfrms = map_db_->get_all_keyframes();
    return map_db_->get_num_keyframes();
}

unsigned int map_publisher::get_landmarks(std::vector<data::landmark*>& all_landmarks,
                                          std::set<data::landmark*>& local_landmarks) {
    all_landmarks = map_db_->get_all_landmarks();
    const auto _local_landmarks = map_db_->get_local_landmarks();
    local_landmarks = std::set<data::landmark*>(_local_landmarks.begin(), _local_landmarks.end());
    return map_db_->get_num_landmarks();
}

unsigned long map_publisher::get_landmarks_count(){
    return map_db_->get_num_landmarks();
}

unsigned long map_publisher::get_keyframe_count() {
    return map_db_->get_num_keyframes();
}

void map_publisher::execute_on_landmarks( LambdaLandmark & lmd ) {
    map_db_->execute_on_landmarks(lmd);
}

unsigned long map_publisher::occupancy_map_export_size_required() {
    return map_exporter_->estimate_buffer_size();
}

data::occupancy_map_info map_publisher::occupancy_map_export(int8_t * map_data, unsigned long map_data_size) {
    return map_exporter_->map_export(map_data, map_data_size);
}

bool map_publisher::export_to_csv(std::string csv_filename) {
    const auto all_landmarks = map_db_->get_all_landmarks();

    std::ofstream csv_out(csv_filename, std::ios::out | std::ios::trunc);

    if (!csv_out.is_open()) {
        return false;
    }

    // this export is in lpglobal fusion coords frame !

    csv_out << "LandmarkId;PosX; PosY; PosZ; NumObs" << std::endl;
    for (auto & lm: all_landmarks) {
        const auto world_pos = lm->get_pos_in_world();
        csv_out << lm->id_ << ";"
            << -world_pos.x() << ";"
            << -world_pos.y() << ";"
            << world_pos.z() << ";"
            << lm->num_observations() << std::endl;
    }

    csv_out.close();
    return true;
}


} // namespace publish
} // namespace openvslam
