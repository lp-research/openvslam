#ifndef OPENVSLAM_TRACKER_STATE_MODULE_H
#define OPENVSLAM_TRACKER_STATE_MODULE_H

namespace openvslam {

// tracker state
enum class tracker_state_t {
    NotInitialized,
    Initializing,
    Tracking,
    Lost
};
}

#endif