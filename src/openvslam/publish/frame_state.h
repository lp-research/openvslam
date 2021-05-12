#ifndef OPENVSLAM_FRAME_STATE_H
#define OPENVSLAM_FRAME_STATE_H

#include "openvslam/tracker_state.h"

#include <cstddef>
namespace openvslam {

namespace publish {
struct frame_state {
    size_t curr_keypts;
    tracker_state_t tracker_state;
};
}
}

#endif