#ifndef OPENVSLAM_UTIL_LOGGING_H
#define OPENVSLAM_UTIL_LOGGING_H

#include "openvslam/camera/base.h"

#include <opencv2/core.hpp>

#include <sstream>

namespace openvslam {
namespace util {

template <class TSomething>
inline std::string to_string(TSomething const& t ){
    std::stringstream sstream;
    sstream << t;
    return sstream.str();
}

}
}

#endif