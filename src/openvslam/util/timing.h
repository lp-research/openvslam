#ifndef OPENVSLAM_UTIL_TIMING_H
#define OPENVSLAM_UTIL_TIMING_H

#include <chrono>

namespace openvslam {
namespace util {

/**
 * Base class for measurng timing differences
 */
class TimingBase {
public:
    /**
	 * Create and start the time taking
	 */
    TimingBase() {
        start();
    }

    /**
	 * Take the start time
	 */
    void start() {
        m_startTime = std::chrono::high_resolution_clock::now();
    }

    /**
	 * return the time difference in seconds between the calls to start() and
	 * end()
	 */
    float delta() const {
        std::chrono::milliseconds secs = std::chrono::duration_cast<
            std::chrono::milliseconds>(m_endTime - m_startTime);
        return secs.count() * 0.001f;
    }

    /**
	 * Take the stop time
	 */
    float end() {
        m_endTime = std::chrono::high_resolution_clock::now();
        return delta();
    }

private:
    /**
	 * Time point when start() was called
	 */
    std::chrono::high_resolution_clock::time_point m_startTime;

    /**
	 * Time point when end() was called
	 */
    std::chrono::high_resolution_clock::time_point m_endTime;
};

} // namespace util
} // namespace openvslam

#endif