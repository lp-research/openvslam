#ifndef OPENVSLAM_OPTIMIZE_POSE_OPTIMIZER_H
#define OPENVSLAM_OPTIMIZE_POSE_OPTIMIZER_H

namespace openvslam {

namespace data {
class frame;
}

namespace optimize {

class pose_optimizer {
public:
    /**
     * Constructor
     * @param num_trials
     * @param num_each_iter
     */
    explicit pose_optimizer(const unsigned int num_trials = 4, const unsigned int num_each_iter = 10);

    /**
     * Destructor
     */
    virtual ~pose_optimizer() = default;

    /**
     * Perform pose optimization
     * @param frm
     * @param last_frm: If last_frm is given, the navigation data will be used to fit the new pose
     * @return
     */
    unsigned int optimize(data::frame& frm, data::frame const& last_frm = data::frame()) const;

private:
    //! robust optimizationの試行回数
    const unsigned int num_trials_ = 4;

    //! 毎回のoptimizationのiteration回数
    const unsigned int num_each_iter_ = 10;
};

} // namespace optimize
} // namespace openvslam

#endif // OPENVSLAM_OPTIMIZE_POSE_OPTIMIZER_H
