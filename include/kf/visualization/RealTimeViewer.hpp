#pragma once

#include <vector>
#include <functional>
#include "kf/core/EstimatorSnapshot.hpp"

struct GLFWwindow;

namespace kf 
{

class RealTimeViewer
{

public:

    using StepCallback = std::function<bool(EstimatorSnapshot&)>;
    RealTimeViewer();
    ~RealTimeViewer();

    bool initialize();

    void run(const std::vector<EstimatorSnapshot>& history);

    void runLive(std::vector<EstimatorSnapshot> &history, const StepCallback& callback, Scalar sim_dt,
    Scalar qx,
    Scalar qy,
    Scalar rx,
    Scalar ry );

    void shutdown();

private:
    GLFWwindow* window_;
    bool initialized_;

};

}