#pragma once

#include <vector>
#include <mutex>
#include "types.hpp"

namespace vtxmon {

struct UiSharedState {
    std::mutex mtx;
    bool connected{false};
    std::string device_info{"Waiting for StampC5..."};
    std::string rx_mode{"auto -> iq"};
    int current_gain{25};
    float sweep_fps{0.0f};

    SweepFrame latest_frame;
    std::vector<ChannelMeasurement> measurements;
    std::vector<ChannelDef> active_channels{
        {"E2", 5665.0f},
        {"E1", 5705.0f},
        {"F3", 5740.0f},
        {"F5", 5780.0f}
    };

    // UI から SDR タスクへの要求フラグ
    bool request_gain_change{false};
    int requested_gain{25};
};

} // namespace vtxmon
