#pragma once

#include "ui_state.hpp"

namespace vtxmon {

// UI タスクと SDR ワーカータスクが共有する状態
UiSharedState& app_state();

// SDR 受信・解析ワーカー (FreeRTOS タスクエントリ)
void app_sdr_worker(void* arg);

} // namespace vtxmon
