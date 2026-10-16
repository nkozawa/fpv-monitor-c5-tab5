// M5Stack TAB 5 + StampC5 による FPV 5.8GHz バンドモニター
// PlatformIO (ESP32-P4 + M5GFX + EspUsbHost)

#include <Arduino.h>
#include <mutex>

#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "app.hpp"
#include "ui_renderer.hpp"

namespace {

vtxmon::UiRenderer g_renderer;
TaskHandle_t g_sdr_task = nullptr;
uint32_t g_last_render_ms = 0;

// SDR ワーカーは USB Host タスクと同じ core 0、UI 描画は Arduino loopTask の core 1
constexpr BaseType_t SDR_WORKER_CORE = 0;
constexpr uint32_t SDR_WORKER_STACK = 16384;
constexpr UBaseType_t SDR_WORKER_PRIORITY = 5;

constexpr uint32_t FRAME_INTERVAL_MS = 33; // 描画は約 30 fps
constexpr uint32_t TOUCH_POLL_MS = 10;     // タッチはより高頻度に見て取りこぼしを防ぐ

} // namespace

void setup() {
    ESP_LOGI("VTXMon", "Starting VTX Band Monitor for M5Stack TAB 5...");

    if (!g_renderer.init()) {
        ESP_LOGE("VTXMon", "Display init failed");
    }

    if (xTaskCreatePinnedToCore(vtxmon::app_sdr_worker,
                                "sdr_worker",
                                SDR_WORKER_STACK,
                                nullptr,
                                SDR_WORKER_PRIORITY,
                                &g_sdr_task,
                                SDR_WORKER_CORE) != pdPASS) {
        ESP_LOGE("VTXMon", "Failed to create SDR worker task");
        g_sdr_task = nullptr;
    }
}

void loop() {
    vtxmon::UiSharedState& state = vtxmon::app_state();

    // タップ (押下エッジ) を処理。ゲイン変更は state 経由で SDR タスクに渡るためロックが必要。
    int x = 0;
    int y = 0;
    if (g_renderer.poll_tap(x, y)) {
        std::lock_guard<std::mutex> lock(state.mtx);
        g_renderer.handle_touch(x, y, state);
    }

    const uint32_t now = millis();
    if (now - g_last_render_ms >= FRAME_INTERVAL_MS) {
        g_last_render_ms = now;
        std::lock_guard<std::mutex> lock(state.mtx);
        g_renderer.render(state);
    }

    delay(TOUCH_POLL_MS);
}
