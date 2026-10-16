#include "app.hpp"

#include <mutex>
#include <string>
#include <vector>

#include "channel_analysis.hpp"
#include "dsp_stitcher.hpp"
#include "esp_sdr_protocol.hpp"
#include "types.hpp"
#include "usb_host_sdr.hpp"

#if defined(ESP_PLATFORM)
#include <Arduino.h>
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#endif

namespace vtxmon {

namespace {

constexpr const char* TAG = "VtxMonApp";

// 5640 MHz 〜 5800 MHz (E2: 5665, E1: 5705, F3: 5740, F5: 5780 を含む帯域)
constexpr float BAND_START_MHZ = 5640.0f;
constexpr float BAND_STOP_MHZ = 5800.0f;
constexpr int CAP_SAMPLES = 8192;

UiSharedState g_ui_state;

// 受信経路の診断カウンタ
struct RxStats {
    uint32_t sweeps{0};
    uint32_t fail_freq{0};
    uint32_t fail_header{0};
    uint32_t fail_payload{0};
    uint32_t fail_crc{0};
    uint32_t resyncs{0};
};

// PC 版 esp_sdr.py の ask() 相当。
// ESP-SDR はコマンド 1 本ごとに必ず 1 行応答する。応答を読まずに次のコマンドを
// 送ると受信ストリームが 1 行ずつずれていき、CAP16 のペイロードが壊れる。
bool ask(UsbHostSdrClient& client, const std::string& cmd, uint32_t timeout_ms = 500) {
    if (!client.send_command(cmd)) {
        return false;
    }
    std::string reply;
    if (!client.read_line(reply, timeout_ms)) {
        return false;
    }
    return reply.rfind("ERR", 0) != 0;
}

void sleep_ms(uint32_t ms) {
#if defined(ESP_PLATFORM)
    vTaskDelay(pdMS_TO_TICKS(ms));
#else
    (void)ms;
#endif
}

// パターン1: 公式 ESP-SDR ファームウェア (生 I/Q を受けて TAB 5 側で FFT / スティッチング)
void run_raw_iq_mode(UsbHostSdrClient& client, DspStitcher& stitcher, UiSharedState& state) {
    // BANDWIDTH は必ず送る。送るまでファームウェアは PHY の Wi-Fi 20 MHz 校正フィルタを
    // 使い続け、各ホップの ±10 MHz 程度しか残らない (PC 版 esp_sdr.py のコメントより)。
    // 本実装は step 15 MHz / usable ±16 MHz で min 結合するため、フィルタが狭いまま
    // だとホップ外周で減衰した値が採用され、信号ピークが消えてノイズだけになる。
    if (!ask(client, EspSdrProtocol::cmd_bandwidth(0))) {
        ESP_LOGW(TAG, "BANDWIDTH 0 was not acknowledged - analog filter may stay narrow");
    }

    int initial_gain = 0;
    {
        std::lock_guard<std::mutex> lock(state.mtx);
        initial_gain = state.current_gain;
    }
    if (!ask(client, EspSdrProtocol::cmd_gain(initial_gain))) {
        ESP_LOGW(TAG, "GAIN %d was not acknowledged", initial_gain);
    }

    std::vector<std::vector<float>> hop_spectra(stitcher.los().size());
    std::vector<ComplexSample> iq_samples;
    std::vector<uint8_t> raw_buf;

    RxStats stats;
    uint32_t seq = 0;
    uint32_t last_log_ms = millis();
    uint32_t last_fps_calc_ms = millis();
    uint32_t sweeps_since_fps = 0;
    float current_fps = 0.0f;

    while (client.is_connected()) {
        // ゲイン変更要求。USB 送信はブロッキングなのでロックの外で行う。
        bool gain_changed = false;
        int gain_value = 0;
        {
            std::lock_guard<std::mutex> lock(state.mtx);
            if (state.request_gain_change) {
                state.current_gain = state.requested_gain;
                state.request_gain_change = false;
                gain_changed = true;
                gain_value = state.current_gain;
            }
        }
        if (gain_changed && !ask(client, EspSdrProtocol::cmd_gain(gain_value))) {
            ESP_LOGW(TAG, "GAIN %d was not acknowledged", gain_value);
        }

        bool all_hops_valid = true;
        for (size_t h = 0; h < stitcher.los().size(); ++h) {
            const int lo = stitcher.los()[h];
            bool this_hop_ok = false;
            constexpr size_t n_bytes = static_cast<size_t>(CAP_SAMPLES) * 2;

            // PC 版 esp_sdr.py に準拠: ホップ単位で最大3回リトライ
            for (int attempt = 0; attempt < 3; ++attempt) {
                if (attempt > 0) {
                    client.synchronize(60);
                }

                if (!ask(client, EspSdrProtocol::cmd_freq(lo), 150)) {
                    ++stats.fail_freq;
                    continue;
                }

                if (!client.send_command(EspSdrProtocol::cmd_cap16(CAP_SAMPLES, 0))) {
                    ++stats.fail_header;
                    continue;
                }

                std::string data_hdr;
                if (!client.read_line(data_hdr, 250)) {
                    ++stats.fail_header;
                    continue;
                }

                EspSdrProtocol::DataHeader dh;
                if (!EspSdrProtocol::parse_data_header(data_hdr, dh)) {
                    ++stats.fail_header;
                    continue;
                }

                raw_buf.resize(n_bytes);
                if (!client.read_bytes(raw_buf.data(), n_bytes, 350)) {
                    ++stats.fail_payload;
                    continue;
                }

                if (EspSdrProtocol::calculate_crc32(raw_buf.data(), n_bytes) != dh.crc32) {
                    ++stats.fail_crc;
                    continue;
                }

                if (!EspSdrProtocol::unpack_iq(raw_buf.data(), n_bytes, CAP_SAMPLES, 8, iq_samples)) {
                    ++stats.fail_payload;
                    continue;
                }

                stitcher.compute_hop_spectrum_db(iq_samples, hop_spectra[h]);
                this_hop_ok = true;
                break;
            }

            if (!this_hop_ok) {
                ++stats.resyncs;
                if (hop_spectra[h].empty()) {
                    // 初回などで過去データがない場合は全体やり直し
                    all_hops_valid = false;
                    ESP_LOGW(TAG, "hop %u (LO %d) initial fail - resyncing", (unsigned)h, lo);
                    client.synchronize(80);
                    break;
                }
                // 過去データがある場合は前回のホップスペクトラムを維持してスティッチングを継続
                ESP_LOGW(TAG, "hop %u (LO %d) dropped after 3 attempts, keeping previous hop data",
                         (unsigned)h, lo);
            }
        }

        if (!all_hops_valid) {
            continue;
        }

        SweepFrame frame;
        frame.seq = ++seq;
        frame.start_mhz = stitcher.start_mhz();
        frame.bin_hz = stitcher.bin_hz();
        stitcher.stitch(hop_spectra, frame.powers_db);

        // 診断: 実際に受信できている内容 (ピークの周波数とレベル) を定期出力する
        float peak_db = -300.0f;
        float peak_f = 0.0f;
        size_t valid_bins = 0;
        for (size_t i = 0; i < frame.powers_db.size(); ++i) {
            const float p = frame.powers_db[i];
            if (std::isnan(p)) {
                continue;
            }
            ++valid_bins;
            if (p > peak_db) {
                peak_db = p;
                peak_f = frame.freq_mhz(i);
            }
        }

        auto measurements = ChannelAnalysis::measure_all(
            frame.powers_db, frame.start_mhz, frame.bin_hz, state.active_channels);

        ++stats.sweeps;
        ++sweeps_since_fps;
        const uint32_t now = millis();
        if (now - last_fps_calc_ms >= 1000) {
            current_fps = static_cast<float>(sweeps_since_fps) * 1000.0f / (now - last_fps_calc_ms);
            sweeps_since_fps = 0;
            last_fps_calc_ms = now;
        }

        {
            std::lock_guard<std::mutex> lock(state.mtx);
            state.latest_frame = std::move(frame);
            state.measurements = std::move(measurements);
            state.sweep_fps = current_fps;
        }

        if (now - last_log_ms >= 2000) {
            ESP_LOGI(TAG, "sweeps=%u (%.1f fps) bins=%u/%u peak=%.1f dBFS @ %.3f MHz (crc=%u resync=%u overrun=%u)",
                     (unsigned)stats.sweeps, (double)current_fps, (unsigned)valid_bins,
                     (unsigned)stitcher.output_bins(), (double)peak_db, (double)peak_f,
                     (unsigned)stats.fail_crc, (unsigned)stats.resyncs,
                     (unsigned)client.overrun_count());
            last_log_ms = now;
        }
    }

}

// パターン2: 液晶版 FPV-LCD ファームウェア (FFT 済み SWEEP ストリームを受信)
void run_fft_stream_mode(UsbHostSdrClient& client, UiSharedState& state) {
    int initial_gain = 0;
    {
        std::lock_guard<std::mutex> lock(state.mtx);
        initial_gain = state.current_gain;
    }

    // いずれも 1 行応答があるので read しないと SWEEP ストリームとずれる
    ask(client, EspSdrProtocol::cmd_fpv_span(BAND_START_MHZ, BAND_STOP_MHZ,
                                             state.active_channels));
    ask(client, EspSdrProtocol::cmd_fpv_gain(initial_gain));
    ask(client, EspSdrProtocol::cmd_fpv_nolcd(true)); // No LCD: 高速掃引モード
    ask(client, EspSdrProtocol::cmd_fpv_stream(true));

    std::vector<uint8_t> payload_buf;
    while (client.is_connected()) {
        bool gain_changed = false;
        int gain_value = 0;
        {
            std::lock_guard<std::mutex> lock(state.mtx);
            if (state.request_gain_change) {
                state.current_gain = state.requested_gain;
                state.request_gain_change = false;
                gain_changed = true;
                gain_value = state.current_gain;
            }
        }
        if (gain_changed && !ask(client, EspSdrProtocol::cmd_fpv_gain(gain_value))) {
            ESP_LOGW(TAG, "FPV GAIN %d was not acknowledged", gain_value);
        }

        std::string sweep_line;
        if (!client.read_line(sweep_line, 1000)) {
            continue;
        }

        EspSdrProtocol::SweepHeader sh;
        if (!EspSdrProtocol::parse_sweep_header(sweep_line, sh)) {
            continue;
        }

        const size_t byte_len = sh.nbins * 2;
        payload_buf.resize(byte_len);
        if (!client.read_bytes(payload_buf.data(), byte_len, 500)) {
            continue;
        }
        if (EspSdrProtocol::calculate_crc32(payload_buf.data(), byte_len) != sh.crc32) {
            continue;
        }

        SweepFrame frame;
        frame.seq = sh.seq;
        frame.start_mhz = sh.start_mhz;
        frame.bin_hz = sh.bin_hz;
        frame.timestamp_ms = sh.t_ms;

        if (!EspSdrProtocol::decode_sweep_payload(payload_buf.data(), byte_len, sh.nbins,
                                                 frame.powers_db)) {
            continue;
        }

        auto measurements = ChannelAnalysis::measure_all(
            frame.powers_db, frame.start_mhz, frame.bin_hz, state.active_channels);

        std::lock_guard<std::mutex> lock(state.mtx);
        state.latest_frame = std::move(frame);
        state.measurements = std::move(measurements);
    }
}

} // namespace

UiSharedState& app_state() { return g_ui_state; }

void app_sdr_worker(void* /*arg*/) {
    ESP_LOGI(TAG, "SDR worker started (band %.0f-%.0f MHz)", BAND_START_MHZ, BAND_STOP_MHZ);

    UsbHostSdrClient client;
    if (!client.init()) {
        ESP_LOGE(TAG, "USB Host init failed!");
        std::lock_guard<std::mutex> lock(g_ui_state.mtx);
        g_ui_state.device_info = "USB Host init failed";
#if defined(ESP_PLATFORM)
        vTaskDelete(nullptr);
#endif
        return;
    }

    DspSettings dsp_settings;
    dsp_settings.nfft = 1024;
    dsp_settings.step_mhz = 15;
    DspStitcher stitcher(BAND_START_MHZ, BAND_STOP_MHZ, dsp_settings);

    ESP_LOGI(TAG, "hop plan: %u LOs, %d..%d MHz, bin=%.3f kHz, %u output bins",
             (unsigned)stitcher.los().size(), stitcher.los().front(), stitcher.los().back(),
             (double)(stitcher.bin_hz() / 1000.0f), (unsigned)stitcher.output_bins());

    while (true) {
        {
            std::lock_guard<std::mutex> lock(g_ui_state.mtx);
            g_ui_state.connected = false;
            g_ui_state.device_info = "Waiting for StampC5...";
        }

        if (!client.wait_for_device(5000)) {
            sleep_ms(500);
            continue;
        }

        client.synchronize(500);

        // プロトコル自動判別: FPV? に応答があれば液晶版ファームウェア
        client.send_command(EspSdrProtocol::cmd_fpv_probe());
        std::string reply;
        const bool is_fpv_fw = client.read_line(reply, 500) && reply.rfind("FPV ", 0) == 0;
        ESP_LOGI(TAG, "FPV? probe -> '%s' (%s firmware)", reply.c_str(),
                 is_fpv_fw ? "FPV-LCD" : "official ESP-SDR");

        // 素性確認。PC 版は INFO が "C5SDR" で始まることを要求している
        // (5.8 GHz 帯は ESP32-C5 でないとチューニングできない)。
        std::string info_line;
        if (client.send_command(EspSdrProtocol::cmd_info()) &&
            client.read_line(info_line, 500)) {
            ESP_LOGI(TAG, "device INFO: %s", info_line.c_str());
        } else {
            ESP_LOGW(TAG, "no INFO reply - is the ESP-SDR firmware flashed?");
        }

        {
            std::lock_guard<std::mutex> lock(g_ui_state.mtx);
            g_ui_state.connected = true;
            g_ui_state.rx_mode = is_fpv_fw ? "fft (Stream)" : "iq (Raw I/Q)";
            g_ui_state.device_info = client.device_info();
        }
        ESP_LOGI(TAG, "connected: %s mode=%s", client.device_info().c_str(),
                 is_fpv_fw ? "fft-stream" : "raw-iq");

        if (is_fpv_fw) {
            run_fft_stream_mode(client, g_ui_state);
        } else {
            run_raw_iq_mode(client, stitcher, g_ui_state);
        }

        client.close();
    }
}

} // namespace vtxmon
