#include "app.hpp"

#include <mutex>
#include <string>
#include <vector>

#include "channel_analysis.hpp"
#include "dsp_stitcher.hpp"
#include "esp_sdr_protocol.hpp"
#include "widscan_protocol.hpp"
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

// パターン3: Widscan58 ファームウェア (ESP32-C5 側で高速ホッピング & FFT & メトリクス計算)
void run_widscan_mode(UsbHostSdrClient& client, UiSharedState& state) {
    ESP_LOGI(TAG, "entering Widscan58 mode");

    // 5640〜5800 MHz の全帯域を隙間なくカバーするチャンネル計画を生成
    auto scan_plan = WidscanProtocol::build_full_band_plan(
        state.active_channels, BAND_START_MHZ, BAND_STOP_MHZ);

    std::string ch_cmd = WidscanProtocol::cmd_channels(scan_plan);
    if (!ask(client, ch_cmd, 300)) {
        ESP_LOGW(TAG, "CHANNELS command failed: %s", ch_cmd.c_str());
    } else {
        ESP_LOGI(TAG, "sent wide CHANNELS: %s", ch_cmd.c_str());
    }

    // scan_plan の各インデックス (0..scan_plan.size()-1) から
    // state.active_channels (0..3) への逆引きインデックスを構築
    std::vector<int> plan_to_active(scan_plan.size(), -1);
    for (size_t i = 0; i < scan_plan.size(); ++i) {
        for (size_t j = 0; j < state.active_channels.size(); ++j) {
            if (std::abs(scan_plan[i].freq_mhz - state.active_channels[j].freq_mhz) < 2.0f) {
                plan_to_active[i] = static_cast<int>(j);
                break;
            }
        }
    }

    int initial_gain = 0;
    {
        std::lock_guard<std::mutex> lock(state.mtx);
        initial_gain = state.current_gain;
    }
    if (initial_gain < 0) {
        ask(client, WidscanProtocol::cmd_gain_hardware(), 300);
    } else {
        ask(client, WidscanProtocol::cmd_gain_manual(initial_gain), 300);
    }

    // スペクトラム設定: 毎ホップ (every=1), 256 bins, span 40MHz (40000000 = C5 usable_hz 全幅), MAX detector
    ask(client, WidscanProtocol::cmd_spec(1, 256, 40000000, true), 300);

    // スキャン開始
    if (!ask(client, WidscanProtocol::cmd_scan_start(), 500)) {
        ESP_LOGE(TAG, "SCAN START failed");
        return;
    }

    WidscanFrameParser parser;
    WidscanSpectrumComposite composite(BAND_START_MHZ, BAND_STOP_MHZ, 100000.0f /* 100 kHz bin */);

    std::vector<ChannelMeasurement> measurements(state.active_channels.size());
    for (size_t i = 0; i < state.active_channels.size(); ++i) {
        measurements[i].name = state.active_channels[i].name;
        measurements[i].nominal_mhz = state.active_channels[i].freq_mhz;
        measurements[i].present = false;
        measurements[i].reason = "WAITING";
    }

    uint32_t seq = 0;
    uint32_t last_ui_update_ms = millis();
    uint32_t last_log_ms = millis();
    float current_fps = 0.0f;
    uint32_t frames_received = 0;
    uint32_t last_fps_calc_ms = millis();
    uint32_t frames_since_fps = 0;

    std::vector<uint8_t> rx_buf(2048);

    while (client.is_connected()) {
        // ゲイン変更のチェック
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
        if (gain_changed) {
            if (gain_value < 0) {
                ask(client, WidscanProtocol::cmd_gain_hardware(), 200);
            } else {
                ask(client, WidscanProtocol::cmd_gain_manual(gain_value), 200);
            }
        }

        // ノンブロッキングで利用可能なバイトを読み出し
        size_t n_read = client.read_available(rx_buf.data(), rx_buf.size());
        if (n_read == 0) {
            sleep_ms(2);
            continue;
        }

        parser.feed(rx_buf.data(), n_read, [&](const WidscanParsedFrame& frame) {
            if (frame.type == FRAME_METRICS) {
                ++frames_received;
                ++frames_since_fps;
                const auto& mf = frame.metrics;
                for (uint8_t c = 0; c < mf.header.nch; ++c) {
                    const auto& ch = mf.channels[c];
                    if (ch.id < plan_to_active.size()) {
                        const int active_idx = plan_to_active[ch.id];
                        if (active_idx >= 0 && static_cast<size_t>(active_idx) < measurements.size()) {
                            auto& m = measurements[active_idx];
                            m.name = state.active_channels[active_idx].name;
                            m.nominal_mhz = state.active_channels[active_idx].freq_mhz;
                            m.present = (ch.flags & WIDSCAN_CHF_SIGNAL) != 0;
                            m.offset_mhz = static_cast<float>(ch.centroid_off_hz) / 1e6f;
                            m.centre_mhz = m.nominal_mhz + m.offset_mhz;
                            m.peak_mhz = m.nominal_mhz + (static_cast<float>(ch.peak_off_hz) / 1e6f);
                            m.peak_dbm = static_cast<float>(ch.peak_cdb) / 100.0f;
                            m.floor_dbm = static_cast<float>(ch.floor_cdb) / 100.0f;
                            m.snr_db = static_cast<float>(ch.snr_cdb) / 100.0f;
                            m.bw_db3_mhz = static_cast<float>(ch.obw_hi_khz - ch.obw_lo_khz) / 1000.0f;
                            m.bw_db20_mhz = m.bw_db3_mhz;
                            if (!m.present) {
                                m.reason = "NO SIG";
                            } else {
                                m.reason.clear();
                            }
                        }
                    }
                }
            } else if (frame.type == FRAME_SPECTRUM) {
                const auto& sf = frame.spectrum;
                composite.update_segment(sf.header.lo_khz, sf.header.start_off_hz,
                                         sf.header.bin_hz, sf.codes.data(), sf.codes.size());
            } else if (frame.type == FRAME_TELEMETRY) {
                const auto& tf = frame.telemetry;
                if (tf.rotation_mhz > 0) {
                    current_fps = static_cast<float>(tf.rotation_mhz) / 1000.0f;
                }
            }
        });

        const uint32_t now = millis();
        if (now - last_fps_calc_ms >= 1000) {
            if (current_fps <= 0.0f) {
                current_fps = static_cast<float>(frames_since_fps) * 1000.0f / (now - last_fps_calc_ms);
            }
            frames_since_fps = 0;
            last_fps_calc_ms = now;
        }

        // 画面更新 (約 60fps 間隔: 16ms)
        if (now - last_ui_update_ms >= 16) {
            last_ui_update_ms = now;
            SweepFrame frame;
            frame.seq = ++seq;
            composite.compose(frame);

            std::lock_guard<std::mutex> lock(state.mtx);
            state.latest_frame = std::move(frame);
            state.measurements = measurements;
            state.sweep_fps = current_fps;
        }

        if (now - last_log_ms >= 2000) {
            ESP_LOGI(TAG, "widscan58: %.1f fps (frames=%u valid=%u crc_err=%u resyncs=%u overrun=%u)",
                     (double)current_fps, (unsigned)frames_received,
                     (unsigned)parser.valid_frames(), (unsigned)parser.crc_errors(),
                     (unsigned)parser.resyncs(), (unsigned)client.overrun_count());
            last_log_ms = now;
        }
    }

    ask(client, WidscanProtocol::cmd_scan_stop(), 200);
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

        // プロトコル自動判別
        // 1. Widscan58 ファームウェアの判定:
        //    起動時は RotorHazard モードで無言のため、PROTOCOL MONITOR を送って OK が返るか確認
        bool is_widscan = false;
        std::string info_line;
        if (ask(client, WidscanProtocol::cmd_protocol_monitor(), 300)) {
            client.send_command(WidscanProtocol::cmd_info());
            if (client.read_line(info_line, 300) && info_line.find("widscan58") != std::string::npos) {
                is_widscan = true;
                ESP_LOGI(TAG, "detected Widscan58 firmware: %s", info_line.c_str());
            }
        }

        bool is_fpv_fw = false;
        if (!is_widscan) {
            // 2. FPV-LCD ファームウェアの判定: FPV? に応答があるか
            client.send_command(EspSdrProtocol::cmd_fpv_probe());
            std::string reply;
            is_fpv_fw = client.read_line(reply, 300) && reply.rfind("FPV ", 0) == 0;
            ESP_LOGI(TAG, "FPV? probe -> '%s' (%s firmware)", reply.c_str(),
                     is_fpv_fw ? "FPV-LCD" : "official ESP-SDR");

            // 3. 公式 ESP-SDR: 素性確認 (INFO が "C5SDR" 等)
            if (client.send_command(EspSdrProtocol::cmd_info()) &&
                client.read_line(info_line, 300)) {
                ESP_LOGI(TAG, "device INFO: %s", info_line.c_str());
            } else {
                ESP_LOGW(TAG, "no INFO reply - is the ESP-SDR firmware flashed?");
            }
        }

        {
            std::lock_guard<std::mutex> lock(g_ui_state.mtx);
            g_ui_state.connected = true;
            if (is_widscan) {
                g_ui_state.rx_mode = "widscan (Widscan58)";
            } else if (is_fpv_fw) {
                g_ui_state.rx_mode = "fft (Stream)";
            } else {
                g_ui_state.rx_mode = "iq (Raw I/Q)";
            }
            g_ui_state.device_info = info_line.empty() ? client.device_info() : info_line;
        }
        ESP_LOGI(TAG, "connected: %s mode=%s", g_ui_state.device_info.c_str(),
                 g_ui_state.rx_mode.c_str());

        if (is_widscan) {
            run_widscan_mode(client, g_ui_state);
        } else if (is_fpv_fw) {
            run_fft_stream_mode(client, g_ui_state);
        } else {
            run_raw_iq_mode(client, stitcher, g_ui_state);
        }

        client.close();
    }
}

} // namespace vtxmon
