#pragma once

#include <cstdint>
#include <string>
#include <vector>
#include <map>
#include <functional>
#include "types.hpp"

namespace vtxmon {

// widscan58 wire protocol v1 定数
constexpr uint8_t WIDSCAN_SYNC0 = 0xA5;
constexpr uint8_t WIDSCAN_SYNC1 = 'R';
constexpr uint8_t WIDSCAN_PROTO_VERSION = 1;

enum WidscanFrameType : uint8_t {
    FRAME_METRICS = 1,
    FRAME_SPECTRUM = 2,
    FRAME_TELEMETRY = 3,
    FRAME_RAWIQ = 4,
    FRAME_LOG = 5,
    FRAME_LEVELS = 6,
};

// チャンネルフラグ
constexpr uint8_t WIDSCAN_CHF_SIGNAL = 1;
constexpr uint8_t WIDSCAN_CHF_SATURATED = 2;
constexpr uint8_t WIDSCAN_CHF_MASKED = 4;
constexpr uint8_t WIDSCAN_CHF_EDGE = 8;
constexpr uint8_t WIDSCAN_CHF_NO_CENTROID = 16;
constexpr uint8_t WIDSCAN_CHF_SPUR = 32;

#pragma pack(push, 1)

struct WidscanHeader {
    uint8_t sync0;
    uint8_t sync1;
    uint8_t type;
    uint8_t version;
    uint16_t len;
};

// FRAME_METRICS (type = 1)
struct WidscanFrameMetricsHeader {
    uint32_t seq;
    uint32_t t_ms;
    uint32_t lo_khz;
    uint32_t tune_us;
    uint32_t capture_us;
    uint32_t dsp_us;
    uint16_t saturated;
    int16_t floor_bin_cdb;   // mean noise per FFT bin (dBFS x 100)
    int16_t dc_cdb;          // DC (LO leakage) power vs full scale
    uint8_t gain;            // 255 = hardware AGC
    uint8_t hop_index;
    uint8_t hops;
    uint8_t nch;
};

struct WidscanFrameChannel {
    uint8_t id;              // channel index (0..nch-1)
    uint8_t flags;           // WIDSCAN_CHF_*
    int16_t rssi_cdb;        // RSSI (dBFS x 100)
    int16_t rssi_ema_cdb;
    int16_t floor_cdb;       // noise in RSSI window
    int16_t snr_cdb;         // (S+N)/N (dB x 100)
    int16_t clear_floor_cdb;
    int16_t peak_cdb;        // strongest bin this hop
    int32_t peak_off_hz;     // peak offset from nominal in Hz
    int32_t centroid_off_hz; // centroid offset from nominal in Hz
    int16_t obw_lo_khz;      // 99% occupied bandwidth relative to nominal
    int16_t obw_hi_khz;
};

// FRAME_SPECTRUM (type = 2)
struct WidscanFrameSpectrumHeader {
    uint32_t seq;
    uint32_t lo_khz;
    int32_t start_off_hz;    // center of first bin relative to LO
    uint32_t bin_hz;
    uint16_t n;
    uint8_t detector;        // 0 = mean, 1 = max
    uint8_t reserved;
};

// FRAME_TELEMETRY (type = 3)
struct WidscanFrameTelemetry {
    uint32_t t_ms;
    uint32_t rotations;
    uint32_t hops_total;
    uint32_t rotation_mhz;   // rotations/sec x 1000
    uint32_t hop_us_avg;
    uint32_t hop_us_max;
    uint32_t tune_us_avg;
    uint32_t tune_us_max;
    uint32_t capture_us_avg;
    uint32_t dsp_us_avg;
    uint32_t dsp_us_max;
    uint16_t link_fill_pm;
    uint32_t heap_free;
    uint32_t drops;
    uint32_t capture_errors;
    uint8_t running;
};

#pragma pack(pop)

// パース済みフレームの保持構造
struct WidscanMetricsData {
    WidscanFrameMetricsHeader header;
    std::vector<WidscanFrameChannel> channels;
};

struct WidscanSpectrumData {
    WidscanFrameSpectrumHeader header;
    std::vector<uint8_t> codes; // dBFS = code / 2.0 - 127.5
};

struct WidscanParsedFrame {
    uint8_t type{0};
    WidscanMetricsData metrics;
    WidscanSpectrumData spectrum;
    WidscanFrameTelemetry telemetry;
};

class WidscanProtocol {
public:
    // --- コマンド生成 ---
    static std::string cmd_protocol_monitor();
    static std::string cmd_info();
    static std::string cmd_profile_query();
    static std::string cmd_channels(const std::vector<ChannelDef>& channels);
    static std::string cmd_gain_manual(int gain);
    static std::string cmd_gain_hardware();
    static std::string cmd_spec(int every_n_hops, int bins, uint32_t span_hz, bool max_detector = true);
    static std::string cmd_scan_start();
    static std::string cmd_scan_stop();

    // 5640〜5800 MHz の全帯域を隙間なくカバーできるように、
    // active_channels を含めた最大8チャンネルのスキャン計画 (CHANNELS 用) を構築する
    static std::vector<ChannelDef> build_full_band_plan(
        const std::vector<ChannelDef>& active_channels,
        float band_start_mhz = 5640.0f,
        float band_stop_mhz = 5800.0f);

    // --- CRC32 (IEEE 802.3 / zlib互換) ---
    static uint32_t calculate_crc32(const uint8_t* data, size_t length);
};

// バイナリフレームを逐次パースするストリームパーサー
class WidscanFrameParser {
public:
    using FrameCallback = std::function<void(const WidscanParsedFrame&)>;
    using TextCallback = std::function<void(const std::string&)>;

    WidscanFrameParser();

    // 受信データを投入し、フレームが完成するたびにコールバックを呼ぶ
    void feed(const uint8_t* data, size_t length,
              const FrameCallback& on_frame,
              const TextCallback& on_text = nullptr);

    void reset();

    uint32_t valid_frames() const { return valid_frames_; }
    uint32_t crc_errors() const { return crc_errors_; }
    uint32_t resyncs() const { return resyncs_; }

private:
    enum State {
        STATE_SEARCH_SYNC,
        STATE_READ_HEADER,
        STATE_READ_PAYLOAD_AND_CRC
    };

    State state_{STATE_SEARCH_SYNC};
    std::vector<uint8_t> buffer_;
    uint8_t frame_type_{0};
    uint8_t frame_ver_{0};
    uint16_t payload_len_{0};
    std::string text_line_;

    uint32_t valid_frames_{0};
    uint32_t crc_errors_{0};
    uint32_t resyncs_{0};

    bool parse_payload(uint8_t type, const uint8_t* payload, size_t len, WidscanParsedFrame& out);
};

// 複数 LO のスペクトラムセグメントを合成 (Composite) するクラス
class WidscanSpectrumComposite {
public:
    WidscanSpectrumComposite(float band_start_mhz, float band_stop_mhz, float bin_hz);

    // 受信したセグメントを登録/更新
    void update_segment(uint32_t lo_khz, int32_t start_off_hz, uint32_t bin_hz,
                        const uint8_t* codes, size_t n_codes);

    // 全セグメントから合成して SweepFrame を生成
    void compose(SweepFrame& out_frame) const;

    void clear();

private:
    struct Segment {
        uint32_t lo_khz{0};
        float start_mhz{0.0f};
        float bin_hz{0.0f};
        std::vector<float> powers_db;
    };

    float band_start_mhz_;
    float band_stop_mhz_;
    float bin_hz_;
    size_t total_bins_;
    std::map<uint32_t, Segment> segments_by_lo_; // LO (kHz) -> Segment
};

} // namespace vtxmon
