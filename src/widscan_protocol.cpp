#include "widscan_protocol.hpp"

#include <algorithm>
#include <cmath>
#include <cstring>
#include <array>
#include <sstream>

namespace vtxmon {

namespace {

// CRC32 Lookup Table (IEEE 802.3)
constexpr std::array<uint32_t, 256> generate_crc32_table() {
    std::array<uint32_t, 256> table{};
    for (uint32_t i = 0; i < 256; ++i) {
        uint32_t crc = i;
        for (int j = 0; j < 8; ++j) {
            if (crc & 1) {
                crc = (crc >> 1) ^ 0xEDB88320U;
            } else {
                crc >>= 1;
            }
        }
        table[i] = crc;
    }
    return table;
}

constexpr auto CRC32_TABLE = generate_crc32_table();

constexpr size_t MAX_PAYLOAD_BYTES = 4096;

} // namespace

std::string WidscanProtocol::cmd_protocol_monitor() {
    return "PROTOCOL MONITOR\n";
}

std::string WidscanProtocol::cmd_info() {
    return "INFO\n";
}

std::string WidscanProtocol::cmd_profile_query() {
    return "PROFILE?\n";
}

std::string WidscanProtocol::cmd_channels(const std::vector<ChannelDef>& channels) {
    std::string s = "CHANNELS";
    for (const auto& ch : channels) {
        s += " " + ch.name + "=" + std::to_string(static_cast<int>(std::round(ch.freq_mhz)));
    }
    s += "\n";
    return s;
}

std::string WidscanProtocol::cmd_gain_manual(int gain) {
    return "GAIN MANUAL " + std::to_string(gain) + "\n";
}

std::string WidscanProtocol::cmd_gain_hardware() {
    return "GAIN HARDWARE\n";
}

std::string WidscanProtocol::cmd_spec(int every_n_hops, int bins, uint32_t span_hz, bool max_detector) {
    std::ostringstream oss;
    oss << "SPEC " << every_n_hops << " " << bins << " " << span_hz
        << (max_detector ? " MAX\n" : " MEAN\n");
    return oss.str();
}

std::string WidscanProtocol::cmd_scan_start() {
    return "SCAN START\n";
}

std::string WidscanProtocol::cmd_scan_stop() {
    return "SCAN STOP\n";
}

std::vector<ChannelDef> WidscanProtocol::build_full_band_plan(
    const std::vector<ChannelDef>& active_channels,
    float band_start_mhz,
    float band_stop_mhz) {
    // 5.8GHz FPV 帯の候補チャンネル (5640〜5800 MHz 全域を均等にカバーする定番周波数)
    static const std::vector<ChannelDef> CANDIDATES = {
        {"E4", 5645.0f},
        {"E2", 5665.0f},
        {"E3", 5685.0f},
        {"E1", 5705.0f},
        {"A8", 5725.0f},
        {"F3", 5740.0f},
        {"F2", 5760.0f},
        {"F5", 5780.0f},
        {"F4", 5800.0f}
    };

    std::vector<ChannelDef> plan = active_channels;

    auto is_covered = [&](float freq) {
        for (const auto& ch : plan) {
            if (std::abs(ch.freq_mhz - freq) < 5.0f) {
                return true;
            }
        }
        return false;
    };

    // 最大 8 チャンネルまで候補から隙間を埋めるチャンネルを追加
    for (const auto& cand : CANDIDATES) {
        if (plan.size() >= 8) {
            break;
        }
        if (cand.freq_mhz >= band_start_mhz - 10.0f && cand.freq_mhz <= band_stop_mhz + 10.0f) {
            if (!is_covered(cand.freq_mhz)) {
                plan.push_back(cand);
            }
        }
    }

    // 周波数の昇順にソート (widscan58 のホップ計画グループ化が最適化される)
    std::sort(plan.begin(), plan.end(), [](const ChannelDef& a, const ChannelDef& b) {
        return a.freq_mhz < b.freq_mhz;
    });

    return plan;
}

uint32_t WidscanProtocol::calculate_crc32(const uint8_t* data, size_t length) {
    uint32_t crc = 0xFFFFFFFFU;
    for (size_t i = 0; i < length; ++i) {
        uint8_t index = static_cast<uint8_t>((crc ^ data[i]) & 0xFF);
        crc = (crc >> 8) ^ CRC32_TABLE[index];
    }
    return crc ^ 0xFFFFFFFFU;
}

// ---------------------------------------------------------------------------
// WidscanFrameParser
// ---------------------------------------------------------------------------

WidscanFrameParser::WidscanFrameParser() {
    buffer_.reserve(2048);
}

void WidscanFrameParser::reset() {
    state_ = STATE_SEARCH_SYNC;
    buffer_.clear();
    text_line_.clear();
}

void WidscanFrameParser::feed(const uint8_t* data, size_t length,
                              const FrameCallback& on_frame,
                              const TextCallback& on_text) {
    for (size_t i = 0; i < length; ++i) {
        const uint8_t byte = data[i];

        switch (state_) {
        case STATE_SEARCH_SYNC:
            if (byte == WIDSCAN_SYNC0) {
                buffer_.clear();
                buffer_.push_back(byte);
                state_ = STATE_READ_HEADER;
            } else {
                // テキスト行の蓄積
                if (byte == '\n') {
                    if (on_text && !text_line_.empty()) {
                        on_text(text_line_);
                    }
                    text_line_.clear();
                } else if (byte >= 0x20 && byte <= 0x7E) {
                    if (text_line_.size() < 256) {
                        text_line_.push_back(static_cast<char>(byte));
                    }
                }
            }
            break;

        case STATE_READ_HEADER:
            buffer_.push_back(byte);
            if (buffer_.size() == 2) {
                if (buffer_[1] != WIDSCAN_SYNC1) {
                    // 不正な同期ワード。再同期
                    state_ = STATE_SEARCH_SYNC;
                    buffer_.clear();
                    ++resyncs_;
                    continue;
                }
            } else if (buffer_.size() == sizeof(WidscanHeader)) { // 6 バイト
                frame_type_ = buffer_[2];
                frame_ver_ = buffer_[3];
                payload_len_ = static_cast<uint16_t>(buffer_[4] | (buffer_[5] << 8));

                if (frame_ver_ != WIDSCAN_PROTO_VERSION || payload_len_ > MAX_PAYLOAD_BYTES) {
                    // 無効なバージョンまたは長すぎるペイロード
                    state_ = STATE_SEARCH_SYNC;
                    buffer_.clear();
                    ++resyncs_;
                } else {
                    state_ = STATE_READ_PAYLOAD_AND_CRC;
                }
            }
            break;

        case STATE_READ_PAYLOAD_AND_CRC:
            buffer_.push_back(byte);
            // 必要なバイト数 = ヘッダ(6) + ペイロード(len) + CRC(4)
            if (buffer_.size() == sizeof(WidscanHeader) + payload_len_ + 4) {
                const size_t total_frame_data_len = sizeof(WidscanHeader) + payload_len_;
                const uint32_t expected_crc = WidscanProtocol::calculate_crc32(buffer_.data(), total_frame_data_len);

                const size_t crc_idx = total_frame_data_len;
                const uint32_t received_crc = static_cast<uint32_t>(buffer_[crc_idx]) |
                                              (static_cast<uint32_t>(buffer_[crc_idx + 1]) << 8) |
                                              (static_cast<uint32_t>(buffer_[crc_idx + 2]) << 16) |
                                              (static_cast<uint32_t>(buffer_[crc_idx + 3]) << 24);

                if (expected_crc == received_crc) {
                    WidscanParsedFrame frame;
                    const uint8_t* payload = buffer_.data() + sizeof(WidscanHeader);
                    if (parse_payload(frame_type_, payload, payload_len_, frame)) {
                        ++valid_frames_;
                        if (on_frame) {
                            on_frame(frame);
                        }
                    }
                } else {
                    ++crc_errors_;
                }

                state_ = STATE_SEARCH_SYNC;
                buffer_.clear();
            }
            break;
        }
    }
}

bool WidscanFrameParser::parse_payload(uint8_t type, const uint8_t* payload, size_t len,
                                      WidscanParsedFrame& out) {
    out.type = type;

    switch (type) {
    case FRAME_METRICS: {
        if (len < sizeof(WidscanFrameMetricsHeader)) {
            return false;
        }
        std::memcpy(&out.metrics.header, payload, sizeof(WidscanFrameMetricsHeader));
        const size_t ch_count = out.metrics.header.nch;
        const size_t expected_len = sizeof(WidscanFrameMetricsHeader) + ch_count * sizeof(WidscanFrameChannel);
        if (len < expected_len) {
            return false;
        }
        out.metrics.channels.resize(ch_count);
        const uint8_t* ch_ptr = payload + sizeof(WidscanFrameMetricsHeader);
        for (size_t i = 0; i < ch_count; ++i) {
            std::memcpy(&out.metrics.channels[i], ch_ptr + i * sizeof(WidscanFrameChannel),
                        sizeof(WidscanFrameChannel));
        }
        return true;
    }

    case FRAME_SPECTRUM: {
        if (len < sizeof(WidscanFrameSpectrumHeader)) {
            return false;
        }
        std::memcpy(&out.spectrum.header, payload, sizeof(WidscanFrameSpectrumHeader));
        const size_t n = out.spectrum.header.n;
        if (len < sizeof(WidscanFrameSpectrumHeader) + n) {
            return false;
        }
        out.spectrum.codes.resize(n);
        std::memcpy(out.spectrum.codes.data(),
                    payload + sizeof(WidscanFrameSpectrumHeader), n);
        return true;
    }

    case FRAME_TELEMETRY: {
        if (len < sizeof(WidscanFrameTelemetry)) {
            return false;
        }
        std::memcpy(&out.telemetry, payload, sizeof(WidscanFrameTelemetry));
        return true;
    }

    default:
        // その他のフレーム (RAWIQ, LOG, LEVELS) はここでは処理せずスルー
        return false;
    }
}

// ---------------------------------------------------------------------------
// WidscanSpectrumComposite
// ---------------------------------------------------------------------------

WidscanSpectrumComposite::WidscanSpectrumComposite(float band_start_mhz, float band_stop_mhz, float bin_hz)
    : band_start_mhz_(band_start_mhz),
      band_stop_mhz_(band_stop_mhz),
      bin_hz_(bin_hz) {
    const float span_mhz = band_stop_mhz_ - band_start_mhz_;
    total_bins_ = static_cast<size_t>(std::ceil(span_mhz * 1e6f / bin_hz_));
    if (total_bins_ == 0) {
        total_bins_ = 1024;
    }
}

void WidscanSpectrumComposite::clear() {
    segments_by_lo_.clear();
}

void WidscanSpectrumComposite::update_segment(uint32_t lo_khz, int32_t start_off_hz, uint32_t bin_hz,
                                             const uint8_t* codes, size_t n_codes) {
    if (codes == nullptr || n_codes == 0) {
        return;
    }

    Segment seg;
    seg.lo_khz = lo_khz;
    seg.start_mhz = static_cast<float>(lo_khz) / 1000.0f + static_cast<float>(start_off_hz) / 1e6f;
    seg.bin_hz = static_cast<float>(bin_hz);
    seg.powers_db.resize(n_codes);

    // code -> dBFS = code / 2.0 - 127.5
    for (size_t i = 0; i < n_codes; ++i) {
        seg.powers_db[i] = static_cast<float>(codes[i]) * 0.5f - 127.5f;
    }

    segments_by_lo_[lo_khz] = std::move(seg);
}

void WidscanSpectrumComposite::compose(SweepFrame& out_frame) const {
    out_frame.start_mhz = band_start_mhz_;
    out_frame.bin_hz = bin_hz_;
    out_frame.powers_db.assign(total_bins_, NAN_FLOAT);

    if (segments_by_lo_.empty()) {
        return;
    }

    const float bin_mhz = bin_hz_ / 1e6f;

    // 各出力 bin の中心周波数に対し、最も LO 周波数に近いセグメントの値をサンプリング
    for (size_t x = 0; x < total_bins_; ++x) {
        const float f = band_start_mhz_ + (static_cast<float>(x) + 0.5f) * bin_mhz;
        float best_dist = 1e9f;
        float best_val = NAN_FLOAT;

        for (const auto& pair : segments_by_lo_) {
            const auto& seg = pair.second;
            const float lo_mhz = static_cast<float>(seg.lo_khz) / 1000.0f;
            const float seg_bin_mhz = seg.bin_hz / 1e6f;

            if (seg_bin_mhz <= 0.0f || seg.powers_db.empty()) {
                continue;
            }

            const int k = static_cast<int>(std::round((f - seg.start_mhz) / seg_bin_mhz));
            if (k >= 0 && k < static_cast<int>(seg.powers_db.size())) {
                const float dist = std::abs(f - lo_mhz);
                if (dist < best_dist) {
                    best_dist = dist;
                    best_val = seg.powers_db[k];
                }
            }
        }

        out_frame.powers_db[x] = best_val;
    }

    // わずかな境界隙間や端部の未カバー領域を最寄りの有効値で補完
    float last_valid = NAN_FLOAT;
    for (size_t x = 0; x < total_bins_; ++x) {
        if (!std::isnan(out_frame.powers_db[x])) {
            last_valid = out_frame.powers_db[x];
            break;
        }
    }
    if (!std::isnan(last_valid)) {
        for (size_t x = 0; x < total_bins_; ++x) {
            if (std::isnan(out_frame.powers_db[x])) {
                out_frame.powers_db[x] = last_valid;
            } else {
                last_valid = out_frame.powers_db[x];
            }
        }
    }
}

} // namespace vtxmon
