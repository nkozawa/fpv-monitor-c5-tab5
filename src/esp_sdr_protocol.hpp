#pragma once

#include <cstdint>
#include <string>
#include <vector>
#include "types.hpp"

namespace vtxmon {

class EspSdrProtocol {
public:
    // --- ESP-SDR 公式ファームウェア向けコマンド ---
    static std::string cmd_freq(int mhz);
    static std::string cmd_bandwidth(int mhz);
    static std::string cmd_gain(int manual_index); // index < 0 で HARDWARE AGC
    static std::string cmd_cap16(int samples, int rate_index = 0);
    static std::string cmd_cap20(int samples, int rate_index = 0);
    static std::string cmd_sync(uint64_t nonce);
    static std::string cmd_info();
    static std::string cmd_release();

    // --- キャプチャ応答 (DATA <n> <crc32_hex> <us>) のパース ---
    struct DataHeader {
        int samples{0};
        uint32_t crc32{0};
        uint32_t duration_us{0};
    };
    static bool parse_data_header(const std::string& line, DataHeader& out);

    // --- CRC32 (IEEE 802.3 / zlib互換) ---
    static uint32_t calculate_crc32(const uint8_t* data, size_t length);

    // --- 生 I/Q データのアンパック ---
    // bits: 8 (CAP16) または 10 (CAP20)
    // 戻り値: 成功時に true
    static bool unpack_iq(const uint8_t* raw, size_t byte_len, int n_samples, int bits,
                          std::vector<ComplexSample>& out_samples);

    // --- 液晶版ファームウェア (FPV プロトコル) ---
    static std::string cmd_fpv_probe();
    static std::string cmd_fpv_span(float start_mhz, float stop_mhz,
                                    const std::vector<ChannelDef>& channels);
    static std::string cmd_fpv_gain(int manual_index);
    static std::string cmd_fpv_stream(bool enable);
    static std::string cmd_fpv_nolcd(bool nolcd);

    struct SweepHeader {
        uint32_t seq{0};
        float start_mhz{0.0f};
        size_t nbins{0};
        float bin_hz{0.0f};
        uint32_t t_ms{0};
        uint32_t crc32{0};
    };
    static bool parse_sweep_header(const std::string& line, SweepHeader& out);
    static bool decode_sweep_payload(const uint8_t* raw, size_t byte_len, size_t nbins,
                                    std::vector<float>& out_powers_db);
};

} // namespace vtxmon
