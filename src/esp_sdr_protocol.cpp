#include "esp_sdr_protocol.hpp"
#include <sstream>
#include <iomanip>
#include <array>
#include <cmath>

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

} // namespace

std::string EspSdrProtocol::cmd_freq(int mhz) {
    return "FREQ " + std::to_string(mhz) + "\n";
}

std::string EspSdrProtocol::cmd_bandwidth(int mhz) {
    return "BANDWIDTH " + std::to_string(mhz) + "\n";
}

std::string EspSdrProtocol::cmd_gain(int manual_index) {
    if (manual_index < 0) {
        return "GAIN HARDWARE\n";
    }
    return "GAIN MANUAL " + std::to_string(manual_index) + "\n";
}

std::string EspSdrProtocol::cmd_cap16(int samples, int rate_index) {
    return "CAP16 " + std::to_string(samples) + " " + std::to_string(rate_index) + "\n";
}

std::string EspSdrProtocol::cmd_cap20(int samples, int rate_index) {
    return "CAP20 " + std::to_string(samples) + " " + std::to_string(rate_index) + "\n";
}

std::string EspSdrProtocol::cmd_sync(uint64_t nonce) {
    return "\nSYNC " + std::to_string(nonce) + "\n";
}

std::string EspSdrProtocol::cmd_info() {
    return "INFO\n";
}

std::string EspSdrProtocol::cmd_release() {
    return "RELEASE\n";
}

bool EspSdrProtocol::parse_data_header(const std::string& line, DataHeader& out) {
    // 形式: DATA <n> <crc32_hex> <us>
    std::istringstream iss(line);
    std::string tag;
    std::string crc_hex;
    if (!(iss >> tag >> out.samples >> crc_hex >> out.duration_us)) {
        return false;
    }
    if (tag != "DATA") {
        return false;
    }
    try {
        out.crc32 = static_cast<uint32_t>(std::stoul(crc_hex, nullptr, 16));
    } catch (...) {
        return false;
    }
    return true;
}

uint32_t EspSdrProtocol::calculate_crc32(const uint8_t* data, size_t length) {
    uint32_t crc = 0xFFFFFFFFU;
    for (size_t i = 0; i < length; ++i) {
        uint8_t index = static_cast<uint8_t>((crc ^ data[i]) & 0xFF);
        crc = (crc >> 8) ^ CRC32_TABLE[index];
    }
    return crc ^ 0xFFFFFFFFU;
}

bool EspSdrProtocol::unpack_iq(const uint8_t* raw, size_t byte_len, int n_samples, int bits,
                              std::vector<ComplexSample>& out_samples) {
    if (raw == nullptr || n_samples <= 0) {
        return false;
    }

    if (bits == 8) {
        size_t required = static_cast<size_t>(n_samples) * 2;
        if (byte_len < required) {
            return false;
        }
        out_samples.resize(n_samples);
        constexpr float scale = 4.0f / 512.0f; // 1.0 / 128.0f
        for (int k = 0; k < n_samples; ++k) {
            int8_t i_raw = static_cast<int8_t>(raw[2 * k]);
            int8_t q_raw = static_cast<int8_t>(raw[2 * k + 1]);
            out_samples[k].i = static_cast<float>(i_raw) * scale;
            out_samples[k].q = -static_cast<float>(q_raw) * scale; // Q反転
        }
        return true;
    } else if (bits == 10) {
        size_t m = (static_cast<size_t>(n_samples) + 1) / 2;
        size_t required = (static_cast<size_t>(n_samples) * 20 + 7) / 8;
        if (byte_len < required) {
            return false;
        }
        out_samples.resize(n_samples);

        auto decode_word = [](uint32_t w, float& out_i, float& out_q) {
            int32_t i_raw = static_cast<int32_t>(w & 0x3FF);
            int32_t q_raw = static_cast<int32_t>((w >> 10) & 0x3FF);
            if (i_raw >= 512) i_raw -= 1024;
            if (q_raw >= 512) q_raw -= 1024;
            out_i = static_cast<float>(i_raw) / 512.0f;
            out_q = -static_cast<float>(q_raw) / 512.0f; // Q反転
        };

        for (size_t pair_idx = 0; pair_idx < m; ++pair_idx) {
            size_t base = pair_idx * 5;
            uint64_t b0 = (base < byte_len) ? raw[base] : 0;
            uint64_t b1 = (base + 1 < byte_len) ? raw[base + 1] : 0;
            uint64_t b2 = (base + 2 < byte_len) ? raw[base + 2] : 0;
            uint64_t b3 = (base + 3 < byte_len) ? raw[base + 3] : 0;
            uint64_t b4 = (base + 4 < byte_len) ? raw[base + 4] : 0;
            uint64_t pair = b0 | (b1 << 8) | (b2 << 16) | (b3 << 24) | (b4 << 32);

            uint32_t s0 = static_cast<uint32_t>(pair & 0xFFFFF);
            uint32_t s1 = static_cast<uint32_t>((pair >> 20) & 0xFFFFF);

            int s_idx0 = static_cast<int>(pair_idx * 2);
            int s_idx1 = static_cast<int>(pair_idx * 2 + 1);

            if (s_idx0 < n_samples) {
                decode_word(s0, out_samples[s_idx0].i, out_samples[s_idx0].q);
            }
            if (s_idx1 < n_samples) {
                decode_word(s1, out_samples[s_idx1].i, out_samples[s_idx1].q);
            }
        }
        return true;
    }

    return false;
}

std::string EspSdrProtocol::cmd_fpv_probe() {
    return "FPV?\n";
}

std::string EspSdrProtocol::cmd_fpv_span(float start_mhz, float stop_mhz,
                                        const std::vector<ChannelDef>& channels) {
    std::ostringstream oss;
    oss << "FPV SPAN " << static_cast<int>(std::round(start_mhz)) << " "
        << static_cast<int>(std::round(stop_mhz));
    for (const auto& ch : channels) {
        oss << " " << ch.name;
    }
    oss << "\n";
    return oss.str();
}

std::string EspSdrProtocol::cmd_fpv_gain(int manual_index) {
    if (manual_index < 0) {
        return "FPV GAIN AGC\n";
    }
    return "FPV GAIN " + std::to_string(manual_index) + "\n";
}

std::string EspSdrProtocol::cmd_fpv_stream(bool enable) {
    return enable ? "FPV STREAM ON\n" : "FPV STREAM OFF\n";
}

std::string EspSdrProtocol::cmd_fpv_nolcd(bool nolcd) {
    return nolcd ? "FPV NOLCD\n" : "FPV LCD\n";
}

bool EspSdrProtocol::parse_sweep_header(const std::string& line, SweepHeader& out) {
    // 形式: SWEEP <seq> <start_mhz> <nbins> <bin_hz> <t_ms> <crc32>
    std::istringstream iss(line);
    std::string tag;
    std::string crc_hex;
    if (!(iss >> tag >> out.seq >> out.start_mhz >> out.nbins >> out.bin_hz >> out.t_ms >> crc_hex)) {
        return false;
    }
    if (tag != "SWEEP") {
        return false;
    }
    try {
        out.crc32 = static_cast<uint32_t>(std::stoul(crc_hex, nullptr, 16));
    } catch (...) {
        return false;
    }
    return true;
}

bool EspSdrProtocol::decode_sweep_payload(const uint8_t* raw, size_t byte_len, size_t nbins,
                                         std::vector<float>& out_powers_db) {
    if (raw == nullptr || byte_len < nbins * 2) {
        return false;
    }
    out_powers_db.resize(nbins);
    for (size_t k = 0; k < nbins; ++k) {
        uint16_t code = static_cast<uint16_t>(raw[2 * k] | (raw[2 * k + 1] << 8));
        if (code == 0xFFFF) {
            out_powers_db[k] = NAN_FLOAT;
        } else {
            out_powers_db[k] = static_cast<float>(code) / 100.0f - 200.0f;
        }
    }
    return true;
}

} // namespace vtxmon
