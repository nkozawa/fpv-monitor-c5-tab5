#pragma once

#include <cstdint>
#include <string>
#include <vector>
#include <complex>
#include <cmath>
#include <limits>

namespace vtxmon {

constexpr float NAN_FLOAT = std::numeric_limits<float>::quiet_NaN();
// newlib の <sys/param.h> が `HZ` をマクロ定義しているため、この名前は使えない
constexpr double HZ_PER_MHZ = 1e6; // 1 MHz in sweep coordinates

struct ComplexSample {
    float i{0.0f};
    float q{0.0f};

    ComplexSample() = default;
    ComplexSample(float real, float imag) : i(real), q(imag) {}
};

struct ChannelDef {
    std::string name;
    float freq_mhz{0.0f};
};

struct ChannelMeasurement {
    std::string name;
    float nominal_mhz{0.0f};
    bool present{false};
    float centre_mhz{NAN_FLOAT};      // power-weighted centroid
    float offset_mhz{NAN_FLOAT};      // centre - nominal
    float peak_mhz{NAN_FLOAT};
    float peak_dbm{NAN_FLOAT};
    float floor_dbm{NAN_FLOAT};
    float snr_db{NAN_FLOAT};
    float bw_db3_mhz{NAN_FLOAT};
    float bw_db20_mhz{NAN_FLOAT};
    std::string reason;               // why not present if false
};

struct SweepSpan {
    float start_mhz{5600.0f};
    float stop_mhz{5850.0f};
    float bin_hz{100000.0f};          // default bin width
};

struct SweepFrame {
    float start_mhz{0.0f};
    float bin_hz{0.0f};
    std::vector<float> powers_db;     // dB full scale
    uint32_t timestamp_ms{0};
    uint32_t seq{0};

    [[nodiscard]] size_t size() const { return powers_db.size(); }
    [[nodiscard]] float freq_mhz(size_t index) const {
        return start_mhz + static_cast<float>(index) * (bin_hz / 1e6f);
    }
};

} // namespace vtxmon
