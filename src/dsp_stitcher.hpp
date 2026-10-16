#pragma once

#include <cstdint>
#include <vector>
#include "types.hpp"

namespace vtxmon {

struct DspSettings {
    double rate_hz{80e6};         // ESP32-C5: 80 MS/s
    int step_mhz{15};             // LO spacing between hops
    float usable_mhz{16.0f};      // half-width of each hop kept
    float dc_khz{500.0f};         // half-width around LO dropped (DC hole)
    int nfft{1024};               // FFT size (1024 matches 78.125 kHz bins)
    bool flip{false};             // mirror the spectrum if needed
};

class DspStitcher {
public:
    DspStitcher(float start_mhz, float stop_mhz, const DspSettings& settings = DspSettings{});

    // ホッピング計画: 各ホップの LO 周波数 (MHz)
    [[nodiscard]] const std::vector<int>& los() const { return los_; }

    [[nodiscard]] float start_mhz() const { return start_mhz_; }
    [[nodiscard]] float stop_mhz() const { return stop_mhz_; }
    [[nodiscard]] float bin_hz() const { return static_cast<float>(bin_hz_); }
    [[nodiscard]] size_t output_bins() const { return n_bins_; }

    // 単一ホップの I/Q サンプル群から Welch PSD (dB) を計算 (サイズ: nfft)
    void compute_hop_spectrum_db(const std::vector<ComplexSample>& iq,
                                 std::vector<float>& out_spectrum_db) const;

    // 全ホップのスペクトラム（los_.size() 個）を Min スティッチングして単一の結合スイープを出力
    void stitch(const std::vector<std::vector<float>>& hop_spectra,
                std::vector<float>& out_stitched_db) const;

    // ホッピング計画の計算ヘルパー
    static std::vector<int> calculate_hop_plan_mhz(float start_mhz, float stop_mhz, int step_mhz);

private:
    struct HopMapping {
        std::vector<size_t> out_indices;  // 結合出力のどのビンに対応するか
        std::vector<int> src_fft_indices; // ホップの FFT スペクトラムのどのビンか
    };

    DspSettings settings_;
    float start_mhz_{0.0f};
    float stop_mhz_{0.0f};
    double bin_hz_{0.0};
    size_t n_bins_{0};
    std::vector<int> los_;
    std::vector<HopMapping> hop_maps_;

    // ハミング / ハニング窓
    std::vector<float> hann_window_;
    float window_norm_{1.0f};

    void init_window();
    void build_mapping();
};

} // namespace vtxmon
