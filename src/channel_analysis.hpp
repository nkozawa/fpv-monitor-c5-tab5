#pragma once

#include <vector>
#include <string>
#include "types.hpp"

namespace vtxmon {

struct AnalysisSettings {
    float search_mhz{8.0f};          // チャンネル周辺のピーク検索半幅
    float min_snr_db{18.0f};         // 信号存在と判定する最小 SNR (dB)
    float noise_margin_db{3.0f};     // 重心計算時にノイズフロアより上に要求するマージン
    float centroid_half_mhz{7.5f};   // 重心計算の洗練窓半幅
    float min_bw_mhz{0.0f};          // 最小 -3dB 帯域幅フィルタ (0で無効)
};

class ChannelAnalysis {
public:
    // ノイズフロア (中央値) を推定
    static float estimate_noise_floor(const std::vector<float>& powers_db);

    // 単一チャンネルの測定・在り判定
    static ChannelMeasurement measure_channel(
        const std::vector<float>& powers_db,
        float start_mhz,
        float bin_hz,
        const ChannelDef& channel,
        const AnalysisSettings& settings = AnalysisSettings{},
        float forced_floor_dbm = NAN_FLOAT);

    // 複数チャンネルの一括測定
    static std::vector<ChannelMeasurement> measure_all(
        const std::vector<float>& powers_db,
        float start_mhz,
        float bin_hz,
        const std::vector<ChannelDef>& channels,
        const AnalysisSettings& settings = AnalysisSettings{});

    // 隣接チャンネルとの干渉を防ぐための検索半幅の自動調整
    static float auto_search_halfwidth(const std::vector<ChannelDef>& channels, float requested);
};

} // namespace vtxmon
