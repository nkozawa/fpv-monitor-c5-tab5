#include "channel_analysis.hpp"
#include <algorithm>
#include <cmath>
#include <sstream>
#include <iomanip>

namespace vtxmon {

namespace {

inline bool is_valid(float v) {
    return !std::isnan(v) && !std::isinf(v);
}

float edge_interp(const std::vector<float>& powers, int inside, int outside, float threshold) {
    if (outside < 0 || outside >= static_cast<int>(powers.size()) || !is_valid(powers[outside])) {
        return 0.0f;
    }
    float p_in = powers[inside];
    float p_out = powers[outside];
    if (p_in <= p_out) {
        return 0.0f;
    }
    float frac = (p_in - threshold) / (p_in - p_out);
    return std::clamp(frac, 0.0f, 1.0f);
}

void contiguous_run(const std::vector<float>& powers, int peak_i, float threshold, int& out_lo, int& out_hi) {
    out_lo = peak_i;
    while (out_lo - 1 >= 0 && is_valid(powers[out_lo - 1]) && powers[out_lo - 1] >= threshold) {
        --out_lo;
    }
    out_hi = peak_i;
    int n = static_cast<int>(powers.size());
    while (out_hi + 1 < n && is_valid(powers[out_hi + 1]) && powers[out_hi + 1] >= threshold) {
        ++out_hi;
    }
}

float calc_bandwidth(const std::vector<float>& powers, float bin_mhz, int peak_i, float peak, float drop_db) {
    float threshold = peak - drop_db;
    int lo = 0, hi = 0;
    contiguous_run(powers, peak_i, threshold, lo, hi);
    if (bin_mhz <= 0.0f) {
        return NAN_FLOAT;
    }
    float left = edge_interp(powers, lo, lo - 1, threshold);
    float right = edge_interp(powers, hi, hi + 1, threshold);
    return (static_cast<float>(hi - lo) + left + right) * bin_mhz;
}

float window_centroid(const std::vector<float>& powers,
                      float start_mhz,
                      float bin_mhz,
                      float centre_mhz,
                      float half_mhz,
                      float floor_dbm,
                      float noise_margin_db) {
    double sum_weighted_f = 0.0;
    double total_weight = 0.0;

    int n = static_cast<int>(powers.size());
    float min_f = centre_mhz - half_mhz;
    float max_f = centre_mhz + half_mhz;
    float threshold = floor_dbm + noise_margin_db;
    double floor_linear = std::pow(10.0, static_cast<double>(floor_dbm) / 10.0);

    for (int i = 0; i < n; ++i) {
        float f = start_mhz + (static_cast<float>(i) + 0.5f) * bin_mhz;
        if (f <= min_f || f >= max_f) {
            continue;
        }
        float p = powers[i];
        if (is_valid(p) && p > threshold) {
            double p_linear = std::pow(10.0, static_cast<double>(p) / 10.0);
            double w = std::max(0.0, p_linear - floor_linear);
            sum_weighted_f += static_cast<double>(f) * w;
            total_weight += w;
        }
    }

    if (total_weight <= 0.0) {
        return NAN_FLOAT;
    }
    return static_cast<float>(sum_weighted_f / total_weight);
}

} // namespace

float ChannelAnalysis::estimate_noise_floor(const std::vector<float>& powers_db) {
    std::vector<float> valid;
    valid.reserve(powers_db.size());
    for (float v : powers_db) {
        if (is_valid(v)) {
            valid.push_back(v);
        }
    }
    if (valid.empty()) {
        return NAN_FLOAT;
    }
    size_t mid = valid.size() / 2;
    std::nth_element(valid.begin(), valid.begin() + mid, valid.end());
    return valid[mid];
}

ChannelMeasurement ChannelAnalysis::measure_channel(
    const std::vector<float>& powers_db,
    float start_mhz,
    float bin_hz,
    const ChannelDef& channel,
    const AnalysisSettings& settings,
    float forced_floor_dbm) {

    ChannelMeasurement m;
    m.name = channel.name;
    m.nominal_mhz = channel.freq_mhz;
    m.present = false;

    float bin_mhz = bin_hz / 1e6f;
    if (bin_mhz <= 0.0f || powers_db.empty()) {
        m.reason = "invalid grid or empty data";
        return m;
    }

    float floor = is_valid(forced_floor_dbm) ? forced_floor_dbm : estimate_noise_floor(powers_db);
    m.floor_dbm = floor;

    // 検索ウィンドウのインデックス範囲
    int lo_idx = std::max(0, static_cast<int>(std::floor((channel.freq_mhz - settings.search_mhz - start_mhz) / bin_mhz)));
    int hi_idx = std::min(static_cast<int>(powers_db.size()),
                          static_cast<int>(std::ceil((channel.freq_mhz + settings.search_mhz - start_mhz) / bin_mhz)));

    if (lo_idx >= hi_idx) {
        m.reason = "no data in this window";
        return m;
    }

    // 検索窓内のピーク探索
    float peak = -999.0f;
    int peak_i = -1;
    for (int i = lo_idx; i < hi_idx; ++i) {
        float p = powers_db[i];
        if (is_valid(p) && p > peak) {
            peak = p;
            peak_i = i;
        }
    }

    if (peak_i < 0) {
        m.reason = "no valid samples in search window";
        return m;
    }

    m.peak_mhz = start_mhz + (static_cast<float>(peak_i) + 0.5f) * bin_mhz;
    m.peak_dbm = peak;
    m.snr_db = peak - floor;

    if (!is_valid(m.snr_db) || m.snr_db < settings.min_snr_db) {
        std::ostringstream oss;
        oss << "SNR " << std::fixed << std::setprecision(1) << m.snr_db
            << " < " << settings.min_snr_db << " dB";
        m.reason = oss.str();
        return m;
    }

    // 帯域幅の計算
    m.bw_db3_mhz = calc_bandwidth(powers_db, bin_mhz, peak_i, peak, 3.0f);
    m.bw_db20_mhz = calc_bandwidth(powers_db, bin_mhz, peak_i, peak, 20.0f);

    if (settings.min_bw_mhz > 0.0f && (std::isnan(m.bw_db3_mhz) || m.bw_db3_mhz < settings.min_bw_mhz)) {
        m.reason = "-3 dB bandwidth below threshold";
        return m;
    }

    // 重心計算 (パス1: 検索窓全体)
    float half = std::min(settings.centroid_half_mhz, settings.search_mhz);
    float centre = window_centroid(powers_db, start_mhz, bin_mhz, channel.freq_mhz,
                                   settings.search_mhz, floor, settings.noise_margin_db);

    if (is_valid(centre)) {
        // 検索窓外に飛び出さないようクリップ
        centre = std::clamp(centre, channel.freq_mhz - settings.search_mhz,
                                    channel.freq_mhz + settings.search_mhz);
        // 重心計算 (パス2: パス1の結果を中心に再計算)
        float refined = window_centroid(powers_db, start_mhz, bin_mhz, centre,
                                        half, floor, settings.noise_margin_db);
        if (is_valid(refined)) {
            centre = refined;
        }
    } else {
        centre = m.peak_mhz;
    }

    m.present = true;
    m.centre_mhz = centre;
    m.offset_mhz = centre - channel.freq_mhz;
    return m;
}

std::vector<ChannelMeasurement> ChannelAnalysis::measure_all(
    const std::vector<float>& powers_db,
    float start_mhz,
    float bin_hz,
    const std::vector<ChannelDef>& channels,
    const AnalysisSettings& settings) {

    float floor = estimate_noise_floor(powers_db);
    std::vector<ChannelMeasurement> results;
    results.reserve(channels.size());

    for (const auto& ch : channels) {
        results.push_back(measure_channel(powers_db, start_mhz, bin_hz, ch, settings, floor));
    }
    return results;
}

float ChannelAnalysis::auto_search_halfwidth(const std::vector<ChannelDef>& channels, float requested) {
    if (channels.size() < 2) {
        return requested;
    }
    std::vector<float> freqs;
    freqs.reserve(channels.size());
    for (const auto& ch : channels) {
        freqs.push_back(ch.freq_mhz);
    }
    std::sort(freqs.begin(), freqs.end());

    float min_gap = 9999.0f;
    for (size_t i = 1; i < freqs.size(); ++i) {
        min_gap = std::min(min_gap, freqs[i] - freqs[i - 1]);
    }
    return std::min(requested, std::max(1.0f, min_gap / 2.0f));
}

} // namespace vtxmon
