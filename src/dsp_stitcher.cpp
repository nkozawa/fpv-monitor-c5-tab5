#include "dsp_stitcher.hpp"
#include <cmath>
#include <algorithm>
#include <complex>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

namespace vtxmon {

namespace {

// ポータブルな基数2 Cooley-Tukey FFT (Mac/PC およびテスト用)
void fft_radix2(std::vector<std::complex<float>>& a, bool invert) {
    size_t n = a.size();
    if (n <= 1) return;

    // ビット反転並べ替え
    for (size_t i = 1, j = 0; i < n; ++i) {
        size_t bit = n >> 1;
        for (; j & bit; bit >>= 1) {
            j ^= bit;
        }
        j ^= bit;
        if (i < j) {
            std::swap(a[i], a[j]);
        }
    }

    // バタフライ演算
    for (size_t len = 2; len <= n; len <<= 1) {
        double ang = 2.0 * M_PI / len * (invert ? 1.0 : -1.0);
        std::complex<float> wlen(static_cast<float>(std::cos(ang)),
                                static_cast<float>(std::sin(ang)));
        for (size_t i = 0; i < n; i += len) {
            std::complex<float> w(1.0f, 0.0f);
            for (size_t j = 0; j < len / 2; ++j) {
                std::complex<float> u = a[i + j];
                std::complex<float> v = a[i + j + len / 2] * w;
                a[i + j] = u + v;
                a[i + j + len / 2] = u - v;
                w *= wlen;
            }
        }
    }

    if (invert) {
        for (auto& x : a) {
            x /= static_cast<float>(n);
        }
    }
}

} // namespace

std::vector<int> DspStitcher::calculate_hop_plan_mhz(float start_mhz, float stop_mhz, int step_mhz) {
    std::vector<int> los;
    int first_lo = static_cast<int>(std::floor(start_mhz)) - step_mhz / 2;
    los.push_back(first_lo);
    while (los.back() < stop_mhz + step_mhz / 2.0f) {
        los.push_back(los.back() + step_mhz);
    }
    return los;
}

DspStitcher::DspStitcher(float start_mhz, float stop_mhz, const DspSettings& settings)
    : settings_(settings),
      start_mhz_(start_mhz),
      stop_mhz_(stop_mhz),
      bin_hz_(settings.rate_hz / settings.nfft) {
    los_ = calculate_hop_plan_mhz(start_mhz_, stop_mhz_, settings_.step_mhz);
    init_window();
    build_mapping();
}

void DspStitcher::init_window() {
    hann_window_.resize(settings_.nfft);
    double sum = 0.0;
    for (int i = 0; i < settings_.nfft; ++i) {
        // Hann window: 0.5 * (1 - cos(2*pi*i / N))
        float w = 0.5f * (1.0f - static_cast<float>(std::cos(2.0 * M_PI * i / settings_.nfft)));
        hann_window_[i] = w;
        sum += w;
    }
    window_norm_ = static_cast<float>(sum * sum);
}

void DspStitcher::build_mapping() {
    double hz_low = std::floor(start_mhz_) * 1e6;
    double span_hz = (std::ceil(stop_mhz_) - std::floor(start_mhz_)) * 1e6;
    n_bins_ = std::max<size_t>(1, static_cast<size_t>(std::round(span_hz / bin_hz_)));

    hop_maps_.resize(los_.size());

    double usable_hz = settings_.usable_mhz * 1e6;
    double dc_hz = settings_.dc_khz * 1e3;

    for (size_t hop_idx = 0; hop_idx < los_.size(); ++hop_idx) {
        double lo_hz = static_cast<double>(los_[hop_idx]) * 1e6;
        auto& map = hop_maps_[hop_idx];
        map.out_indices.clear();
        map.src_fft_indices.clear();

        for (size_t out_k = 0; out_k < n_bins_; ++out_k) {
            double freq_k = hz_low + (static_cast<double>(out_k) + 0.5) * bin_hz_;
            double offset = freq_k - lo_hz;
            double abs_off = std::abs(offset);

            if (abs_off <= usable_hz && abs_off >= dc_hz) {
                int src = static_cast<int>(std::round(offset / bin_hz_)) + settings_.nfft / 2;
                if (src >= 0 && src < settings_.nfft) {
                    map.out_indices.push_back(out_k);
                    map.src_fft_indices.push_back(src);
                }
            }
        }
    }
}

void DspStitcher::compute_hop_spectrum_db(const std::vector<ComplexSample>& iq,
                                         std::vector<float>& out_spectrum_db) const {
    out_spectrum_db.assign(settings_.nfft, -150.0f);
    if (iq.size() < static_cast<size_t>(settings_.nfft)) {
        return;
    }

    size_t nfft = settings_.nfft;
    size_t hop = nfft / 2; // 50% overlap
    size_t nseg = (iq.size() - nfft) / hop + 1;
    if (nseg == 0) return;

    std::vector<double> acc(nfft, 0.0);
    std::vector<std::complex<float>> seg_buf(nfft);

    for (size_t k = 0; k < nseg; ++k) {
        size_t start = k * hop;
        // 平均除去 (DCオフセット低減)
        float sum_i = 0.0f;
        float sum_q = 0.0f;
        for (size_t i = 0; i < nfft; ++i) {
            sum_i += iq[start + i].i;
            sum_q += iq[start + i].q;
        }
        float mean_i = sum_i / static_cast<float>(nfft);
        float mean_q = sum_q / static_cast<float>(nfft);

        for (size_t i = 0; i < nfft; ++i) {
            float val_i = (iq[start + i].i - mean_i) * hann_window_[i];
            float val_q = (iq[start + i].q - mean_q) * hann_window_[i];
            seg_buf[i] = std::complex<float>(val_i, val_q);
        }

        // FFT 実行
        fft_radix2(seg_buf, false);

        // パワースペクトル積算
        for (size_t i = 0; i < nfft; ++i) {
            double p = static_cast<double>(seg_buf[i].real()) * seg_buf[i].real() +
                       static_cast<double>(seg_buf[i].imag()) * seg_buf[i].imag();
            acc[i] += p;
        }
    }

    double norm_factor = static_cast<double>(nseg) * window_norm_;
    if (norm_factor <= 0.0) norm_factor = 1.0;

    // fftshift & dB変換
    for (size_t i = 0; i < nfft; ++i) {
        size_t shifted_idx = (i + nfft / 2) % nfft;
        double p = acc[shifted_idx] / norm_factor;
        if (p < 1e-15) p = 1e-15;
        out_spectrum_db[i] = static_cast<float>(10.0 * std::log10(p));
    }
}

void DspStitcher::stitch(const std::vector<std::vector<float>>& hop_spectra,
                        std::vector<float>& out_stitched_db) const {
    out_stitched_db.assign(n_bins_, NAN_FLOAT);

    size_t count = std::min(los_.size(), hop_spectra.size());
    for (size_t hop_idx = 0; hop_idx < count; ++hop_idx) {
        const auto& map = hop_maps_[hop_idx];
        const auto& spec = hop_spectra[hop_idx];
        if (spec.size() < static_cast<size_t>(settings_.nfft)) continue;

        for (size_t k = 0; k < map.out_indices.size(); ++k) {
            size_t out_idx = map.out_indices[k];
            int src_idx = map.src_fft_indices[k];
            float val = spec[src_idx];

            if (std::isnan(out_stitched_db[out_idx])) {
                out_stitched_db[out_idx] = val;
            } else {
                // Min 結合: 2つのホップのうち小さい方を採用してスプリアス/イメージを除去
                out_stitched_db[out_idx] = std::min(out_stitched_db[out_idx], val);
            }
        }
    }

    if (settings_.flip) {
        std::reverse(out_stitched_db.begin(), out_stitched_db.end());
    }
}

} // namespace vtxmon
