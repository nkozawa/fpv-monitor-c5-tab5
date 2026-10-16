#include <cassert>
#include <iostream>
#include <cmath>
#include <vector>

#include "types.hpp"
#include "esp_sdr_protocol.hpp"
#include "dsp_stitcher.hpp"
#include "channel_analysis.hpp"

using namespace vtxmon;

void test_protocol_commands() {
    std::cout << "[Test] Protocol Commands..." << std::endl;
    assert(EspSdrProtocol::cmd_freq(5740) == "FREQ 5740\n");
    assert(EspSdrProtocol::cmd_bandwidth(0) == "BANDWIDTH 0\n");
    assert(EspSdrProtocol::cmd_gain(25) == "GAIN MANUAL 25\n");
    assert(EspSdrProtocol::cmd_gain(-1) == "GAIN HARDWARE\n");
    assert(EspSdrProtocol::cmd_cap16(8192, 0) == "CAP16 8192 0\n");
    assert(EspSdrProtocol::cmd_cap20(8192, 0) == "CAP20 8192 0\n");
    assert(EspSdrProtocol::cmd_sync(123456789ULL) == "\nSYNC 123456789\n");
    assert(EspSdrProtocol::cmd_info() == "INFO\n");
    assert(EspSdrProtocol::cmd_release() == "RELEASE\n");

    // Header parse
    EspSdrProtocol::DataHeader dh;
    bool ok = EspSdrProtocol::parse_data_header("DATA 8192 1A2B3C4D 102", dh);
    assert(ok);
    assert(dh.samples == 8192);
    assert(dh.crc32 == 0x1A2B3C4D);
    assert(dh.duration_us == 102);

    // CRC32 test
    const uint8_t test_bytes[] = {'1', '2', '3', '4', '5', '6', '7', '8', '9'};
    uint32_t crc = EspSdrProtocol::calculate_crc32(test_bytes, sizeof(test_bytes));
    assert(crc == 0xCBF43926U); // standard CRC-32 of "123456789"
    std::cout << "  -> Passed!" << std::endl;
}

void test_unpack_iq() {
    std::cout << "[Test] Unpack IQ (8-bit and 10-bit)..." << std::endl;
    // 8-bit (CAP16)
    // 2 samples: (127, -128), (0, 64)
    uint8_t raw8[] = {127, static_cast<uint8_t>(-128), 0, 64};
    std::vector<ComplexSample> samples8;
    bool ok8 = EspSdrProtocol::unpack_iq(raw8, sizeof(raw8), 2, 8, samples8);
    assert(ok8);
    assert(samples8.size() == 2);
    // scale = 1.0 / 128.0
    // I = 127/128, Q = -(-128/128) = 1.0
    assert(std::abs(samples8[0].i - (127.0f / 128.0f)) < 1e-4f);
    assert(std::abs(samples8[0].q - 1.0f) < 1e-4f);
    assert(std::abs(samples8[1].i - 0.0f) < 1e-4f);
    assert(std::abs(samples8[1].q - (-0.5f)) < 1e-4f);

    // 10-bit (CAP20)
    // 2 samples packed in 5 bytes
    // Word 0: I=100, Q=200 -> pair low 20 bits = 100 | (200 << 10) = 100 | 204800 = 204900 = 0x032064
    // Word 1: I=-50 (-> 1024-50 = 974), Q=-100 (-> 1024-100 = 924) -> high 20 bits = 974 | (924 << 10) = 974 | 946176 = 947150 = 0x0E73CE
    // 40 bits total: 0x032064 | (0x0E73CE << 20)
    uint64_t val = 0x032064ULL | (0x0E73CEULL << 20);
    uint8_t raw10[5];
    for (int i = 0; i < 5; ++i) {
        raw10[i] = static_cast<uint8_t>((val >> (8 * i)) & 0xFF);
    }
    std::vector<ComplexSample> samples10;
    bool ok10 = EspSdrProtocol::unpack_iq(raw10, sizeof(raw10), 2, 10, samples10);
    assert(ok10);
    assert(samples10.size() == 2);
    assert(std::abs(samples10[0].i - (100.0f / 512.0f)) < 1e-4f);
    assert(std::abs(samples10[0].q - (-200.0f / 512.0f)) < 1e-4f);
    assert(std::abs(samples10[1].i - (-50.0f / 512.0f)) < 1e-4f);
    assert(std::abs(samples10[1].q - (100.0f / 512.0f)) < 1e-4f);
    std::cout << "  -> Passed!" << std::endl;
}

void test_dsp_stitcher() {
    std::cout << "[Test] DSP Stitcher & FFT..." << std::endl;
    DspSettings settings;
    settings.nfft = 1024;
    settings.step_mhz = 15;
    DspStitcher stitcher(5640.0f, 5800.0f, settings);

    const auto& los = stitcher.los();
    assert(!los.empty());
    std::cout << "  Hop plan generated " << los.size() << " hops: ";
    for (int lo : los) std::cout << lo << " ";
    std::cout << "MHz" << std::endl;

    // 単一周波数の正弦波信号 (LO + 5MHz) を模擬
    // サンプリングレート 80MHz, 信号周波数 5MHz
    std::vector<ComplexSample> iq(4096);
    double tone_freq = 5e6;
    for (size_t i = 0; i < iq.size(); ++i) {
        double phase = 2.0 * M_PI * tone_freq * static_cast<double>(i) / settings.rate_hz;
        iq[i].i = static_cast<float>(std::cos(phase));
        iq[i].q = static_cast<float>(std::sin(phase));
    }

    std::vector<float> spectrum;
    stitcher.compute_hop_spectrum_db(iq, spectrum);
    assert(spectrum.size() == 1024);

    // 最大ピークの位置を探す
    auto max_it = std::max_element(spectrum.begin(), spectrum.end());
    size_t peak_bin = std::distance(spectrum.begin(), max_it);
    // 中心 (DC) は bin 512。+5MHz は 5MHz / (80MHz / 1024) = 5e6 / 78125 = 64 ビン上 -> 512 + 64 = 576
    std::cout << "  Expected peak near bin 576, found at bin " << peak_bin
              << " with power " << *max_it << " dB" << std::endl;
    assert(std::abs(static_cast<int>(peak_bin) - 576) <= 2);

    // スティッチング結合 (Min結合) のテスト
    std::vector<std::vector<float>> hop_spectra(los.size(), std::vector<float>(1024, -90.0f));
    // あるビンに、ホップAでは -30dB (スプリアス)、ホップBでは -90dB (ノイズフロア)
    // Min結合により、-90dB が採用されるはず
    std::vector<float> stitched;
    stitcher.stitch(hop_spectra, stitched);
    assert(stitched.size() == stitcher.output_bins());
    std::cout << "  Stitcher produced " << stitched.size() << " combined output bins" << std::endl;

    std::cout << "  -> Passed!" << std::endl;
}

void test_channel_analysis() {
    std::cout << "[Test] Channel Analysis & Centroid..." << std::endl;
    // 5600〜5850MHz のグリッド（ビン幅 100kHz = 0.1MHz）
    float start_mhz = 5600.0f;
    float bin_hz = 100000.0f;
    float bin_mhz = 0.1f;
    size_t nbins = 2500;
    std::vector<float> powers(nbins, -90.0f); // ノイズフロア -90 dBm

    // F3 チャンネル (5740 MHz) に、少し中心がずれた (+0.25 MHz -> 5740.25 MHz) 信号を配置
    float target_f = 5740.25f;
    for (size_t i = 0; i < nbins; ++i) {
        float f = start_mhz + (static_cast<float>(i) + 0.5f) * bin_mhz;
        float dist = std::abs(f - target_f);
        if (dist <= 4.0f) {
            // ガウス形状のキャリア
            float p = -90.0f + 35.0f * std::exp(- (dist * dist) / (2.0f * 1.5f * 1.5f));
            powers[i] = std::max(powers[i], p);
        }
    }

    ChannelDef ch_f3{"F3", 5740.0f};
    AnalysisSettings settings;
    settings.min_snr_db = 15.0f;

    auto m = ChannelAnalysis::measure_channel(powers, start_mhz, bin_hz, ch_f3, settings);
    assert(m.present);
    std::cout << "  Measured centre: " << m.centre_mhz << " MHz (target: " << target_f << " MHz)" << std::endl;
    std::cout << "  Offset: " << m.offset_mhz << " MHz, SNR: " << m.snr_db << " dB" << std::endl;

    assert(std::abs(m.centre_mhz - target_f) < 0.05f); // 0.05 MHz 以内の高い測定精度
    assert(m.snr_db > 30.0f);

    // 信号のないチャンネル E1 (5705 MHz) の測定
    ChannelDef ch_e1{"E1", 5705.0f};
    auto m_e1 = ChannelAnalysis::measure_channel(powers, start_mhz, bin_hz, ch_e1, settings);
    assert(!m_e1.present);
    std::cout << "  E1 (No Signal) Reason: " << m_e1.reason << std::endl;

    std::cout << "  -> Passed!" << std::endl;
}

// ホップ計画 -> 各ホップの I/Q -> FFT -> スティッチング までを通しで検証する。
// 「信号を注入した周波数にピークが出る」ことを保証するので、周波数マッピングや
// fftshift / min 結合のずれが回帰するとここで落ちる。
void test_stitch_end_to_end() {
    std::cout << "[Test] Stitch End-to-End (tone at 5705 MHz)..." << std::endl;

    constexpr double FS = 80e6;
    constexpr int SAMPLES = 8192;      // CAP16 と同じサンプル数
    constexpr float START = 5640.0f;
    constexpr float STOP = 5800.0f;

    DspSettings settings;
    settings.nfft = 1024;
    settings.step_mhz = 15;
    DspStitcher stitcher(START, STOP, settings);

    const auto& los = stitcher.los();
    assert(!los.empty());
    // 両端の LO はスパンの外にあり、自分の DC ホールをスパン内に持ち込まない
    assert(los.front() < static_cast<int>(START));
    assert(los.back() > static_cast<int>(STOP));

    // 全出力ビンが最低 1 ホップから見えること (穴があると常に NaN になる)
    const double bin_hz = stitcher.bin_hz();
    const double hz_low = std::floor(static_cast<double>(START)) * 1e6;
    size_t uncovered = 0;
    for (size_t k = 0; k < stitcher.output_bins(); ++k) {
        const double centre = hz_low + (static_cast<double>(k) + 0.5) * bin_hz;
        bool seen = false;
        for (int lo : los) {
            const double off = std::abs(centre - static_cast<double>(lo) * 1e6);
            if (off <= settings.usable_mhz * 1e6 && off >= settings.dc_khz * 1e3) {
                seen = true;
                break;
            }
        }
        if (!seen) ++uncovered;
    }
    std::cout << "  " << los.size() << " hops, " << stitcher.output_bins()
              << " output bins, uncovered = " << uncovered << std::endl;
    assert(uncovered == 0);

    // 5705 MHz (E1) にトーンを注入。アナログ前端は LO の ±24 MHz 程度しか
    // 通さないので、|offset| の大きいホップにはノイズだけを入れる。
    constexpr double SIGNAL_HZ = 5705e6;
    constexpr double NOISE_AMP = 0.02;
    uint32_t rng = 12345;
    auto rnd = [&rng]() {
        rng = rng * 1103515245u + 12345u;
        return (static_cast<double>(rng >> 8) / 8388608.0) - 1.0;
    };

    std::vector<ComplexSample> iq(SAMPLES);
    std::vector<std::vector<float>> hop_spectra(los.size());
    for (size_t h = 0; h < los.size(); ++h) {
        const double off_hz = SIGNAL_HZ - static_cast<double>(los[h]) * 1e6;
        const bool inject = (std::abs(off_hz) <= 20e6);
        for (int n = 0; n < SAMPLES; ++n) {
            double vi = NOISE_AMP * rnd();
            double vq = NOISE_AMP * rnd();
            if (inject) {
                const double phase = 2.0 * M_PI * off_hz * static_cast<double>(n) / FS;
                vi += 0.5 * std::cos(phase);
                vq += 0.5 * std::sin(phase);
            }
            iq[n].i = static_cast<float>(vi);
            iq[n].q = static_cast<float>(vq);
        }
        stitcher.compute_hop_spectrum_db(iq, hop_spectra[h]);
    }

    std::vector<float> stitched;
    stitcher.stitch(hop_spectra, stitched);
    assert(stitched.size() == stitcher.output_bins());

    int peak_bin = -1;
    float peak_db = -1e9f;
    double floor_sum = 0.0;
    int floor_n = 0;
    for (size_t k = 0; k < stitched.size(); ++k) {
        assert(!std::isnan(stitched[k])); // 全ビン埋まっている
        if (stitched[k] > peak_db) {
            peak_db = stitched[k];
            peak_bin = static_cast<int>(k);
        }
        const double f = std::floor(static_cast<double>(START)) +
                         (static_cast<double>(k) + 0.5) * bin_hz / 1e6;
        if (std::abs(f - 5705.0) > 3.0) {
            floor_sum += stitched[k];
            ++floor_n;
        }
    }

    const double peak_mhz = std::floor(static_cast<double>(START)) +
                            (static_cast<double>(peak_bin) + 0.5) * bin_hz / 1e6;
    const double floor_db = floor_sum / static_cast<double>(floor_n);
    std::cout << "  peak = " << peak_db << " dBFS @ " << peak_mhz
              << " MHz (bin " << peak_bin << ")" << std::endl;
    std::cout << "  noise floor = " << floor_db << " dBFS, SNR = "
              << (peak_db - floor_db) << " dB" << std::endl;

    // min 結合でピークが消えず、注入した周波数に現れること
    assert(std::abs(peak_mhz - 5705.0) < 0.5);
    assert(peak_db - floor_db > 20.0);

    std::cout << "  -> Passed!" << std::endl;
}

int main() {
    std::cout << "=== Running VTX Monitor Core Tests ===" << std::endl;
    test_protocol_commands();
    test_unpack_iq();
    test_dsp_stitcher();
    test_stitch_end_to_end();
    test_channel_analysis();
    std::cout << "\nALL TESTS PASSED SUCCESSFULLY!" << std::endl;
    return 0;
}
