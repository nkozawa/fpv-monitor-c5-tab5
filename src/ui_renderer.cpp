#include "ui_renderer.hpp"
#include <cmath>
#include <algorithm>
#include <cstdio>
#include <sstream>
#include <iomanip>

#if defined(ESP_PLATFORM)
#include <M5GFX.h>
#include "esp_log.h"

static const char* TAG = "UiRenderer";
static M5GFX display;
static M5Canvas canvas_header(&display);
static M5Canvas canvas_cards(&display);
static M5Canvas canvas_spectrum(&display);
static M5Canvas canvas_waterfall(&display);
static M5Canvas canvas_footer(&display);
#endif

namespace vtxmon {

namespace {

// 色定数 (RGB565)
constexpr uint16_t COLOR_BG_DARK     = 0x0842; // #080810 非常に濃いグレー
constexpr uint16_t COLOR_CARD_BG     = 0x18E3; // #181c20 カード背景
constexpr uint16_t COLOR_BORDER_GRAY = 0x39E7; // #383c40
constexpr uint16_t COLOR_GREEN_ON    = 0x07E0; // #00FF00 アクティブグリーン
constexpr uint16_t COLOR_CYAN_LINE   = 0x07FF; // #00FFFF スペクトラム折れ線
constexpr uint16_t COLOR_GRID_LINE   = 0x2124; // #202028 暗いグリッド線
constexpr uint16_t COLOR_TEXT_WHITE  = 0xFFFF;
constexpr uint16_t COLOR_TEXT_GRAY   = 0x8410;
constexpr uint16_t COLOR_TEXT_YELLOW = 0xFFE0;

// 表示幅の 1/8 程度になる「見やすい」グリッド間隔を選ぶ
float nice_grid_step(float span_mhz) {
    constexpr float CANDIDATES[] = {0.5f, 1.0f, 2.0f, 2.5f, 5.0f, 10.0f, 20.0f, 50.0f, 100.0f};
    const float target = span_mhz / 8.0f;
    for (float c : CANDIDATES) {
        if (c >= target) {
            return c;
        }
    }
    return CANDIDATES[sizeof(CANDIDATES) / sizeof(CANDIDATES[0]) - 1];
}

bool in_rect(int x, int y, int rx, int ry, int rw, int rh) {
    return x >= rx && x < rx + rw && y >= ry && y < ry + rh;
}

// ゲイン index の表示文字列 (-1 は AGC)
void format_gain(int index, char* buf, size_t size) {
    if (index < 0) {
        snprintf(buf, size, "AGC");
    } else {
        snprintf(buf, size, "%d", index);
    }
}

} // namespace

UiRenderer::UiRenderer() = default;
UiRenderer::~UiRenderer() = default;

uint16_t UiRenderer::db_to_rgb565(float db, float min_db, float max_db) {
    if (std::isnan(db)) return 0x0000;
    float t = (db - min_db) / (max_db - min_db);
    t = std::clamp(t, 0.0f, 1.0f);

    uint8_t r = 0, g = 0, b = 0;
    if (t < 0.25f) {
        // 黒 -> 青
        float f = t / 0.25f;
        b = static_cast<uint8_t>(f * 255.0f);
    } else if (t < 0.5f) {
        // 青 -> シアン -> 緑
        float f = (t - 0.25f) / 0.25f;
        g = static_cast<uint8_t>(f * 255.0f);
        b = static_cast<uint8_t>((1.0f - f) * 255.0f);
    } else if (t < 0.75f) {
        // 緑 -> 黄
        float f = (t - 0.5f) / 0.25f;
        r = static_cast<uint8_t>(f * 255.0f);
        g = 255;
    } else {
        // 黄 -> 赤 -> 白
        float f = (t - 0.75f) / 0.25f;
        r = 255;
        g = static_cast<uint8_t>((1.0f - f * 0.7f) * 255.0f);
        b = static_cast<uint8_t>(f * 255.0f);
    }
    return ((r >> 3) << 11) | ((g >> 2) << 5) | (b >> 3);
}

bool UiRenderer::init() {
#if defined(ESP_PLATFORM)
    ESP_LOGI(TAG, "Initializing M5GFX for TAB 5 (1280x720)...");
    // begin() が失敗すると panel が nullptr のままになり、以降の setRotation 等で
    // クラッシュループする。ここで止めて理由をログに残す。
    if (!display.begin()) {
        ESP_LOGE(TAG, "M5GFX begin() failed: display not detected (PSRAM enabled?)");
        return false;
    }
    ESP_LOGI(TAG, "Panel detected, native %dx%d", display.width(), display.height());
    display.setRotation(1); // 横向き (Tab5 パネルは native 720x1280 なので 1280x720 になる)
    display.setColorDepth(16);

    // 各キャンバスを PSRAM に作成
    canvas_header.setColorDepth(16);
    canvas_header.setPsram(true);
    canvas_header.createSprite(SCREEN_WIDTH, HEADER_H);

    canvas_cards.setColorDepth(16);
    canvas_cards.setPsram(true);
    canvas_cards.createSprite(SCREEN_WIDTH, CARDS_H);

    canvas_spectrum.setColorDepth(16);
    canvas_spectrum.setPsram(true);
    canvas_spectrum.createSprite(SCREEN_WIDTH, SPECTRUM_H);

    canvas_waterfall.setColorDepth(16);
    canvas_waterfall.setPsram(true);
    canvas_waterfall.createSprite(SCREEN_WIDTH, WATERFALL_H);
    canvas_waterfall.fillScreen(COLOR_BG_DARK);

    canvas_footer.setColorDepth(16);
    canvas_footer.setPsram(true);
    canvas_footer.createSprite(SCREEN_WIDTH, FOOTER_H);

    initialized_ = true;
    return true;
#else
    initialized_ = true;
    return true;
#endif
}

void UiRenderer::render_header(const UiSharedState& state) {
#if defined(ESP_PLATFORM)
    canvas_header.fillScreen(0x10A2); // 濃いヘッダー背景
    canvas_header.setTextColor(COLOR_TEXT_WHITE);
    canvas_header.setTextSize(2);

    // タイトル
    canvas_header.drawString("5.8GHz FPV BAND MONITOR", 16, 12);

    // デバイスステータス (右側のゲイン表示と重ならないよう長さを制限)
    std::string status = (state.connected ? "● CONNECTED: " : "○ DISCONNECTED: ")
                         + state.device_info;
    constexpr size_t STATUS_MAX = 60;
    if (status.size() > STATUS_MAX) {
        status.resize(STATUS_MAX);
    }
    canvas_header.setTextSize(1);
    canvas_header.setTextColor(state.connected ? COLOR_GREEN_ON : 0xF800);
    canvas_header.drawString(status.c_str(), 480, 15);

    // 受信モード
    canvas_header.setTextColor(COLOR_TEXT_YELLOW);
    canvas_header.drawRightString(("Rx: " + state.rx_mode).c_str(), GAIN_MINUS_X - 12, 15);

    // ゲイン調整ボタン (タップ可能であることを示すため枠付きで描く)
    const int text_y = GAIN_BTN_Y + 6;
    auto draw_gain_btn = [&](int x, const char* label) {
        canvas_header.fillRoundRect(x, GAIN_BTN_Y, GAIN_BTN_W, GAIN_BTN_H, 6, COLOR_CARD_BG);
        canvas_header.drawRoundRect(x, GAIN_BTN_Y, GAIN_BTN_W, GAIN_BTN_H, 6, COLOR_BORDER_GRAY);
        canvas_header.setTextSize(2);
        canvas_header.setTextColor(COLOR_TEXT_WHITE);
        canvas_header.drawCenterString(label, x + GAIN_BTN_W / 2, text_y);
    };
    draw_gain_btn(GAIN_MINUS_X, "-");
    draw_gain_btn(GAIN_PLUS_X, "+");

    // ゲイン値は PHY ゲインテーブルの index (dB ではない)。-1 は AGC。
    // 未反映の要求があればそちらを表示する (SDR 未接続でも操作結果が見えるように)。
    const int shown_gain = state.request_gain_change ? state.requested_gain : state.current_gain;
    char gain_buf[32];
    char gain_val[8];
    format_gain(shown_gain, gain_val, sizeof(gain_val));
    snprintf(gain_buf, sizeof(gain_buf), "GAIN %s", gain_val);
    canvas_header.setTextSize(2);
    canvas_header.setTextColor(COLOR_TEXT_YELLOW);
    canvas_header.drawRightString(gain_buf, GAIN_VALUE_X + GAIN_VALUE_W, text_y);
#endif
}

void UiRenderer::render_channel_cards(const UiSharedState& state) {
#if defined(ESP_PLATFORM)
    canvas_cards.fillScreen(COLOR_BG_DARK);

    for (size_t i = 0; i < state.active_channels.size() && i < static_cast<size_t>(CARD_COUNT); ++i) {
        const auto& ch = state.active_channels[i];
        const int idx = static_cast<int>(i);
        const bool zoomed = (selected_channel_index_ == idx);
        const int x = card_x(idx);
        int y = 2;
        int h = CARDS_H - 4;

        // 該当チャンネルの測定結果を検索
        const ChannelMeasurement* meas = nullptr;
        for (const auto& m : state.measurements) {
            if (m.name == ch.name) {
                meas = &m;
                break;
            }
        }

        bool present = (meas != nullptr && meas->present);

        // カード外枠と背景 (選択中はシアンで強調)
        canvas_cards.fillRoundRect(x, y, CARD_W, h, 6, COLOR_CARD_BG);
        uint16_t border_color = zoomed ? COLOR_CYAN_LINE : (present ? COLOR_GREEN_ON : COLOR_BORDER_GRAY);
        canvas_cards.drawRoundRect(x, y, CARD_W, h, 6, border_color);
        if (zoomed || present) {
            canvas_cards.drawRoundRect(x + 1, y + 1, CARD_W - 2, h - 2, 5, border_color);
        }
        if (zoomed) {
            canvas_cards.setTextSize(1);
            canvas_cards.setTextColor(COLOR_CYAN_LINE);
            canvas_cards.drawRightString("ZOOM x", x + CARD_W - 14, y + 100);
        }

        // チャンネル名 & 公称周波数
        canvas_cards.setTextSize(2);
        canvas_cards.setTextColor(present ? COLOR_TEXT_WHITE : COLOR_TEXT_GRAY);
        canvas_cards.drawString(ch.name.c_str(), x + 14, y + 12);

        canvas_cards.setTextSize(1);
        std::string nom_str = std::to_string(static_cast<int>(ch.freq_mhz)) + " MHz";
        canvas_cards.drawString(nom_str.c_str(), x + 60, y + 16);

        if (present) {
            // 実測中心周波数
            std::ostringstream oss_f;
            oss_f << std::fixed << std::setprecision(2) << meas->centre_mhz << " MHz";
            canvas_cards.setTextSize(2);
            canvas_cards.setTextColor(COLOR_GREEN_ON);
            canvas_cards.drawString(oss_f.str().c_str(), x + 14, y + 42);

            // オフセット & SNR
            std::ostringstream oss_sub;
            oss_sub << (meas->offset_mhz >= 0 ? "+" : "") << std::fixed << std::setprecision(2)
                    << meas->offset_mhz << " MHz  |  SNR: " << std::setprecision(1) << meas->snr_db << " dB";
            canvas_cards.setTextSize(1);
            canvas_cards.setTextColor(COLOR_TEXT_YELLOW);
            canvas_cards.drawString(oss_sub.str().c_str(), x + 14, y + 74);

            // 帯域幅
            if (!std::isnan(meas->bw_db3_mhz)) {
                std::ostringstream oss_bw;
                oss_bw << "-3dB: " << std::fixed << std::setprecision(1) << meas->bw_db3_mhz << " MHz";
                canvas_cards.drawString(oss_bw.str().c_str(), x + 14, y + 94);
            }
        } else {
            canvas_cards.setTextSize(2);
            canvas_cards.setTextColor(COLOR_TEXT_GRAY);
            canvas_cards.drawString("NO SIGNAL", x + 14, y + 50);

            canvas_cards.setTextSize(1);
            if (meas != nullptr && !meas->reason.empty()) {
                canvas_cards.drawString(meas->reason.c_str(), x + 14, y + 84);
            } else {
                canvas_cards.drawString("Scanning band...", x + 14, y + 84);
            }
        }
    }
#endif
}

void UiRenderer::render_spectrum(const UiSharedState& state) {
#if defined(ESP_PLATFORM)
    canvas_spectrum.fillScreen(COLOR_BG_DARK);

    constexpr float MIN_DB = -95.0f;
    constexpr float MAX_DB = -25.0f;

    float start_f = 0.0f;
    float stop_f = 0.0f;
    current_span(state, start_f, stop_f);
    const float span_f = stop_f - start_f;

    auto freq_to_x = [&](float f) -> int {
        return static_cast<int>((f - start_f) / span_f * SCREEN_WIDTH);
    };
    auto db_to_y = [&](float db) -> int {
        float norm = (MAX_DB - db) / (MAX_DB - MIN_DB);
        return std::clamp(static_cast<int>(norm * SPECTRUM_H), 0, SPECTRUM_H - 1);
    };

    // グリッド線: 表示幅に応じた間隔 (ズームすると自動的に細かくなる)
    const float grid_step = nice_grid_step(span_f);
    const int first_grid = static_cast<int>(std::ceil(start_f / grid_step));
    const int last_grid = static_cast<int>(std::floor(stop_f / grid_step));
    for (int k = first_grid; k <= last_grid; ++k) {
        const float f = static_cast<float>(k) * grid_step;
        const int gx = freq_to_x(f);
        canvas_spectrum.drawFastVLine(gx, 0, SPECTRUM_H, COLOR_GRID_LINE);
        canvas_spectrum.setTextSize(1);
        canvas_spectrum.setTextColor(COLOR_TEXT_GRAY);
        char label[16];
        if (grid_step >= 10.0f) {
            snprintf(label, sizeof(label), "%dM", static_cast<int>(std::lround(f)));
        } else {
            snprintf(label, sizeof(label), "%.1fM", static_cast<double>(f));
        }
        canvas_spectrum.drawString(label, gx + 4, SPECTRUM_H - 14);
    }

    // グリッド線: dB (10dB 間隔)
    for (float db = -90.0f; db <= -30.0f; db += 10.0f) {
        int gy = db_to_y(db);
        canvas_spectrum.drawFastHLine(0, gy, SCREEN_WIDTH, COLOR_GRID_LINE);
        canvas_spectrum.setTextSize(1);
        canvas_spectrum.setTextColor(COLOR_TEXT_GRAY);
        canvas_spectrum.drawString((std::to_string(static_cast<int>(db)) + "dB").c_str(), 8, gy - 10);
    }

    // 各チャンネルの 9MHz 帯域幅マーカー枠
    for (const auto& ch : state.active_channels) {
        int x_center = freq_to_x(ch.freq_mhz);
        int x_lo = freq_to_x(ch.freq_mhz - 4.5f);
        int x_hi = freq_to_x(ch.freq_mhz + 4.5f);
        canvas_spectrum.drawFastVLine(x_center, 0, SPECTRUM_H, 0x2965);
        canvas_spectrum.drawRect(x_lo, 10, x_hi - x_lo, SPECTRUM_H - 20, 0x18E3);
        canvas_spectrum.setTextColor(COLOR_TEXT_GRAY);
        canvas_spectrum.drawString(ch.name.c_str(), x_center - 8, 14);
    }

    // スペクトラム折れ線グラフ
    const auto& powers = state.latest_frame.powers_db;
    if (!powers.empty()) {
        float bin_mhz = state.latest_frame.bin_hz / 1e6f;
        float frame_start = state.latest_frame.start_mhz;

        int prev_x = -1;
        int prev_y = -1;

        for (size_t i = 0; i < powers.size(); ++i) {
            float p = powers[i];
            if (std::isnan(p)) continue;

            float f = frame_start + static_cast<float>(i) * bin_mhz;
            int x = freq_to_x(f);
            int y = db_to_y(p);

            if (prev_x >= 0 && x != prev_x) {
                canvas_spectrum.drawLine(prev_x, prev_y, x, y, COLOR_CYAN_LINE);
            }
            prev_x = x;
            prev_y = y;
        }
    }

    // 検出信号の実測中心周波数マーカー
    for (const auto& m : state.measurements) {
        if (m.present && !std::isnan(m.centre_mhz)) {
            int mx = freq_to_x(m.centre_mhz);
            canvas_spectrum.drawFastVLine(mx, 0, SPECTRUM_H, COLOR_GREEN_ON);
            // 上部に逆三角マーカー
            canvas_spectrum.fillTriangle(mx - 6, 0, mx + 6, 0, mx, 10, COLOR_GREEN_ON);
        }
    }
#endif
}

void UiRenderer::render_waterfall(const UiSharedState& state) {
#if defined(ESP_PLATFORM)
    // ズーム範囲が変わった直後は、周波数対応が異なる過去の履歴が残っていると
    // 誤解を招くので一度クリアする
    if (waterfall_dirty_) {
        canvas_waterfall.fillScreen(COLOR_BG_DARK);
        waterfall_dirty_ = false;
        return;
    }

    // 1行下へスクロール
    canvas_waterfall.scroll(0, 1);

    const auto& powers = state.latest_frame.powers_db;
    if (powers.empty()) return;

    float start_f = 0.0f;
    float stop_f = 0.0f;
    current_span(state, start_f, stop_f);
    const float span_f = stop_f - start_f;

    float bin_mhz = state.latest_frame.bin_hz / 1e6f;
    float frame_start = state.latest_frame.start_mhz;

    // 最上行 (y=0) に新しい掃引データをプロット
    for (int x = 0; x < SCREEN_WIDTH; ++x) {
        float f = start_f + (static_cast<float>(x) / SCREEN_WIDTH) * span_f;
        int bin_idx = static_cast<int>((f - frame_start) / bin_mhz);

        float p = -100.0f;
        if (bin_idx >= 0 && bin_idx < static_cast<int>(powers.size())) {
            p = powers[bin_idx];
        }

        uint16_t color = db_to_rgb565(p);
        canvas_waterfall.drawPixel(x, 0, color);
    }
#endif
}

void UiRenderer::render_footer(const UiSharedState& state) {
#if defined(ESP_PLATFORM)
    canvas_footer.fillScreen(0x10A2);
    canvas_footer.setTextSize(1);

    float start_f = 0.0f;
    float stop_f = 0.0f;
    current_span(state, start_f, stop_f);

    char span_buf[64];
    const size_t sel = static_cast<size_t>(selected_channel_index_);
    if (selected_channel_index_ >= 0 && sel < state.active_channels.size()) {
        snprintf(span_buf, sizeof(span_buf), "SPAN: %s %.0f-%.0f MHz",
                 state.active_channels[sel].name.c_str(),
                 static_cast<double>(start_f), static_cast<double>(stop_f));
    } else {
        snprintf(span_buf, sizeof(span_buf), "SPAN: ALL %.0f-%.0f MHz",
                 static_cast<double>(start_f), static_cast<double>(stop_f));
    }

    canvas_footer.setTextColor(COLOR_TEXT_GRAY);
    canvas_footer.drawString("TOUCH: Card = Zoom / again = Reset    GAIN -/+ = PHY index (-1 = AGC)", 16, 8);
    canvas_footer.setTextColor(COLOR_TEXT_YELLOW);
    canvas_footer.drawRightString(span_buf, SCREEN_WIDTH - 16, 8);
#endif
}

void UiRenderer::render(const UiSharedState& state) {
    if (!initialized_) return;

#if defined(ESP_PLATFORM)
    render_header(state);
    render_channel_cards(state);
    render_spectrum(state);
    render_waterfall(state);
    render_footer(state);

    // 画面へ一括転送 (フリッカーなし)
    display.startWrite();
    canvas_header.pushSprite(0, HEADER_Y);
    canvas_cards.pushSprite(0, CARDS_Y);
    canvas_spectrum.pushSprite(0, SPECTRUM_Y);
    canvas_waterfall.pushSprite(0, WATERFALL_Y);
    canvas_footer.pushSprite(0, FOOTER_Y);
    display.endWrite();
#endif
}

bool UiRenderer::poll_tap(int& x, int& y) {
#if defined(ESP_PLATFORM)
    if (!initialized_ || display.touch() == nullptr) {
        return false;
    }
    // getTouch は「今押されている点数」を返すので、そのまま while で回すと
    // 指を離すまで描画が止まってしまう。押下エッジだけを 1 回返す。
    const bool down = (display.getTouch(&x, &y) != 0);
    const bool pressed = down && !touch_was_down_;
    touch_was_down_ = down;
    return pressed;
#else
    (void)x;
    (void)y;
    return false;
#endif
}

void UiRenderer::current_span(const UiSharedState& state, float& start_mhz, float& stop_mhz) const {
    start_mhz = BAND_START_MHZ;
    stop_mhz = BAND_STOP_MHZ;
    const size_t sel = static_cast<size_t>(selected_channel_index_);
    if (selected_channel_index_ >= 0 && sel < state.active_channels.size()) {
        const float centre = state.active_channels[sel].freq_mhz;
        start_mhz = centre - ZOOM_HALF_SPAN_MHZ;
        stop_mhz = centre + ZOOM_HALF_SPAN_MHZ;
    }
}

void UiRenderer::change_gain(int delta, UiSharedState& state) {
    // 未反映の要求が残っている場合はそちらを基準にする (連打で取りこぼさない)
    const int base = state.request_gain_change ? state.requested_gain : state.current_gain;
    const int next = std::clamp(base + delta, GAIN_MIN, GAIN_MAX);
    if (next == base) {
        return;
    }
    state.requested_gain = next;
    state.request_gain_change = true;
#if defined(ESP_PLATFORM)
    ESP_LOGI(TAG, "gain index request: %d -> %d", base, next);
#endif
}

void UiRenderer::handle_touch(int x, int y, UiSharedState& state) {
#if defined(ESP_PLATFORM)
    ESP_LOGI(TAG, "tap at (%d, %d)", x, y);
#endif

    // ヘッダ右端のゲインボタン
    if (in_rect(x, y, GAIN_MINUS_X, GAIN_BTN_Y, GAIN_BTN_W, GAIN_BTN_H)) {
        change_gain(-GAIN_STEP, state);
        return;
    }
    if (in_rect(x, y, GAIN_PLUS_X, GAIN_BTN_Y, GAIN_BTN_W, GAIN_BTN_H)) {
        change_gain(+GAIN_STEP, state);
        return;
    }

    // チャンネルカード (カードは canvas 内の y=2 に描画し、canvas は CARDS_Y に配置)
    if (y >= CARDS_Y && y < CARDS_Y + CARDS_H) {
        for (size_t i = 0; i < state.active_channels.size() && i < static_cast<size_t>(CARD_COUNT); ++i) {
            if (in_rect(x, y, card_x(static_cast<int>(i)), CARDS_Y + 2, CARD_W, CARDS_H - 4)) {
                const int idx = static_cast<int>(i);
                selected_channel_index_ = (selected_channel_index_ == idx) ? -1 : idx;
                waterfall_dirty_ = true; // 周波数対応が変わるので履歴を破棄
#if defined(ESP_PLATFORM)
                float s = 0.0f;
                float e = 0.0f;
                current_span(state, s, e);
                ESP_LOGI(TAG, "zoom -> %s (%.1f-%.1f MHz)",
                         selected_channel_index_ < 0 ? "ALL" : state.active_channels[i].name.c_str(),
                         static_cast<double>(s), static_cast<double>(e));
#endif
                return;
            }
        }
    }
}

} // namespace vtxmon
