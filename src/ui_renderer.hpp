#pragma once

#include <cstdint>
#include <string>
#include <vector>
#include "types.hpp"
#include "ui_state.hpp"

namespace vtxmon {

class UiRenderer {
public:
    UiRenderer();
    ~UiRenderer();

    // 画面初期化 (1280x720 MIPI-DSI)
    bool init();

    // UI フレーム全体の描画
    void render(const UiSharedState& state);

    // 「離されていた指が押された瞬間」を 1 件だけ取り出す (エッジトリガ)。
    // 指を押し続けても true は 1 回しか返さないので、長押しで描画が止まらない。
    bool poll_tap(int& x, int& y);

    // タップ座標の処理。ゲイン変更は state 経由で SDR タスクに要求するため、
    // 呼び出し側は state.mtx を保持しておくこと。
    void handle_touch(int x, int y, UiSharedState& state);

private:
    bool initialized_{false};
    bool touch_was_down_{false};
    bool waterfall_dirty_{false}; // ズーム変更後にウォーターフォールを一度クリアする
    uint32_t last_waterfall_seq_{0}; // 新しいスイープが来た時だけウォーターフォールを進める
    int selected_channel_index_{-1}; // -1 = 4CH 全表示 (ALL), 0..3 = 単一チャンネル拡大

    // 画面レイアウト定数
    static constexpr int SCREEN_WIDTH = 1280;
    static constexpr int SCREEN_HEIGHT = 720;

    static constexpr int HEADER_Y = 0;
    static constexpr int HEADER_H = 40;

    static constexpr int CARDS_Y = 42;
    static constexpr int CARDS_H = 118;

    static constexpr int SPECTRUM_Y = 162;
    static constexpr int SPECTRUM_H = 260;

    static constexpr int WATERFALL_Y = 424;
    static constexpr int WATERFALL_H = 266;

    static constexpr int FOOTER_Y = 692;
    static constexpr int FOOTER_H = 28;

    // チャンネルカードの形状 (描画とタップ判定で共有)
    static constexpr int CARD_COUNT = 4;
    static constexpr int CARD_W = 306;
    static constexpr int CARD_MARGIN = 10;
    static constexpr int CARD_START_X = 12;

    // ヘッダ右端のゲインボタン
    static constexpr int GAIN_BTN_W = 84;
    static constexpr int GAIN_BTN_H = 28;
    static constexpr int GAIN_BTN_Y = (HEADER_H - GAIN_BTN_H) / 2;
    static constexpr int GAIN_VALUE_W = 96;
    static constexpr int GAIN_PLUS_X = SCREEN_WIDTH - 16 - GAIN_BTN_W;
    static constexpr int GAIN_VALUE_X = GAIN_PLUS_X - 12 - GAIN_VALUE_W;
    static constexpr int GAIN_MINUS_X = GAIN_VALUE_X - 12 - GAIN_BTN_W;

    // スペクトラム表示の既定帯域 (app.cpp の掃引範囲と一致させる)
    static constexpr float BAND_START_MHZ = 5640.0f;
    static constexpr float BAND_STOP_MHZ = 5800.0f;
    // カードをタップしたときのズーム半幅。FPV チャンネル幅 9 MHz より少し広く取る。
    // bin 幅 78.125 kHz なので 20 MHz スパンで約 256 bin = 5 px/bin になる。
    static constexpr float ZOOM_HALF_SPAN_MHZ = 10.0f;

    // ゲインは PHY ゲインテーブルの index で dB ではない。-1 = AGC (GAIN HARDWARE)。
    // 範囲とステップは PC 版 fpv_band_monitor/gui.py の spin_gain に合わせる。
    static constexpr int GAIN_MIN = -1;
    static constexpr int GAIN_MAX = 127;
    static constexpr int GAIN_STEP = 4;

    static int card_x(int index) { return CARD_START_X + index * (CARD_W + CARD_MARGIN); }

    // 現在表示している周波数範囲 (ズーム中は選択チャンネル中心の ±ZOOM_HALF_SPAN_MHZ)
    void current_span(const UiSharedState& state, float& start_mhz, float& stop_mhz) const;

    void change_gain(int delta, UiSharedState& state);

    // dB -> RGB565 カラーマップ変換
    static uint16_t db_to_rgb565(float db, float min_db = -95.0f, float max_db = -30.0f);

    void render_header(const UiSharedState& state);
    void render_channel_cards(const UiSharedState& state);
    void render_spectrum(const UiSharedState& state);
    void render_waterfall(const UiSharedState& state);
    void render_footer(const UiSharedState& state);
};

} // namespace vtxmon
