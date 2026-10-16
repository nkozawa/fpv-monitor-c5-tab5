#pragma once

#include <cstdint>
#include <memory>
#include <string>

namespace vtxmon {

// StampC5 (ESP-SDR) と USB CDC-ACM 経由で通信するクライアント。
// Arduino では EspUsbHost、ESP-IDF では usb_host_cdc_acm をバックエンドに使う。
// ヘッダはバックエンド非依存に保つため pimpl にしている。
class UsbHostSdrClient {
public:
    UsbHostSdrClient();
    ~UsbHostSdrClient();

    UsbHostSdrClient(const UsbHostSdrClient&) = delete;
    UsbHostSdrClient& operator=(const UsbHostSdrClient&) = delete;

    // USB Host サブシステムの初期化
    bool init();

    // StampC5 の接続を待機 (タイムアウト ms)
    // タイムアウトした場合は次回呼び出しで別の OTG ポートを試す (ESP32-P4 は HS/FS の 2 系統)
    bool wait_for_device(uint32_t timeout_ms = 5000);

    bool is_connected() const;

    // コマンド送信
    bool send_command(const std::string& cmd);

    // 1行のテキスト応答を受信 (改行文字まで)
    bool read_line(std::string& line_out, uint32_t timeout_ms = 1000);

    // バイナリペイロードの受信
    bool read_bytes(uint8_t* buffer, size_t length, uint32_t timeout_ms = 1000);

    // 受信リングに溜まっている分だけ読む (ブロックしない)。戻り値は読み出したバイト数。
    size_t read_available(uint8_t* buffer, size_t max_length);

    // バッファのクリア
    void flush_input();

    // 同期処理: SYNC <nonce> を投げ、"SYNC <nonce>\n" が受信ストリームに
    // 現れるまで待つ。読みかけのバイナリペイロードを跨いで復旧できる。
    bool synchronize(uint32_t timeout_ms = 500);

    void close();

    // 接続中デバイスの説明 ("303A:1001 USB JTAG/serial debug unit" など)
    const std::string& device_info() const { return device_info_; }

    // リングバッファに取りこぼしが生じた回数 (デバッグ用)
    uint32_t overrun_count() const;

private:
    struct Impl;

    bool start_host();

    std::unique_ptr<Impl> impl_;
    bool connected_{false};
    std::string device_info_;
};

} // namespace vtxmon
