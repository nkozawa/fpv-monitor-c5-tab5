#include "usb_host_sdr.hpp"

#include <atomic>
#include <cstring>

#include "esp_sdr_protocol.hpp"

#if defined(ESP_PLATFORM)
#include <Arduino.h>
#include <EspUsbHost.h>
#include "esp_heap_caps.h"
#include "esp_log.h"
#include "esp_rom_sys.h"
#endif

namespace vtxmon {

#if defined(ESP_PLATFORM)

namespace {

constexpr const char* TAG = "UsbHostSdr";

// PC 版 fpv_band_monitor/esp_sdr.py と同じ値。ESP32-C5 の Native USB では
// ボーレートは無視されるが、UART ブリッジ経由の場合に備えて合わせておく。
constexpr uint32_t SDR_BAUD = 2000000;

// CAP16 は 1 ホップあたり 16384 バイトを一気に送ってくる。
// EspUsbHostCdcSerial の既定リング (512 B) では取りこぼすため、
// onSerialData を直接受けて自前の大容量リングに memcpy する。
constexpr size_t RX_RING_BYTES = 128 * 1024;

} // namespace

// SpscRing は Impl のメンバ型になるため、匿名名前空間に置くと内部リンクの型を
// 外部リンクの型が持つことになり -Wsubobject-linkage が出る。名前付きにする。
namespace usb_detail {

// 単一生産者 (USB クライアントタスク) / 単一消費者 (SDR ワーカータスク) の
// ロックフリーリング。head/tail を atomic にしてクリティカルセクションを避ける。
class SpscRing {
public:
    bool is_ready() const { return buf_ != nullptr; }

    bool allocate(size_t bytes) {
        free();
        // 低遅延な内部 SRAM (DIRAM) を最優先で確保し、取れなければ PSRAM にフォールバック
        buf_ = static_cast<uint8_t*>(
            heap_caps_malloc(bytes, MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT));
        if (buf_ == nullptr) {
            buf_ = static_cast<uint8_t*>(heap_caps_malloc(bytes, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT));
        }
        if (buf_ == nullptr) {
            buf_ = static_cast<uint8_t*>(heap_caps_malloc(bytes, MALLOC_CAP_8BIT));
        }
        if (buf_ == nullptr) {
            return false;
        }
        capacity_ = bytes;
        head_.store(0, std::memory_order_relaxed);
        tail_.store(0, std::memory_order_relaxed);
        return true;
    }

    void free() {
        if (buf_ != nullptr) {
            heap_caps_free(buf_);
            buf_ = nullptr;
        }
        capacity_ = 0;
        head_.store(0, std::memory_order_relaxed);
        tail_.store(0, std::memory_order_relaxed);
    }

    void clear() {
        // 消費者側からの破棄。生産者は head しか動かさないので、
        // head を読んで tail を合わせるだけで競合しない。
        tail_.store(head_.load(std::memory_order_acquire), std::memory_order_release);
        overruns_.store(0, std::memory_order_relaxed);
    }

    // 生産者側 (USB コールバック)
    void write(const uint8_t* data, size_t length) {
        if (buf_ == nullptr || length == 0) {
            return;
        }
        const size_t head = head_.load(std::memory_order_relaxed);
        const size_t tail = tail_.load(std::memory_order_acquire);
        if (head - tail + length > capacity_) {
            overruns_.fetch_add(1, std::memory_order_relaxed);
            return; // 入りきらないチャンクは丸ごと捨てる (部分書き込みは行わない)
        }
        const size_t offset = head % capacity_;
        const size_t first = capacity_ - offset;
        if (first >= length) {
            std::memcpy(buf_ + offset, data, length);
        } else {
            std::memcpy(buf_ + offset, data, first);
            std::memcpy(buf_, data + first, length - first);
        }
        head_.store(head + length, std::memory_order_release);
    }

    // 消費者側 (ワーカーループ)
    size_t read(uint8_t* dst, size_t length) {
        if (buf_ == nullptr) {
            return 0;
        }
        const size_t head = head_.load(std::memory_order_acquire);
        const size_t tail = tail_.load(std::memory_order_relaxed);
        size_t avail = head - tail;
        if (avail == 0) {
            return 0;
        }
        if (length < avail) {
            avail = length;
        }
        const size_t offset = tail % capacity_;
        const size_t first = capacity_ - offset;
        if (first >= avail) {
            std::memcpy(dst, buf_ + offset, avail);
        } else {
            std::memcpy(dst, buf_ + offset, first);
            std::memcpy(dst + first, buf_, avail - first);
        }
        tail_.store(tail + avail, std::memory_order_release);
        return avail;
    }

    int peek_byte() const {
        if (buf_ == nullptr) {
            return -1;
        }
        const size_t head = head_.load(std::memory_order_acquire);
        const size_t tail = tail_.load(std::memory_order_relaxed);
        if (head == tail) {
            return -1;
        }
        return buf_[tail % capacity_];
    }

    void drop_byte() {
        tail_.store(tail_.load(std::memory_order_relaxed) + 1, std::memory_order_release);
    }

    uint32_t overruns() const { return overruns_.load(std::memory_order_relaxed); }

private:
    uint8_t* buf_{nullptr};
    size_t capacity_{0};
    std::atomic<size_t> head_{0};
    std::atomic<size_t> tail_{0};
    std::atomic<uint32_t> overruns_{0};
};

} // namespace usb_detail

struct UsbHostSdrClient::Impl {
    EspUsbHost usb;
    usb_detail::SpscRing ring;

    volatile bool device_ready{false};
    volatile bool device_lost{false};
    uint8_t address{ESP_USB_HOST_ANY_ADDRESS};
    uint8_t port{ESP_USB_HOST_ANY_PORT};
    std::string description;

    // ESP32-P4 は HS / FS の 2 系統の OTG を持つ。Tab5 の USB-A がどちらに
    // 配線されているかは断定できないので、見つからなければ切り替えて再試行する。
    int port_index{0};
    bool host_started{false};

    EspUsbHostPort current_port() const {
        return port_index == 0 ? ESP_USB_HOST_PORT_DEFAULT : ESP_USB_HOST_PORT_FULL_SPEED;
    }
};

UsbHostSdrClient::UsbHostSdrClient() : impl_(std::make_unique<Impl>()) {}

UsbHostSdrClient::~UsbHostSdrClient() {
    close();
    if (impl_ && impl_->host_started) {
        impl_->usb.end();
        impl_->host_started = false;
    }
}

bool UsbHostSdrClient::start_host() {
    Impl& impl = *impl_;

    if (impl.host_started) {
        impl.usb.end();
        impl.host_started = false;
    }

    impl.device_ready = false;
    impl.device_lost = false;
    impl.address = ESP_USB_HOST_ANY_ADDRESS;
    impl.port = ESP_USB_HOST_ANY_PORT;
    impl.description.clear();

    // ポート切替のたびに 64KB を確保し直さないよう、既にあるものは再利用する
    if (!impl.ring.is_ready() && !impl.ring.allocate(RX_RING_BYTES)) {
        ESP_LOGE(TAG, "RX ring allocation failed (%u bytes)", (unsigned)RX_RING_BYTES);
    }
    impl.ring.clear();

    Impl* self = &impl;

    impl.usb.onDeviceConnected([self](const EspUsbHostDeviceInfo& dev) {
        if (self->device_ready) {
            return; // 既にバインド済み (2 つ目以降のデバイスは無視)
        }
        if (self->usb.serialPortCount(dev.address) == 0) {
            ESP_LOGW(TAG, "addr %u has no CDC-ACM port, skipping (%04X:%04X)",
                     dev.address, dev.vid, dev.pid);
            return;
        }
        self->address = dev.address;
        self->port = 0;
        self->usb.setSerialBaudRate(SDR_BAUD, dev.address, 0);

        char buf[96];
        snprintf(buf, sizeof(buf), "%04X:%04X %s", dev.vid, dev.pid,
                 (dev.product != nullptr && dev.product[0] != '\0') ? dev.product : "CDC-ACM");
        self->description = buf;
        self->ring.clear();
        self->device_lost = false;
        self->device_ready = true;
        ESP_LOGI(TAG, "device ready: %s (speed=%d)", self->description.c_str(), (int)dev.speed);
    });

    impl.usb.onDeviceDisconnected([self](const EspUsbHostDeviceInfo& dev) {
        if (dev.address != self->address) {
            return;
        }
        ESP_LOGW(TAG, "device addr %u disconnected", dev.address);
        self->device_ready = false;
        self->device_lost = true;
        self->address = ESP_USB_HOST_ANY_ADDRESS;
        self->port = ESP_USB_HOST_ANY_PORT;
    });

    impl.usb.onSerialData([self](const EspUsbHostSerialData& data) {
        if (!self->device_ready || data.address != self->address) {
            return;
        }
        self->ring.write(data.data, data.length);
    });

    EspUsbHostConfig config;
    config.taskStackSize = 8192;
    config.taskPriority = 8;
    config.taskCore = 0; // SDR ワーカーと同じ core 0。UI (Arduino loopTask) は core 1
    config.port = impl.current_port();

    if (!impl.usb.begin(config)) {
        ESP_LOGE(TAG, "usb.begin(port=%d) failed: %s",
                 (int)config.port, impl.usb.lastErrorName());
        return false;
    }
    impl.host_started = true;
    ESP_LOGI(TAG, "USB Host started on port %d", (int)config.port);
    return true;
}

bool UsbHostSdrClient::init() {
    ESP_LOGI(TAG, "Initializing USB Host (EspUsbHost)...");
    return start_host();
}

bool UsbHostSdrClient::wait_for_device(uint32_t timeout_ms) {
    if (impl_ == nullptr || !impl_->host_started) {
        if (!start_host()) {
            return false;
        }
    }

    const uint32_t deadline = millis() + timeout_ms;
    while (static_cast<int32_t>(deadline - millis()) > 0) {
        if (impl_->device_ready && impl_->usb.serialReady(impl_->address, impl_->port)) {
            connected_ = true;
            device_info_ = impl_->description;
            return true;
        }
        delay(20);
    }

    // タイムアウト: ESP32-P4 は HS / FS の 2 系統の OTG を持つので、
    // 次はもう片方のポートで待ち受ける。
    const EspUsbHostPort failed_port = impl_->current_port();
    impl_->port_index = (impl_->port_index + 1) % 2;
    ESP_LOGW(TAG, "no device on port %d, switching to port %d",
             (int)failed_port, (int)impl_->current_port());
    connected_ = false;
    start_host();
    return false;
}

bool UsbHostSdrClient::is_connected() const {
    return connected_ && impl_ != nullptr && !impl_->device_lost;
}

bool UsbHostSdrClient::send_command(const std::string& cmd) {
    if (!is_connected()) {
        return false;
    }
    return impl_->usb.sendSerial(reinterpret_cast<const uint8_t*>(cmd.data()), cmd.size(),
                                 impl_->address, impl_->port);
}

bool UsbHostSdrClient::read_line(std::string& line_out, uint32_t timeout_ms) {
    line_out.clear();
    if (!is_connected()) {
        return false;
    }
    const uint32_t deadline = millis() + timeout_ms;
    int spin_count = 0;
    while (static_cast<int32_t>(deadline - millis()) > 0) {
        const int c = impl_->ring.peek_byte();
        if (c < 0) {
            if (impl_->device_lost) {
                connected_ = false;
                return false;
            }
            if (++spin_count < 40) {
                esp_rom_delay_us(25);
            } else {
                spin_count = 0;
                taskYIELD();
            }
            continue;
        }
        spin_count = 0;
        impl_->ring.drop_byte();
        if (c == '\n') {
            return true;
        }
        if (c != '\r') {
            line_out.push_back(static_cast<char>(c));
        }
    }
    return !line_out.empty();
}

bool UsbHostSdrClient::read_bytes(uint8_t* buffer, size_t length, uint32_t timeout_ms) {
    if (!is_connected() || buffer == nullptr || length == 0) {
        return false;
    }
    const uint32_t deadline = millis() + timeout_ms;
    size_t got = 0;
    int spin_count = 0;
    while (got < length) {
        const size_t n = impl_->ring.read(buffer + got, length - got);
        if (n > 0) {
            got += n;
            spin_count = 0;
            if (got >= length) {
                return true;
            }
            continue;
        }
        if (static_cast<int32_t>(deadline - millis()) <= 0) {
            return false;
        }
        if (impl_->device_lost) {
            connected_ = false;
            return false;
        }
        if (++spin_count < 40) {
            esp_rom_delay_us(25);
        } else {
            spin_count = 0;
            taskYIELD();
        }
    }
    return true;
}

size_t UsbHostSdrClient::read_available(uint8_t* buffer, size_t max_length) {
    if (!is_connected() || buffer == nullptr || max_length == 0) {
        return 0;
    }
    return impl_->ring.read(buffer, max_length);
}

void UsbHostSdrClient::flush_input() {
    if (impl_ != nullptr) {
        impl_->ring.clear();
    }
}

uint32_t UsbHostSdrClient::overrun_count() const {
    return (impl_ != nullptr) ? impl_->ring.overruns() : 0;
}

bool UsbHostSdrClient::synchronize(uint32_t timeout_ms) {
    if (!is_connected()) {
        return false;
    }

    // PC 版 esp_sdr.py の synchronize() 相当。
    // CAP16 のペイロード (バイナリ) を読みかけの状態で 1 行一致を待つと、
    // 0x0A で途中で切れたゴミ行しか得られず復旧できない。直近の受信バイトを
    // 保持したまま "SYNC <nonce>\n" の部分一致を探す。
    // 1回の試行を短く (50ms) 刻み、全体で素早く復旧する。
    static uint32_t nonce_counter = 0;
    const uint32_t attempt_timeout = std::min<uint32_t>(60, std::max<uint32_t>(20, timeout_ms / 6));

    for (int attempt = 0; attempt < 6; ++attempt) {
        const uint64_t nonce = 1000000ULL + (++nonce_counter);
        const std::string marker = "SYNC " + std::to_string(nonce) + "\n";

        flush_input();
        if (!send_command(EspSdrProtocol::cmd_sync(nonce))) {
            continue;
        }

        std::string tail;
        uint8_t chunk[256];
        const uint32_t deadline = millis() + attempt_timeout;
        while (static_cast<int32_t>(deadline - millis()) > 0) {
            const size_t n = read_available(chunk, sizeof(chunk));
            if (n == 0) {
                if (impl_->device_lost) {
                    connected_ = false;
                    return false;
                }
                esp_rom_delay_us(100);
                continue;
            }
            tail.append(reinterpret_cast<const char*>(chunk), n);
            if (tail.size() > 512) {
                tail.erase(0, tail.size() - 512);
            }
            if (tail.find(marker) != std::string::npos) {
                return true;
            }
        }
    }
    return false;
}


void UsbHostSdrClient::close() {
    connected_ = false;
    if (impl_ != nullptr) {
        impl_->device_ready = false;
        impl_->address = ESP_USB_HOST_ANY_ADDRESS;
        impl_->port = ESP_USB_HOST_ANY_PORT;
        impl_->ring.clear();
    }
}

#else // !ESP_PLATFORM: ホスト側テスト用のスタブ

struct UsbHostSdrClient::Impl {};

UsbHostSdrClient::UsbHostSdrClient() = default;
UsbHostSdrClient::~UsbHostSdrClient() = default;
bool UsbHostSdrClient::start_host() { return true; }
bool UsbHostSdrClient::init() { return true; }
bool UsbHostSdrClient::wait_for_device(uint32_t) { connected_ = true; return true; }
bool UsbHostSdrClient::is_connected() const { return connected_; }
bool UsbHostSdrClient::send_command(const std::string&) { return connected_; }
bool UsbHostSdrClient::read_line(std::string& line_out, uint32_t) {
    line_out = "OK";
    return connected_;
}
bool UsbHostSdrClient::read_bytes(uint8_t*, size_t, uint32_t) { return connected_; }
size_t UsbHostSdrClient::read_available(uint8_t*, size_t) { return 0; }
void UsbHostSdrClient::flush_input() {}
uint32_t UsbHostSdrClient::overrun_count() const { return 0; }
bool UsbHostSdrClient::synchronize(uint32_t) { return connected_; }
void UsbHostSdrClient::close() { connected_ = false; }

#endif // ESP_PLATFORM

} // namespace vtxmon
