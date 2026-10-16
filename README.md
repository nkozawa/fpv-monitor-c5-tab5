# M5Stack TAB 5 + StampC5 による FPV 5.8GHz バンドモニター

**M5Stack TAB 5** (ESP32-P4) の 5インチ 720p（1280×720）IPS ディスプレイで、FPV ドローン用 5.8GHz 帯の電波（FM-ATV）をリアルタイムに監視するバンドモニターシステムです。
PC 向け実装（[fpv-band-monitor-c5](https://github.com/zubon2003/fpv-band-monitor-c5)）と同等の機能を TAB 5 単体で実現します。

---

## 主な特徴

- **4波同時監視**: 既定で **E2 (5665 MHz) / E1 (5705 MHz) / F3 (5740 MHz) / F5 (5780 MHz)** を監視
- **リアルタイム UI (1280×720)**:
  - **4CH ステータスカード**: 実測中心周波数（電力加重重心法）、公称周波数からのオフセット、SNR、-3dB 占有帯域幅、在/無判定
  - **スペクトラム折れ線グラフ**: 9MHz / 17MHz 帯域枠、ノイズフロア基準線、重心周波数マーカー
  - **ウォーターフォール**: 時間経過に伴うカラーヒートマップ（青 → 緑 → 黄 → 赤）
- **タッチ操作対応**:
  - チャンネルカードをタップして「4CH 全体表示 ⇔ 1CH 拡大ズーム (20MHzスパン)」をワンタッチ切替
  - ヘッダー右端の `+` / `-` ボタンで SDR アナログゲインを即座に増減
- **ケーブル 1 本で直結**:
  - TAB 5 の **USB-A Host ポート** から StampC5 (ESP32-C5) に 5V 給電し、USB CDC-ACM 仮想シリアルで高速通信
- **公式 ESP-SDR ファームウェア対応**:
  - StampC5 は公式ファームウェアのまま生 I/Q データを転送し、8192点 FFT・スティッチング・信号解析・UI描画をすべて TAB 5 (ESP32-P4 @ 400MHz) 側でリアルタイム実行
  - FPV-LCD カスタムファームウェア（SWEEP ストリーム転送）が接続された場合でも自動判別して動作可能

---

## ハードウェア接続と役割分担

### 接続構成

```
+------------------------------------+           USB-A to Type-C ケーブル           +---------------------------+
|          M5Stack TAB 5             | -------------------------------------------> |          StampC5          |
|  (ESP32-P4 USB Host Controller)    | <==========================================> |   (ESP32-C5 Native USB)   |
|                                    |       [5V 給電] + [CDC-ACM 仮想シリアル]      |  ESP-SDR 公式ファームウェア  |
+------------------------------------+                                              +---------------------------+
```

1. M5Stack TAB 5 の **USB-A ポート** と StampC5 の **Type-C ポート** を市販の USB ケーブルで接続します。
2. StampC5 への電源は TAB 5 から供給されます（TAB 5 単体または NP-F550 バッテリー運用でも動作可能）。
3. TAB 5 の書き込み・デバッグログ確認は、TAB 5 の **USB-C ポート** を PC に接続して行います（USB-A と USB-C は同時利用可能）。

### 役割分担

| デバイス | 担当機能 | 備考 |
| :--- | :--- | :--- |
| **StampC5 (ESP32-C5)** | 5GHz RF チューニング、アナログゲイン設定、生 I/Q サンプリング（80MS/s, 8192点）、USB 送信 | 公式 ESP-SDR ファームウェア利用。改造不要 |
| **M5Stack TAB 5 (ESP32-P4)** | USB CDC-ACM ホスト制御、ホッピング計画、リアルタイム FFT (8192点)、Min スティッチング、電力加重重心法・SNR解析、1280x720 画面描画、タッチ操作 | 400MHz RISC-V デュアルコア + MIPI-DSI 液晶 + 32MB PSRAM |

---

## プロジェクト構成

```
.
├── platformio.ini             # PlatformIO 設定ファイル (ESP32-P4, M5GFX, EspUsbHost)
├── AGENTS.md                  # プロジェクト要件・指示書
├── FEASIBILITY.md             # ハードウェア適合性・実現可能性検討書
├── README.md                  # 本ドキュメント
├── src/                       # ファームウェアソースコード
│   ├── main.cpp               # エントリポイント (setup / loop, タスク生成, タッチポーリング)
│   ├── types.hpp              # 共通データ構造 (ComplexSample, SweepFrame, ChannelConfig 等)
│   ├── esp_sdr_protocol.hpp/cpp   # コマンド生成、I/Q アンパック、CRC32
│   ├── dsp_stitcher.hpp/cpp   # LOホッピング計画、FFT、Minスティッチング
│   ├── channel_analysis.hpp/cpp   # 電力加重重心法による中心周波数、SNR、帯域幅算出
│   ├── usb_host_sdr.hpp/cpp   # USB Host CDC-ACM 通信ドライバ (EspUsbHost + 64KB SpscRing)
│   ├── app.hpp/cpp            # SDR 受信・解析ワーカー (FreeRTOS Core 0 タスク)
│   ├── ui_state.hpp           # スレッドセーフな共有ステート管理
│   └── ui_renderer.hpp/cpp    # M5GFX による 1280x720 画面描画 & タッチハンドラ
└── core/                      # Mac/PC 上でのコア DSP/プロトコル単体テスト (CMake)
    ├── CMakeLists.txt         #   ../src を直接参照
    └── test/test_all.cpp
```

---

## 開発環境とビルド手順

### 1. VSCode + PlatformIO によるビルド & 書き込み

#### 必要な環境
- **VSCode** + **PlatformIO IDE** 拡張機能
- Platform: `pioarduino/platform-espressif32` (ESP-IDF v5.x / Arduino ESP32 v3.x, ESP32-P4 対応)

#### ビルド
PlatformIO CLI または VSCode の PlatformIO サイドバーから実行できます:

```bash
# ビルド
pio run

# 書き込み (TAB 5 の USB-C ポートを PC に接続)
pio run --target upload

# シリアルモニター確認 (115200 bps)
pio run --target monitor
```

### 2. Mac 上でのコアロジック単体テスト (CMake)

実機を使わずにアルゴリズム（I/Q アンパック、FFT、スティッチング、重心法解析）を Mac 上で即座に検証できます:

```bash
cd core
cmake -B build
cmake --build build
./build/test_vtxmon_core
```

---

## 画面の見方と操作

- **上部ヘッダー**: 接続ポート、現在のゲイン設定、表示スパンモード（ALL / 単一CH）、受信モード（Raw I/Q）、右端の `[-]` `[+]` タッチボタン。
- **4CH カード領域**: 各チャンネル（E2 / E1 / F3 / F5）の公称周波数、実測中心周波数（オフセット）、SNR、帯域幅を表示。カードをタップするとそのチャンネルを中心としたズーム表示（20MHzスパン）に切り替わります（再度タップで ALL に復帰）。
- **スペクトラム領域**: 信号レベル（dBFS）のリアルタイム折れ線グラフ。チャンネル枠（9MHz / 17MHz）と検出信号の中心線を表示。
- **ウォーターフォール領域**: 時間経過に伴うスペクトラムの履歴をヒートマップ表示。
