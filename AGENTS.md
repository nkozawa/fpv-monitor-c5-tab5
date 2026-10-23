# FPVドローンのVTX周波数をM5Stack TAB 5でモニターしたい

## ハードウェア
- StampC5/XIAO ESP32-C5:
  1. **Widscan58 ファームウェア** (推奨): https://github.com/CapnBry/widscan58
     - ESP32-C5 側で高速ホッピング（~4ms/hop）と FFT・メトリクス計算を実行し、約 50〜60 rotations/s の極めて高速な応答を実現
  2. **公式 ESP-SDR ファームウェア**: https://espargos.net/espsdr/
     - 生 I/Q 転送モードに対応。TAB 5 側で FFT・スティッチング・メトリクス解析を実行
  ※ TAB 5 側で接続時に自動判別（Widscan58 / 公式 ESP-SDR / FPV-LCD）するため、どちらのファームウェアでも動作可能。


- M5Stack TAB 5 の画面で5GHz VTXの電波をモニターする
  https://github.com/zubon2003/fpv-band-monitor-c5
  これはPC向けのセットアップ、これと同等の画面をTAB 5で得たい

## 開発環境
VSCode+platformIO

## 検討事項
- TAB 5とStampC5の接続方法
- TAB 5とSTampC5の役割分担

## 実現可能性スタディ
FEASIBILITY.mdに記入済み
