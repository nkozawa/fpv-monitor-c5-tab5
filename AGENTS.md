# FPVドローンのVTX周波数をM5Stack TAB 5でモニターしたい

/Users/kozawa/src/vtxmontabc5 に作りかけたArduino IDE用のコードがあるので参考にする

## ハードウェア
- StampC5/XIAO ESP32-C5 https://espargos.net/espsdr/ に従いファームウェア更新済み、ESP32-C5 でFFTを行うファームウェアは使用しない
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