# ArduinoでDC  
Seeed XIAOで使えるプロトを作る。

## packages/arduino追加パッチ

packages-arduino.tag.gzは設定ファイル群のスナップショットです。これを丸ごと~/.arduino15/にコピーするか、以下のパッチを抜き出すかします。

1. TARGET  
- ...hardware/mbed_nano/4.0.10/cores/arduino/mbed/targets/TARGET_NORDIC/TARGET_NRF5x/TARGET_NRF52/TARGET_MCU_NRF52840/
   - TARGET_SEEED_XIAO_NRF52840
   - TARGET_SEEED_XIAO_NRF52840_SENSE

2. board.txt  
- ...hardware/mbed_nano/4.2.1/  
SeeedXiaoをtargetに追加。追加されるSeeedは、littleFS(FS_Nano33)に適合するためNano33のbootloadedを前提にしています。新しいSeeedには、予めnano33のbootloaderを書き込んでください。

[Seeed XiaoをDAPLinkにする](Seeed_DAP.md)

3. variants  
- ...hardware/mbed_nano/4.2.1/  
    - SEEED_XIAO_NRF52840
      - メモリ割付、bootloaderをNano33をエミュレート
    - SEEED_XIAO_NRF52840_SENSE
      - FastBoot改変版libmbed.aを内包
      - スタートアドレス0 bootloaderなし

## Libraries

### 同梱
- SetTimeout  
メインスレッド(低優先度)で動く簡易スケジューラ

### 要ダウンロード
- ArduinoBLE
- FS_Nano33BLE
- NRF52_MBED_TimerInterrupt-1.4.0

## Source
- dcuino2


