# ESLAB-Final-Project

### 組員：

1. 張迪善 B11902116
2. 李松桓 B11902098
3. 謝明倫 B11901034

## 題目

BLE Position Recognition

## 動機

1. Unfamiliarity with BLE
2. Attempt Spatial Calculation
3. Data transmission and noise processing
4. Effective Visual Presentation

## 預定做法

硬體需求：一台 STM32，三台 Raspberry Pi，一台電腦（個人筆電）

軟體\系統需求：Fully Functional Wifi、BLE interface

定義（Definition）：

- Raspberry Pi：絕對坐標位置（Absolute Coordinates）、Bluetooth Client
- STM32：移動裝置（Mobile Device）、Bluetooth Server
- 電腦：結果呈現（Visual Presentation）

程式實作框架（Code framework）：

- Raspberry Pi：使用 mac address 鏈接 STM32，計算並傳送距離資訊
- STM32：Broadcast as Bluetooth Server，鏈接三台 Raspberry Pi, 接收 + 處理距離以計算自身位置，通過 Wifi 傳給電腦
- 電腦：通過 Wifi 接收來自 STM32 的坐標資訊，進行視覺呈現

執行步驟：

1. 設置三台 RPi 以 BLE 鏈接 STM32
2. RPi 與 STM32 通過 signal propagation speed and time 計算並同步三維距離資訊
3. STM32 儲存、處理三台 RPi 的距離，計算出範圍內相對坐標
4. STM32 將位置坐標通過 Wifi 傳送給電腦
5. 電腦將坐標資訊繪圖呈現

## 預期結果

1. 擺設三台 RPi 作為絕對坐標點 + 初始化 STM32 的位置
2. 隨機移動 STM32 後可以重新計算出準確位置
3. 將坐標訊息圖像化呈現

## 難點

1. Raspberry Pi 身為絕對坐標位置必須放置得相距特定距離（空間限制）
2. STM32 sample frequency defines a tradeoff between dynamic update ↔ accuracy
3. BLE signal noise processing
