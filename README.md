# testG431CBU6withMPU6050

這是一個以 `STM32G431CBU6` 為 MCU 的電子氣泡水平儀專案。系統透過 `MPU6050` / `MPU6500` 讀取三軸加速度計與三軸陀螺儀資料，使用一維 Kalman Filter 融合 X/Y 傾角，並將結果顯示在 `SSD1306` 128x64 OLED 上。

> 注意：實際感測器回應 `WHO_AM_I = 0x70`，因此它更可能是 `MPU6500` 或其變體，而非原廠 `MPU6050`。

專案特色：

- `STM32CubeMX` 產生硬體與週邊初始化
- `FreeRTOS CMSIS-V2` 管理任務
- `I2C1` 連接 `MPU6050 / MPU6500`
- `I2C3` 連接 `SSD1306` 128x64 OLED
- `PC13` 作為校正按鍵 `SW`
- `PC6` 作為狀態指示燈 `IND`
- **🔹 OLED 高效動畫控制**：雙緩衝區 + 區域刷新技術，實現流暢氣泡動畫同時節省 I2C 頻寬

## 功能摘要

- OLED 開機顯示系統與感測器狀態
- 自動偵測 `MPU6050 / MPU6500`
- 支援 I2C 地址 `0xD0` 與 `0xD2`
- 喚醒感測器並設定 100Hz 採樣率與 DLPF
- 透過 MPU 中斷驅動 DMA 讀取資料
- 使用 Kalman Filter 融合 X/Y 角度（調整參數提升靈敏度）
- 支援按鍵校正水平零點
- **🔹 OLED 高效動畫顯示**：使用雙緩衝區技術，實現區域刷新與背景恢復，極大減少 I2C 通信量，提升動畫流暢度
- 將角度映射到 OLED 氣泡座標，無額外平滑以反映真實動態

## 系統流程

1. `main()` 初始化 HAL、時鐘、GPIO、I2C、USART，以及 FreeRTOS。
2. `MX_FREERTOS_Init()` 呼叫 `Service_Init()`，建立按鍵與 MPU 中斷 semaphore，並初始化 `kalmanX`、`kalmanY`。
3. 建立三個任務：
   - `defaultTask`：執行 `Button_Process_Task()`，處理按鍵消抖與校正請求
   - `mpuReadTask`：執行 `MPU6050_Read_Task()`，等待 MPU 中斷並讀取感測器資料
   - `oledTask`：執行 `OLED_Display_Task()`，**🔹 使用高效動畫技術更新 OLED 顯示**
4. `MPU6050_Sensor_Init()` 嘗試讀取 `WHO_AM_I`，若成功則重置、喚醒感測器，並設定 100Hz 採樣率與 DLPF。
5. `MPU6050_Read_Task()` 等待 MPU 中斷，使用 DMA 讀取 14 字節感測器資料，計算時間差 `dt`，並以 Kalman Filter 融合角度。
6. `OLED_Display_Task()` **🔹 初始化雙緩衝區，繪製靜態十字背景並備份；動態更新時僅刷新變化的區域，實現高效氣泡動畫**。

## 主要檔案

- `Core/Src/main.c`
  系統進入點與週邊初始化
- `Core/Src/app_freertos.c`
  FreeRTOS 任務建立與排程
- `Core/Src/service.c`
  主邏輯：OLED 顯示、MPU6050/MPU6500 初始化與讀取、Kalman 濾波、按鍵校正
- `Core/Src/kalman.c`
  一維 Kalman Filter 實作
- `Core/Inc/service.h`
  服務函式宣告
- `Core/Inc/kalman.h`
  Kalman 結構與 API
- `Core/OLED_128x64/OLED128x64_Fast.*`
  **🔹 更新**：OLED 高效驅動，支援 I2C Timeout 自動跳脫機制，確保系統穩定性
- `testG431CBU6withMPU6050.ioc`
  STM32CubeMX 專案設定
- `CMakeLists.txt`
  CMake 建置入口

## 主要功能說明

### `Service_Init(void)`

位置：`Core/Src/service.c`

用途：

- 建立按鍵 semaphore `binSemButtonHandle`
- 建立 MPU 中斷 semaphore `binSemMpuIntHandle`
- 建立 I2C1 DMA 完成 semaphore `binSemI2c1DoneHandle`
- 初始化 Kalman Filter 物件 `kalmanX`、`kalmanY`

預設參數：

```c
Kalman_Init(&kalmanX, 0.005f, 0.003f, 0.005f);
Kalman_Init(&kalmanY, 0.005f, 0.003f, 0.005f);
```

### `MPU6050_Sensor_Init(void)`

位置：`Core/Src/service.c`

用途：

- 透過 `I2C1` 讀取 `MPU6050` / `MPU6500` 的 `WHO_AM_I`
- 若回應 `0x70`，表示感測器更可能為 `MPU6500` 或其變體
- 支援 `0xD0` 和 `0xD2` 兩個 I2C 地址
- 若偵測到感測器，則重置並喚醒裝置
- 設定 100Hz 採樣率與 DLPF
- 初始化中斷設定

### `MPU6050_Read_Task(void)`

位置：`Core/Src/service.c`

用途：

- 等待 `binSemMpuIntHandle`
- 讀取 `MPU6050_INT_STATUS`
- 使用 DMA 讀取 14 字節加速度與陀螺儀資料，並等待 `binSemI2c1DoneHandle`
- 計算時間差 `dt`
- 將原始加速度資料轉換成 `accel_angle_x`、`accel_angle_y`
- 扣除陀螺儀偏移並轉成角速度
- 執行按鍵校正與零點更新
- 呼叫 `Kalman_Update()` 計算濾波後角度
- 若 DMA 失敗，提供後備阻塞讀取機制

### `OLED_Display_Task(void)`

位置：`Core/Src/service.c`

用途：

- **🔹 高效 OLED 動畫顯示任務**
- 初始化雙緩衝區：`OLED_Buffer` (動態繪圖) 與 `OLED_Background` (靜態背景)
- 繪製十字基準線並備份到背景緩衝區
- 動態計算氣泡位置，限制邊界防止出界
- 使用區域刷新技術：僅更新舊位置與新位置的像素區域，極大減少 I2C 通信
- 實現流暢的氣泡動畫，每 30ms 更新一次

### `OLED_DrawPixel(int x, int y, uint8_t color)`

位置：`Core/Src/service.c`

用途：

- 在 OLED_Buffer 中繪製單個像素
- 支持黑色 (0) 和白色 (1) 顏色

### `OLED_DrawBubble(int x, int y, uint8_t color)`

位置：`Core/Src/service.c`

用途：

- 在指定位置繪製 8x8 氣泡圖形
- 使用 OLED_DrawPixel 實現圓形氣泡

### `OLED_Refresh_Region(int x, int y, int w, int h)`

位置：`Core/Src/service.c`

用途：

- **🔹 區域刷新核心函式**
- 僅傳送指定矩形區域的緩衝區資料到 OLED
- 極大節省 I2C 頻寬，提升動畫性能

### `OLED_Restore_BG(int x, int y, int w, int h)`

位置：`Core/Src/service.c`

用途：

- 從 OLED_Background 恢復指定區域到 OLED_Buffer
- 用於清除舊的動態元素，準備繪製新位置

### `Button_Process_Task(void)`

位置：`Core/Src/service.c`

用途：

- 等待按鍵中斷 semaphore
- 執行簡單消抖
- 等待按鍵放開
- 設定 `calibrate_flag = 1`
- 觸發下一次讀取時進行校正

## Kalman Filter API

### 結構

```c
typedef struct {
    float Q_angle;
    float Q_bias;
    float R_measure;
    float angle;
    float bias;
    float P[2][2];
} Kalman_t;
```

### `Kalman_Init(Kalman_t *Kalman, float Q_angle, float Q_bias, float R_measure)`

用途：

- 初始化 Kalman 參數
- 將 `angle`、`bias` 與誤差協方差矩陣清零

### `Kalman_Update(Kalman_t *Kalman, float newAngle, float newRate, float dt)`

用途：

- 結合加速度計角度 `newAngle`
- 結合陀螺儀角速度 `newRate`
- 使用時間差 `dt` 計算濾波後角度
- 回傳融合後角度

## 角度映射與平滑

在 `OLED_Display_Task()` 中，傾角會映射成 OLED 氣泡座標：

```c
float target_x = 64.0f + ((local_angle_x - local_offset_x) * 2.0f);
float target_y = 32.0f - 為了反映真實動態，移除額外平滑，直接使用 `target_x` 和 `target_y`th_y * 0.5f + target_y * 0.5f;
```

降低顯示抖動，使氣泡運動更穩定。

### `Kalman_Init(Kalman_t *Kalman, float Q_angle, float Q_bias, float R_measure)`

用途：

- 設定 Kalman 參數
- 將角度、偏移、協方差矩陣清零

典型用法：

```c
Kalman_t kalmanX;
Kalman_Init(&kalmanX, 0.005f, 0.003f, 0.01f);
```

注意事項：

- `Q_angle`、`Q_bias`、`R_measure` 應為正值
- 不建議小於 `1e-6`
- 初始化後若已知目前角度，可手動指定 `kalmanX.angle = current_angle`

### `Kalman_Update(Kalman_t *Kalman, float newAngle, float newRate, float dt)`

用途：

- 根據加速度計量測角 `newAngle`
- 根據陀螺儀角速度 `newRate`
- 結合時間差 `dt`
- 回傳濾波後角度

典型用法：

```c
float filtered = Kalman_Update(&kalmanX, accel_angle_x, gyro_rate_x, dt);
```

參數意義：

- `newAngle`
  由加速度計計算出的角度，適合提供長期參考
- `newRate`
  由陀螺儀得到的角速度，適合短期動態追蹤
- `dt`
  兩次更新的時間差，單位秒

## 角度與畫面映射

在 `MPU6050_BubbleLevel_Task()` 中，傾角會映射成畫面座標：

```c
float target_x = 64.0f + ((angle_x - offset_angle_x) * 2.0f);
float target_y = 32.0f - ((angle_y - offset_angle_y) * 2.0f);
```

畫面中心 `(64, 32)` 代表校正後的水平位置。`offset_angle_x` 與 `offset_angle_y` 用來保留校正零點。

接著再做一次平滑：

```c
smooth_x = smooth_x * 0.5f + target_x * 0.5f;
smooth_y = smooth_y * 0.5f + target_y * 0.5f;
```

這會讓畫面比較穩，但也會增加一點延遲。

## Kalman 參數調整指南

以下內容整合自 `Kalman_Filter_設定方法.txt`，可作為實機調參依據。

### 三個核心參數

- `Q_angle`
  角度過程雜訊。值越大，濾波器越願意接受快速變化，反應更快，但雜訊也會增加。
- `Q_bias`
  陀螺儀偏移雜訊。值越大，偏移修正速度越快。
- `R_measure`
  測量雜訊。值越小，代表越信任加速度計，對慢速傾斜會更敏感，但也更容易受震動影響。

### 建議範圍

| 參數 | 建議範圍 | 典型值 |
| --- | --- | --- |
| `Q_angle` | `0.0001 ~ 0.01` | `0.001` |
| `Q_bias` | `0.001 ~ 0.01` | `0.003` |
| `R_measure` | `0.01 ~ 0.5` | `0.03` |

### 目前專案偏向的設定

```c
Kalman_Init(&kalmanX, 0.005f, 0.003f, 0.01f);
Kalman_Init(&kalmanY, 0.005f, 0.003f, 0.01f);
```

這組設定相對偏靈敏，適合讓氣泡對慢速傾斜也有明顯反應。

### 常見現象與調整方向

- 如果「翻轉很靈敏，但水平慢慢移動幾乎沒反應」
  優先降低 `R_measure`，並適度提高 `Q_angle`
- 如果「靜止時氣泡抖動明顯」
  提高 `R_measure`，或降低 `Q_angle`
- 如果「看起來有拖地感」
  除了 Kalman 參數，也要檢查畫面平滑係數 `smooth = smooth * a + target * (1-a)`

### 實際調整建議

較穩定的起點：

```c
Kalman_Init(&kalmanX, 0.001f, 0.003f, 0.03f);
Kalman_Init(&kalmanY, 0.001f, 0.003f, 0.03f);
```

較靈敏的起點：

```c
Kalman_Init(&kalmanX, 0.005f, 0.003f, 0.01f);
Kalman_Init(&kalmanY, 0.005f, 0.003f, 0.01f);
```

若要進一步提高畫面直接性，可把顯示平滑改為：

```c
smooth_x = smooth_x * 0.5f + target_x * 0.5f;
smooth_y = smooth_y * 0.5f + target_y * 0.5f;
```

或測試時先直接關閉：

```c
smooth_x = target_x;
smooth_y = target_y;
```

## FPU 對 Kalman Filter 的影響

以下內容整合自 `FPU對KalmanFilter的影響討論.txt`。

### 結論先說

- 對目前的 `STM32G431` 而言，Kalman Filter 非常輕鬆，因為它是 `Cortex-M4F`，具備硬體 FPU。
- 若 MCU 沒有 FPU，Kalman Filter 仍然可用，但浮點乘除會明顯增加 CPU 負擔。
- 對 `100Hz` 這類低頻應用，沒有 FPU 通常也還能接受。
- 對 `1kHz` 以上更新率，FPU、DMA 與非阻塞式通訊就變得很重要。

### 為什麼 Kalman 比互補濾波重

互補濾波大致只需要少量乘加：

```c
angle = 0.98f * (angle + gyro * dt) + 0.02f * accel;
```

Kalman Filter 則包含：

- 狀態預測
- 協方差矩陣更新
- 增益計算
- 一次浮點除法

在沒有 FPU 的 MCU 上，浮點除法特別昂貴。

### 沒有 FPU 時的實際影響

- 單次 `Kalman_Update()` 執行時間會大幅增加
- CPU 使用率上升
- 中斷延遲風險增加
- 功耗提高

但對本專案目前約 `100Hz` 的更新週期來說，通常不至於造成系統無法運作。

### 96MHz、32-bit、無 FPU、1kHz 是否可行

整理後的判斷是：可以，而且重點瓶頸通常不是 Kalman 本身，而是感測器通訊。

原因：

- 1D Kalman 的浮點運算量雖然不小，但在 `96MHz` 的 32-bit MCU 上，兩軸估算通常仍在可接受範圍
- 若每次都用阻塞式 `I2C` 讀取 `MPU6050` 的 14 bytes，總線等待時間常比濾波計算更傷即時性

因此若系統往高更新率發展，建議：

- 使用 `I2C + DMA`
- 或改用更高速的 `SPI`
- 必要時改成定點數運算

## 任務與即時性注意事項

- `bubbleLevelTask` 目前使用 `osDelay(10)`，實際更新率約 `100Hz`
- 感測器讀取使用阻塞式 `HAL_I2C_Mem_Read()`
- OLED 更新也使用阻塞式 I2C 傳輸
- 若未來要提升到 `500Hz` 或 `1kHz`，建議優先重新設計通訊方式，而不是只調 Kalman 參數

## 建置方式

本專案可由 CubeMX/CubeIDE 維護，也可透過 CMake 建置。

### CMake

專案根目錄已提供 `CMakeLists.txt` 與 `CMakePresets.json`。

常見流程如下：

```bash
cmake -S . -B build
cmake --build build
```

交叉編譯器設定位於：

- `cmake/gcc-arm-none-eabi.cmake`
- `cmake/stm32cubemx/CMakeLists.txt`

## 硬體設定摘要

根據 `testG431CBU6withMPU6050.ioc`：

- MCU：`STM32G431CBU6`
- 系統時脈：`48 MHz`
- `I2C1`：連接 `MPU6050`
- `I2C3`：連接 `OLED`
- `USART2`：保留作為序列埠
- `FreeRTOS`：CMSIS-V2
- `configENABLE_FPU = 1`
- `configTOTAL_HEAP_SIZE = 10240`

## 後續可優化方向

- 將 `MPU6050` 位址選擇邏輯改成初始化後保存，不要每次固定讀 `0xD0`
- 把感測器讀取改成 `DMA` 或中斷驅動
- 為 Kalman 參數與畫面平滑係數加入集中式設定區
- 增加 UART 診斷輸出，方便觀察角度與原始感測值

