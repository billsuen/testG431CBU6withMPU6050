/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    service.c
  * @brief   此文件處理顯存繪圖、MPU6050 讀取（含濾波）與按鍵控制邏輯
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "service.h"
#include "../OLED_128x64/OLED128x64_Fast.h"
#include "i2c.h"
#include "kalman.h"
#include <math.h>

/* Private variables ---------------------------------------------------------*/
osSemaphoreId_t binSemButtonHandle;

/* 姿態與校正變數 (volatile 確保任務間同步) */
static volatile float angle_x = 0.0f; 
static volatile float angle_y = 0.0f;
static volatile uint8_t calibrate_flag = 0;
static float gyro_bias_x = 0.0f;
static float gyro_bias_y = 0.0f;
static float offset_angle_x = 0.0f; 
static float offset_angle_y = 0.0f;

/* Kalman 濾波器實例 */
static Kalman_t kalmanX;
static Kalman_t kalmanY;

/* 顯存：128x64 像素 = 1024 Byte */
static uint8_t OLED_Buffer[1024];

#define MPU6050_ADDR         0xD0
#define MPU6050_PWR_MGMT_1   0x6B
#define MPU6050_CONFIG       0x1A
#define MPU6050_ACCEL_XOUT_H 0x3B

/* 常數定義 */
#define RAD_TO_DEG     57.2957795f
#define FILTER_ALPHA   0.6125f  // 互補濾波係數

/* --- 顯存繪圖函式 --- */

void OLED_Clear_Buffer(void) {
  for(int i=0; i<1024; i++) OLED_Buffer[i] = 0;
}

void OLED_DrawPixel(int x, int y, uint8_t color) {
  if (x < 0 || x > 127 || y < 0 || y > 63) return;
  if (color) OLED_Buffer[x + (y / 8) * 128] |= (1 << (y % 8));
  else       OLED_Buffer[x + (y / 8) * 128] &= ~(1 << (y % 8));
}

void OLED_DrawBubble(int x, int y) {
  // 實心 8x8 圓形圖案
  static const uint8_t bubble_bmp[] = {0x3C, 0x7E, 0xFF, 0xFF, 0xFF, 0xFF, 0x7E, 0x3C};
  for(int i=0; i<8; i++) {
    for (int j=0; j<8; j++) {
      if(bubble_bmp[i] & (1 << j)) OLED_DrawPixel(x + i - 4, y + j - 4, 1);
    }
  }
}

void OLED_Refresh(void) {
  for (uint8_t i = 0; i < 8; i++) {
    OLED_Set_Pos(0, i);
    HAL_I2C_Mem_Write(&hi2c3, OLED_ADDRESS, 0x40, 1, &OLED_Buffer[128 * i], 128, 100);
  }
}

/* --- 服務函式 --- */

void Service_OLED_DisplayInfo(void) {
  OLED_Init(&hi2c3);
  OLED_CLS();
  OLED_Put8x16Str(0, 0, "G431 System Ready");
  
  HAL_Delay(100); // 等待感測器電源穩定
  
  uint8_t check = 0;
  uint8_t data = 0;
  HAL_StatusTypeDef status;

  // 嘗試讀取 WHO_AM_I 暫存器 (預設位址 0xD0)
  status = HAL_I2C_Mem_Read(&hi2c1, MPU6050_ADDR, 0x75, 1, &check, 1, 100);
  
  // 如果 0xD0 失敗，嘗試 0xD2 (AD0 接高電平時的位址)
  if (status != HAL_OK || check != 0x68) {
    status = HAL_I2C_Mem_Read(&hi2c1, 0xD2, 0x75, 1, &check, 1, 100);
    if (status == HAL_OK && check == 0x68) {
      // 如果 0xD2 成功，後續操作也需改用 0xD2
    }
  }

  if (check == 0x68) {
    uint8_t active_addr = (status == HAL_OK) ? 0xD0 : 0xD2;
    // 1. 喚醒
    data = 0x00;
    HAL_I2C_Mem_Write(&hi2c1, active_addr, MPU6050_PWR_MGMT_1, 1, &data, 1, 100);
    // 2. 開啟硬體低通濾波 (DLPF 42Hz)
    data = 0x03;
    HAL_I2C_Mem_Write(&hi2c1, active_addr, MPU6050_CONFIG, 1, &data, 1, 100);
    HAL_Delay(100);
    OLED_Put8x16Str(0, 2, "MPU6050 OK (DLPF)");
  } else {
    OLED_Put8x16Str(0, 2, "MPU6050 Error");
  }
  OLED_Put8x16Str(0, 4, "SW: Calibrate");
}

void MPU6050_BubbleLevel_Task(void) {
  static uint32_t last_tick = 0;
  static uint16_t startup_delay_cnt = 0;
  static float smooth_x = 64.0f; // 顯示用平滑座標
  static float smooth_y = 32.0f;
  
  uint8_t mpu_data[14];
  int16_t raw_ax, raw_ay, raw_az, raw_gx, raw_gy;
  float dt;
  
  if (HAL_I2C_Mem_Read(&hi2c1, MPU6050_ADDR, MPU6050_ACCEL_XOUT_H, 1, mpu_data, 14, 100) == HAL_OK) {
    raw_ax = (int16_t)(mpu_data[0] << 8 | mpu_data[1]);
    raw_ay = (int16_t)(mpu_data[2] << 8 | mpu_data[3]);
    raw_az = (int16_t)(mpu_data[4] << 8 | mpu_data[5]);
    raw_gx = (int16_t)(mpu_data[8] << 8 | mpu_data[9]);
    raw_gy = (int16_t)(mpu_data[10] << 8 | mpu_data[11]);

    uint32_t current_tick = osKernelGetTickCount();
    dt = (float)(current_tick - last_tick) / 1000.0f;
    if (dt <= 0 || dt > 0.5f) dt = 0.01f;
    last_tick = current_tick;

    float accel_angle_x = atan2f((float)raw_ay, (float)raw_az) * RAD_TO_DEG;
    float accel_angle_y = atan2f(-(float)raw_ax, sqrtf((float)raw_ay * raw_ay + (float)raw_az * raw_az)) * RAD_TO_DEG;
    float gyro_rate_x = (float)(raw_gx - gyro_bias_x) / 131.0f;
    float gyro_rate_y = (float)(raw_gy - gyro_bias_y) / 131.0f;

    if (startup_delay_cnt < 20) {
      startup_delay_cnt++;
      if (startup_delay_cnt == 20) calibrate_flag = 1;
    }

    if (calibrate_flag) {
      angle_x = accel_angle_x;
      angle_y = accel_angle_y;
      gyro_bias_x = (float)raw_gx;
      gyro_bias_y = (float)raw_gy;
      offset_angle_x = angle_x; 
      offset_angle_y = angle_y;
      
      // 同步更新 Kalman 狀態
      kalmanX.angle = angle_x;
      kalmanY.angle = angle_y;
      
      calibrate_flag = 0;
    }

    /* 使用 Kalman 濾波 */
    angle_x = Kalman_Update(&kalmanX, accel_angle_x, gyro_rate_x, dt);
    angle_y = Kalman_Update(&kalmanY, accel_angle_y, gyro_rate_y, dt);

    /* --- 原有的互補濾波 (註解供比較) ---
    angle_x = FILTER_ALPHA * (angle_x + gyro_rate_x * dt) + (1.0f - FILTER_ALPHA) * accel_angle_x;
    angle_y = FILTER_ALPHA * (angle_y + gyro_rate_y * dt) + (1.0f - FILTER_ALPHA) * accel_angle_y;
    ----------------------------------- */

    /* 計算目標像素位置 (扣除偏移量以實現歸零) */
    float target_x = 64.0f + ((angle_x - offset_angle_x) * 2.0f); 
    float target_y = 32.0f - ((angle_y - offset_angle_y) * 2.0f);

    /* 二次平滑濾波 (Lerp) - 消除靜止時的抖動 */
    smooth_x = smooth_x * 0.5f + target_x * 0.5f;
    smooth_y = smooth_y * 0.5f + target_y * 0.5f;

    /* 繪圖流程 */
    OLED_Clear_Buffer(); 
    for(int i=0; i<128; i++) OLED_DrawPixel(i, 32, 1); 
    for(int i=0; i<64; i++)  OLED_DrawPixel(64, i, 1); 
    OLED_DrawBubble((int)smooth_x, (int)smooth_y);
    OLED_Refresh();
  }
  osDelay(10);
}

void Service_Init(void) {
  /* 在此處建立訊號量，確保在任務執行前已就緒 */
  if (binSemButtonHandle == NULL) {
    binSemButtonHandle = osSemaphoreNew(1, 0, NULL);
  }
  /* 初始化 Kalman 參數 (Q_angle, Q_bias, R_measure) */
  Kalman_Init(&kalmanX, 0.005f, 0.003f, 0.01f);
  Kalman_Init(&kalmanY, 0.005f, 0.003f, 0.01f);
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
  if (GPIO_Pin == SW_Pin) {
    if (binSemButtonHandle != NULL) osSemaphoreRelease(binSemButtonHandle);
  }
}

void Button_Process_Task(void) {
  if (osSemaphoreAcquire(binSemButtonHandle, osWaitForever) == osOK) {
    osDelay(10);
    if (HAL_GPIO_ReadPin(SW_GPIO_Port, SW_Pin) == GPIO_PIN_SET) {
      while (HAL_GPIO_ReadPin(SW_GPIO_Port, SW_Pin) == GPIO_PIN_SET) osDelay(10);
      osDelay(10);
      calibrate_flag = 1;
      HAL_GPIO_TogglePin(IND_GPIO_Port, IND_Pin);
      while (osSemaphoreAcquire(binSemButtonHandle, 0) == osOK);
    }
  }
}
