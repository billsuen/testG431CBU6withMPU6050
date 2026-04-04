/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    service.c
  * @brief   此文件處理顯存繪圖、MPU6050/6500 讀取（含濾波）與按鍵控制邏輯
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "service.h"
#include "../OLED_128x64/OLED128x64_Fast.h"
#include "i2c.h"
#include "kalman.h"
#include "stm32g4xx_hal_i2c.h"
#include <math.h>

/* Private variables ---------------------------------------------------------*/
osSemaphoreId_t binSemButtonHandle;
osSemaphoreId_t binSemMpuIntHandle;
osSemaphoreId_t binSemI2c1DoneHandle;

/* 姿態與校正變數 */
static volatile float angle_x = 0.0f; 
static volatile float angle_y = 0.0f;
static volatile uint8_t calibrate_flag = 0;
static float gyro_bias_x = 0.0f;
static float gyro_bias_y = 0.0f;
static volatile float offset_angle_x = 0.0f; 
static volatile float offset_angle_y = 0.0f;

static uint8_t mpu_active_addr = 0xD0;
static volatile uint8_t mpu_init_status = 0; 
static float smooth_x = 64.0f;
static float smooth_y = 32.0f;

static Kalman_t kalmanX;
static Kalman_t kalmanY;

static uint8_t OLED_Buffer[1024];

#define MPU6050_ADDR         0xD0
#define MPU6050_PWR_MGMT_1   0x6B
#define MPU6050_CONFIG       0x1A
#define MPU6050_ACCEL_XOUT_H 0x3B
#define MPU6050_INT_ENABLE   0x38
#define MPU6050_INT_PIN_CFG  0x37
#define MPU6050_INT_STATUS   0x3A
#define MPU6050_WHO_AM_I     0x75

#define RAD_TO_DEG     57.2957795f

/* --- 顯存繪圖函式 (含局部刷新優化) --- */

static uint8_t OLED_DirtyPages = 0x00; // 用位元表示 0-7 頁是否需要更新

/**
 * @brief 清空顯存緩衝區
 */
void OLED_Clear_Buffer(void) {
  for(int i=0; i<1024; i++) OLED_Buffer[i] = 0;
  OLED_DirtyPages = 0xFF; // 清除後標記所有頁面需要刷新
}

/**
 * @brief 繪製像素並追蹤變動區域 (Dirty Page Tracking)
 */
void OLED_DrawPixel(int x, int y, uint8_t color) {
  if (x < 0 || x > 127 || y < 0 || y > 63) return;
  
  uint8_t page = y / 8;
  uint8_t bit = 1 << (y % 8);
  uint16_t idx = x + page * 128;
  
  if (color) {
    if (!(OLED_Buffer[idx] & bit)) { // 只有在值改變時才標記髒頁
      OLED_Buffer[idx] |= bit;
      OLED_DirtyPages |= (1 << page);
    }
  } else {
    if (OLED_Buffer[idx] & bit) {
      OLED_Buffer[idx] &= ~bit;
      OLED_DirtyPages |= (1 << page);
    }
  }
}

/**
 * @brief 繪製或擦除小球
 * @param color 1: 畫白色球, 0: 畫黑色(擦除)
 */
void OLED_DrawBubble(int x, int y, uint8_t color) {
  static const uint8_t bubble_bmp[] = {0x3C, 0x7E, 0xFF, 0xFF, 0xFF, 0xFF, 0x7E, 0x3C};
  for(int i=0; i<8; i++) {
    for (int j=0; j<8; j++) {
      if(bubble_bmp[i] & (1 << j)) {
        OLED_DrawPixel(x + i - 4, y + j - 4, color);
      }
    }
  }
}

/**
 * @brief 修補背景十字線 (防止小球移開後留下空洞)
 * @param x, y 小球之前的座標
 */
void OLED_Repair_Crosshair(int x, int y) {
  // 檢查小球擦除區域 (約 8x8) 是否覆蓋了水平線 (y=32)
  if (32 >= (y - 4) && 32 <= (y + 4)) {
    for (int i = x - 4; i <= x + 4; i++) {
      if (i >= 0 && i < 128) OLED_DrawPixel(i, 32, 1);
    }
  }
  // 檢查小球擦除區域是否覆蓋了垂直線 (x=64)
  if (64 >= (x - 4) && 64 <= (x + 4)) {
    for (int j = y - 4; j <= y + 4; j++) {
      if (j >= 0 && j < 64) OLED_DrawPixel(64, j, 1);
    }
  }
}

/**
 * @brief 執行局部刷新：只將有變動的頁面送往 OLED
 */
void OLED_Refresh(void) {
  if (OLED_DirtyPages == 0) return; // 沒有變動則直接跳過
  
  for (uint8_t i = 0; i < 8; i++) {
    if (OLED_DirtyPages & (1 << i)) {
      OLED_Set_Pos(0, i);
      HAL_I2C_Mem_Write(&hi2c3, OLED_ADDRESS, 0x40, 1, &OLED_Buffer[128 * i], 128, 100);
    }
  }
  OLED_DirtyPages = 0; // 清除髒頁標記
}

/* --- 初始化服務函式 --- */

void MPU6050_Sensor_Init(void) {
  uint8_t check = 0, data = 0, verify = 0;
  HAL_StatusTypeDef status;

  HAL_Delay(500); 

  for(int retry=0; retry<5; retry++) {
    status = HAL_I2C_Mem_Read(&hi2c1, MPU6050_ADDR, MPU6050_WHO_AM_I, 1, &check, 1, 100);
    if (status == HAL_OK && (check == 0x68 || check == 0x70 || check == 0x71)) {
      mpu_active_addr = 0xD0;
      break;
    }
    status = HAL_I2C_Mem_Read(&hi2c1, 0xD2, MPU6050_WHO_AM_I, 1, &check, 1, 100);
    if (status == HAL_OK && (check == 0x68 || check == 0x70 || check == 0x71)) {
      mpu_active_addr = 0xD2;
      break;
    }
    HAL_Delay(100);
  }

  if (status == HAL_OK) {
    data = 0x80;
    HAL_I2C_Mem_Write(&hi2c1, mpu_active_addr, MPU6050_PWR_MGMT_1, 1, &data, 1, 100);
    HAL_Delay(100);
    data = 0x01;
    HAL_I2C_Mem_Write(&hi2c1, mpu_active_addr, MPU6050_PWR_MGMT_1, 1, &data, 1, 100);
    
    // 100Hz 採樣
    data = 9; 
    HAL_I2C_Mem_Write(&hi2c1, mpu_active_addr, 0x19, 1, &data, 1, 100);
    
    data = 0x03; 
    HAL_I2C_Mem_Write(&hi2c1, mpu_active_addr, MPU6050_CONFIG, 1, &data, 1, 100);
    data = 0x20; 
    HAL_I2C_Mem_Write(&hi2c1, mpu_active_addr, MPU6050_INT_PIN_CFG, 1, &data, 1, 100);
    data = 0x01; 
    HAL_I2C_Mem_Write(&hi2c1, mpu_active_addr, MPU6050_INT_ENABLE, 1, &data, 1, 100);

    HAL_I2C_Mem_Read(&hi2c1, mpu_active_addr, MPU6050_INT_STATUS, 1, &verify, 1, 100);
    mpu_init_status = 1; 
  } else {
    mpu_init_status = 2; 
  }
}

void OLED_Display_Init(void) {
  OLED_Init(&hi2c3);
  OLED_CLS();
  OLED_Put8x16Str(0, 0, "G431 Ready");
  uint16_t timeout = 200; 
  while (mpu_init_status == 0 && timeout--) osDelay(10);
  if (mpu_init_status == 1) OLED_Put8x16Str(0, 2, "MPU: 100Hz DMA");
  else OLED_Put8x16Str(0, 2, "MPU: ERROR!");
}

/* --- 任務函式 --- */

void MPU6050_Read_Task(void) {
  uint8_t mpu_data[14];
  uint8_t int_status;
  int16_t raw_ax, raw_ay, raw_az, raw_gx, raw_gy;
  float dt;
  static uint32_t last_tick = 0;
  static uint16_t startup_delay_cnt = 0;

  MPU6050_Sensor_Init();

  for(;;) {
    osSemaphoreAcquire(binSemMpuIntHandle, 50);
    
    // 移除通訊時點亮 IND 的邏輯
    HAL_I2C_Mem_Read(&hi2c1, mpu_active_addr, MPU6050_INT_STATUS, 1, &int_status, 1, 10);
    
   // ✅ 改成單純的阻塞讀取
	if (HAL_I2C_Mem_Read_DMA(&hi2c1, mpu_active_addr, MPU6050_ACCEL_XOUT_H, 1, mpu_data, 14) == HAL_OK) {
	
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
			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6, GPIO_PIN_SET);
			angle_x = accel_angle_x;
			angle_y = accel_angle_y;
			gyro_bias_x = (float)raw_gx;
			gyro_bias_y = (float)raw_gy;
			offset_angle_x = angle_x; 
			offset_angle_y = angle_y;
			kalmanX.angle = angle_x;
			kalmanY.angle = angle_y;
			osDelay(100);
			calibrate_flag = 0;
			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6, GPIO_PIN_RESET);
		}

	angle_x = Kalman_Update(&kalmanX, accel_angle_x, gyro_rate_x, dt);
	angle_y = Kalman_Update(&kalmanY, accel_angle_y, gyro_rate_y, dt);
	}
  }
}
void OLED_Display_Task(void) {
  OLED_Display_Init();
  osDelay(3000);

  // --- 初始繪製背景 ---
  OLED_Clear_Buffer();
  for(int i=0; i<128; i++) OLED_DrawPixel(i, 32, 1); // 畫水平線
  for(int i=0; i<64; i++)  OLED_DrawPixel(64, i, 1);  // 畫垂直線
  OLED_Refresh(); // 首次全屏刷新

  static float last_x = 64.0f;
  static float last_y = 32.0f;
  static uint32_t last_refresh_time = 0;

  for(;;) {
    float local_angle_x = angle_x;
    float local_angle_y = angle_y;
    float local_offset_x = offset_angle_x;
    float local_offset_y = offset_angle_y;

    // 計算小球目標位置
    float target_x = 64.0f + ((local_angle_x - local_offset_x) * 2.0f); 
    float target_y = 32.0f - ((local_angle_y - local_offset_y) * 2.0f);

    // 限制範圍防止小球出界
    if (target_x < 4) target_x = 4; if (target_x > 123) target_x = 123;
    if (target_y < 4) target_y = 4; if (target_y > 59) target_y = 59;

    uint32_t current_time = osKernelGetTickCount();
    // 局部刷新效率高，可提高頻率至約 33fps (30ms)
    if (current_time - last_refresh_time >= 30) {

      // 1. 擦除「舊位置」的小球 (畫黑色)
      OLED_DrawBubble((int)last_x, (int)last_y, 0);

      // 2. 修補被小球蓋住過的背景十字線
      OLED_Repair_Crosshair((int)last_x, (int)last_y);

      // 3. 在「新位置」畫小球 (畫白色)
      OLED_DrawBubble((int)target_x, (int)target_y, 1);

      // 4. 執行刷新 (OLED_Refresh 會自動只送出有變動的 Page)
      OLED_Refresh();

      // 更新舊座標紀錄
      last_x = target_x;
      last_y = target_y;
      last_refresh_time = current_time;
    }

    osDelay(10); 
  }
}

void Service_Init(void) {
  if (binSemButtonHandle == NULL) {
    binSemButtonHandle = osSemaphoreNew(1, 0, NULL);
  }
  if (binSemMpuIntHandle == NULL) {
    binSemMpuIntHandle = osSemaphoreNew(1, 0, NULL);
  }
  if (binSemI2c1DoneHandle == NULL) {
    binSemI2c1DoneHandle = osSemaphoreNew(1, 0, NULL);
  }
  // 提升靈敏度：調小 R_measure (信任 G-sensor)，調大 Q_angle (信任變化)
  Kalman_Init(&kalmanX, 0.005f, 0.003f, 0.005f);
  Kalman_Init(&kalmanY, 0.005f, 0.003f, 0.005f);
}

void HAL_I2C_MemRxCpltCallback(I2C_HandleTypeDef *hi2c) {
  if (hi2c->Instance == I2C1) {
    if (binSemI2c1DoneHandle != NULL) osSemaphoreRelease(binSemI2c1DoneHandle);
  }
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
  if (GPIO_Pin & GPIO_PIN_13) {
    if (binSemButtonHandle != NULL) osSemaphoreRelease(binSemButtonHandle);
  }
  if (GPIO_Pin & GPIO_PIN_14) {
    if (binSemMpuIntHandle != NULL) osSemaphoreRelease(binSemMpuIntHandle);
  }
}

void Button_Process_Task(void) {
  if (osSemaphoreAcquire(binSemButtonHandle, osWaitForever) == osOK) {
    osDelay(10);
    if (HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_13) == GPIO_PIN_SET) {
      while (HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_13) == GPIO_PIN_SET) osDelay(10);
      osDelay(10);
      calibrate_flag = 1;
      while (osSemaphoreAcquire(binSemButtonHandle, 0) == osOK);
    }
  }
}
