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
#include <string.h>

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
static uint8_t OLED_Background[1024]; // 🔹 存放靜態背景 (十字線)

#define MPU6050_ADDR         0xD0
#define MPU6050_PWR_MGMT_1   0x6B
#define MPU6050_CONFIG       0x1A
#define MPU6050_ACCEL_XOUT_H 0x3B
#define MPU6050_INT_ENABLE   0x38
#define MPU6050_INT_PIN_CFG  0x37
#define MPU6050_INT_STATUS   0x3A
#define MPU6050_WHO_AM_I     0x75

#define RAD_TO_DEG     57.2957795f

/* --- 顯存繪圖函式 (含 Window Update 與 memcpy 優化) --- */

/**
 * @brief 繪製像素
 */
void OLED_DrawPixel(int x, int y, uint8_t color) {
  if (x < 0 || x > 127 || y < 0 || y > 63) return;
  if (color) OLED_Buffer[x + (y / 8) * 128] |= (1 << (y % 8));
  else       OLED_Buffer[x + (y / 8) * 128] &= ~(1 << (y % 8));
}

/**
 * @brief 繪製小球
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
 * @brief 🔹 區域刷新：只傳送指定矩形區域的資料到 OLED (極大節省 I2C 頻寬)
 */
void OLED_Refresh_Region(int x, int y, int w, int h) {
  int start_page = y / 8;
  int end_page = (y + h - 1) / 8;
  // 邊界檢查
  if (x < 0) { w += x; x = 0; }
  if (x + w > 128) w = 128 - x;
  if (w <= 0) return;

  for (int p = start_page; p <= end_page; p++) {
    if (p < 0 || p > 7) continue;
    OLED_Set_Pos(x, p); // 設定 OLED 游標位置
    // 🔹 只傳送該 Page 中受影響的 w 個 Byte
    HAL_I2C_Mem_Write(&hi2c3, OLED_ADDRESS, 0x40, 1, &OLED_Buffer[x + p * 128], w, 10);
  }
}

/**
 * @brief 🔹 恢復背景：從背景緩衝區快速拷貝資料回顯存 (使用 memcpy)
 */
void OLED_Restore_BG(int x, int y, int w, int h) {
  int start_page = y / 8;
  int end_page = (y + h - 1) / 8;
  for (int p = start_page; p <= end_page; p++) {
    if (p < 0 || p > 7) continue;
    int offset = x + p * 128;
    int copy_w = (128 - x < w) ? (128 - x) : w;
    if (copy_w > 0) memcpy(&OLED_Buffer[offset], &OLED_Background[offset], copy_w);
  }
}

/**
 * @brief 全螢幕刷新
 */
void OLED_Refresh_All(void) {
  for (uint8_t i = 0; i < 8; i++) {
    OLED_Set_Pos(0, i);
    HAL_I2C_Mem_Write(&hi2c3, OLED_ADDRESS, 0x40, 1, &OLED_Buffer[128 * i], 128, 100);
  }
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

  // --- 🔹 初始繪製背景並保存 ---
  memset(OLED_Buffer, 0, 1024);
  // 畫十字線到 Buffer
  for(int i=0; i<128; i++) OLED_DrawPixel(i, 32, 1); 
  for(int i=0; i<64; i++)  OLED_DrawPixel(64, i, 1); 
  // 🔹 將繪製好的背景備份到 Background Buffer
  memcpy(OLED_Background, OLED_Buffer, 1024);
  
  OLED_Refresh_All(); // 首次全屏刷新

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

    // 限制範圍防止小球出界 (留 4 像素邊距)
    if (target_x < 4) target_x = 4; if (target_x > 123) target_x = 123;
    if (target_y < 4) target_y = 4; if (target_y > 59) target_y = 59;

    uint32_t current_time = osKernelGetTickCount();
    if (current_time - last_refresh_time >= 30) {
      
      // 1. 🔹 恢復「舊位置」的背景 (使用 memcpy，範圍 8x8)
      OLED_Restore_BG((int)last_x - 4, (int)last_y - 4, 8, 8);
      
      // 2. 🔹 在「新位置」繪製小球 (僅修改 Buffer)
      OLED_DrawBubble((int)target_x, (int)target_y, 1);
      
      // 3. 🔹 區域刷新 I2C (僅傳送舊位置與新位置受影響的區域)
      // 為簡單起見，我們刷新舊位置與新位置的聯集或分別刷新
      OLED_Refresh_Region((int)last_x - 4, (int)last_y - 4, 8, 8);
      OLED_Refresh_Region((int)target_x - 4, (int)target_y - 4, 8, 8);
      
      // 更新紀錄
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
