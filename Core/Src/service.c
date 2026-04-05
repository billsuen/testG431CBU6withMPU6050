/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    service.c
  * @brief   此文件處理 MPU6050/6500 讀取（含濾波）、按鍵控制與服務狀態
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "service.h"
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

#define MPU6050_ADDR         0xD0
#define MPU6050_PWR_MGMT_1   0x6B
#define MPU6050_CONFIG       0x1A
#define MPU6050_ACCEL_XOUT_H 0x3B
#define MPU6050_INT_ENABLE   0x38
#define MPU6050_INT_PIN_CFG  0x37
#define MPU6050_INT_STATUS   0x3A
#define MPU6050_WHO_AM_I     0x75

#define RAD_TO_DEG     57.2957795f

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

void Service_GetOrientationSnapshot(float *out_angle_x,
                                    float *out_angle_y,
                                    float *out_offset_angle_x,
                                    float *out_offset_angle_y) {
  if (out_angle_x != NULL) {
    *out_angle_x = angle_x;
  }
  if (out_angle_y != NULL) {
    *out_angle_y = angle_y;
  }
  if (out_offset_angle_x != NULL) {
    *out_offset_angle_x = offset_angle_x;
  }
  if (out_offset_angle_y != NULL) {
    *out_offset_angle_y = offset_angle_y;
  }
}

uint8_t Service_GetMpuInitStatus(void) {
  return mpu_init_status;
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
