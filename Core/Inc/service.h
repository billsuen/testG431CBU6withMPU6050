/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    service.h
  * @brief   此文件定義 MPU 讀取、按鍵處理與服務狀態查詢介面
  ******************************************************************************
  */
/* USER CODE END Header */

#ifndef __SERVICE_H
#define __SERVICE_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
#include "cmsis_os.h"

/* 外部宣告訊號量控制代 */
extern osSemaphoreId_t binSemButtonHandle;
extern osSemaphoreId_t binSemMpuIntHandle;

/* 感測與狀態查詢介面 */
void Service_GetOrientationSnapshot(float *angle_x,
                                    float *angle_y,
                                    float *offset_angle_x,
                                    float *offset_angle_y);
uint8_t Service_GetMpuInitStatus(void);

/* 服務初始化與控制函式 */
void Service_Init(void);
void Button_Process_Task(void);

/* 任務函式 */
void MPU6050_Read_Task(void);

#ifdef __cplusplus
}
#endif

#endif /* __SERVICE_H */
