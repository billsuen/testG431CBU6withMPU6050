/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    service.h
  * @brief   此文件包含按鍵與指示燈控制的服務函式定義
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

/* 服務初始化與處理函式 */
void Service_Init(void);
void Button_Process_Task(void);
void Service_OLED_DisplayInfo(void);

/* 任務函式 */
void MPU6050_Read_Task(void);
void OLED_Display_Task(void);

#ifdef __cplusplus
}
#endif

#endif /* __SERVICE_H */
