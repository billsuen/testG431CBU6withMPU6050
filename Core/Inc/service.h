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

/* 外部宣告訊號量控制代，讓 app_freertos.c 可以建立它 */
extern osSemaphoreId_t binSemButtonHandle;

/* 服務初始化與處理函式 */
void Service_Init(void);
void Button_Process_Task(void);
void Service_OLED_DisplayInfo(void);
void MPU6050_BubbleLevel_Task(void);

#ifdef __cplusplus
}
#endif

#endif /* __SERVICE_H */
