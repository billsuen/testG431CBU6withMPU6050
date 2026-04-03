/* * File: OLED128x64_Fast.h
 * 調用方法：
 * 1. 包含此檔：#include "OLED128x64_Fast.h"
 * 2. 啟動初始化：OLED_Init(); 
 * 3. 本版本內建 I2C Timeout 自動跳脫機制，確保系統不會因通訊異常死機。
 */

#ifndef OLED128X64_FAST_H
#define OLED128X64_FAST_H

// 根據您的 STM32 系列取消註釋或包含正確的標頭檔
// #include "stm32f1xx_hal.h" 
// #include "stm32f4xx_hal.h"
// #include "stm32l4xx_hal.h"
#include "main.h" // 通常 CubeMX 會在 main.h 中包含正確的 HAL 標頭檔

// --- 尺寸與位址定義 ---
#define X_WIDTH             128
#define Y_WIDTH             64
#define OLED_ADDRESS        (0x3C << 1) // STM32 HAL 使用 8-bit 位址

// --- 顏色與模式定義 ---
#define BLACK               0
#define WHITE               1
#define OLED_Command_Stream 0x00 
#define OLED_Data_Stream    0x40 

// --- 工具巨集 ---
#define swap(a, b) { uint8_t t = a; a = b; b = t; }

// --- 核心控制服務 ---
void OLED_Init(I2C_HandleTypeDef *hi2c);
void OLED_ON(void);
void OLED_OFF(void);
void OLED_CLS(void);
void OLED_Fill(uint8_t fill_Data);
void OLED_Set_Pos(uint8_t x, uint8_t y);

// --- 字串顯示服務 ---
void OLED_Put6x8Str(uint8_t x, uint8_t y, const char ch[]);
void OLED_Put8x16Str(uint8_t x, uint8_t y, const char ch[]);
void OLED_Put8x16ASCII(uint8_t x, uint8_t y, uint8_t ch);

// --- 繪圖服務 ---
void Draw_BMP(uint8_t x0, uint8_t y0, uint8_t x1, uint8_t y1, const uint8_t BMP[]);

// --- 底層 API ---
void OLEDWrCmd(uint8_t command);
void OLEDWrDat(uint8_t data);

#endif