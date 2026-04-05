/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    OLED_service.c
  * @brief   此文件處理 OLED 顯示緩衝與水平儀顯示邏輯
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "OLED_service.h"
#include "service.h"
#include "../OLED_128x64/OLED128x64_Fast.h"
#include "i2c.h"
#include <stdint.h>
#include <string.h>

/* Private variables ---------------------------------------------------------*/
static uint8_t OLED_Buffer[1024];
static uint8_t OLED_Background[1024];

/* Private function prototypes -----------------------------------------------*/
static void OLED_DrawPixel(int x, int y, uint8_t color);
static void OLED_DrawBubble(int x, int y, uint8_t color);
static void OLED_Refresh_Region(int x, int y, int w, int h);
static void OLED_Restore_BG(int x, int y, int w, int h);
static void OLED_Refresh_All(void);
static void OLED_Display_Init(void);
static void OLED_GetBubbleUnionRegion(int old_x,
                                      int old_y,
                                      int new_x,
                                      int new_y,
                                      int *region_x,
                                      int *region_y,
                                      int *region_w,
                                      int *region_h);

static void OLED_DrawPixel(int x, int y, uint8_t color) {
  if (x < 0 || x > 127 || y < 0 || y > 63) {
    return;
  }

  if (color) {
    OLED_Buffer[x + (y / 8) * 128] |= (1U << (y % 8));
  } else {
    OLED_Buffer[x + (y / 8) * 128] &= (uint8_t)~(1U << (y % 8));
  }
}

static void OLED_DrawBubble(int x, int y, uint8_t color) {
  static const uint8_t bubble_bmp[] = {
      0x3C, 0x7E, 0xFF, 0xFF, 0xFF, 0xFF, 0x7E, 0x3C};

  for (int i = 0; i < 8; i++) {
    for (int j = 0; j < 8; j++) {
      if ((bubble_bmp[i] & (1U << j)) != 0U) {
        OLED_DrawPixel(x + i - 4, y + j - 4, color);
      }
    }
  }
}

static void OLED_Refresh_Region(int x, int y, int w, int h) {
  int start_page = y / 8;
  int end_page = (y + h - 1) / 8;

  if (x < 0) {
    w += x;
    x = 0;
  }
  if (x + w > 128) {
    w = 128 - x;
  }
  if (w <= 0) {
    return;
  }

  for (int page = start_page; page <= end_page; page++) {
    if (page < 0 || page > 7) {
      continue;
    }

    OLED_Set_Pos((uint8_t)x, (uint8_t)page);
    HAL_I2C_Mem_Write(&hi2c3, OLED_ADDRESS, 0x40, 1, &OLED_Buffer[x + page * 128], w, 10);
  }
}

static void OLED_Restore_BG(int x, int y, int w, int h) {
  int start_page = y / 8;
  int end_page = (y + h - 1) / 8;

  for (int page = start_page; page <= end_page; page++) {
    int offset;
    int copy_w;

    if (page < 0 || page > 7 || x >= 128) {
      continue;
    }

    offset = x + page * 128;
    copy_w = (128 - x < w) ? (128 - x) : w;
    if (copy_w > 0) {
      memcpy(&OLED_Buffer[offset], &OLED_Background[offset], (size_t)copy_w);
    }
  }
}

static void OLED_Refresh_All(void) {
  for (uint8_t page = 0; page < 8; page++) {
    OLED_Set_Pos(0, page);
    HAL_I2C_Mem_Write(&hi2c3, OLED_ADDRESS, 0x40, 1, &OLED_Buffer[128 * page], 128, 100);
  }
}

static void OLED_GetBubbleUnionRegion(int old_x,
                                      int old_y,
                                      int new_x,
                                      int new_y,
                                      int *region_x,
                                      int *region_y,
                                      int *region_w,
                                      int *region_h) {
  int left = (old_x < new_x) ? old_x : new_x;
  int top = (old_y < new_y) ? old_y : new_y;
  int right = ((old_x + 7) > (new_x + 7)) ? (old_x + 7) : (new_x + 7);
  int bottom = ((old_y + 7) > (new_y + 7)) ? (old_y + 7) : (new_y + 7);

  if (region_x != NULL) {
    *region_x = left;
  }
  if (region_y != NULL) {
    *region_y = top;
  }
  if (region_w != NULL) {
    *region_w = right - left + 1;
  }
  if (region_h != NULL) {
    *region_h = bottom - top + 1;
  }
}

static void OLED_Display_Init(void) {
  uint16_t timeout = 200;

  OLED_Init(&hi2c3);
  OLED_CLS();
  OLED_Put8x16Str(0, 0, "G431 Ready");

  while (Service_GetMpuInitStatus() == 0U && timeout-- > 0U) {
    osDelay(10);
  }

  if (Service_GetMpuInitStatus() == 1U) {
    OLED_Put8x16Str(0, 2, "MPU: 100Hz DMA");
  } else {
    OLED_Put8x16Str(0, 2, "MPU: ERROR!");
  }
}

void OLED_Display_Task(void) {
  float local_angle_x;
  float local_angle_y;
  float local_offset_x;
  float local_offset_y;
  float target_x;
  float target_y;
  int prev_bubble_x;
  int prev_bubble_y;
  int next_bubble_x;
  int next_bubble_y;
  int refresh_x;
  int refresh_y;
  int refresh_w;
  int refresh_h;
  static float last_x = 64.0f;
  static float last_y = 32.0f;
  static uint32_t last_refresh_time = 0;

  OLED_Display_Init();
  osDelay(3000);

  memset(OLED_Buffer, 0, sizeof(OLED_Buffer));
  for (int i = 0; i < 128; i++) {
    OLED_DrawPixel(i, 32, 1);
  }
  for (int i = 0; i < 64; i++) {
    OLED_DrawPixel(64, i, 1);
  }
  memcpy(OLED_Background, OLED_Buffer, sizeof(OLED_Buffer));

  OLED_Refresh_All();

  for (;;) {
    uint32_t current_time;

    Service_GetOrientationSnapshot(&local_angle_x, &local_angle_y, &local_offset_x, &local_offset_y);

    target_x = 64.0f + ((local_angle_x - local_offset_x) * 2.0f);
    target_y = 32.0f - ((local_angle_y - local_offset_y) * 2.0f);

    if (target_x < 4.0f) {
      target_x = 4.0f;
    }
    if (target_x > 123.0f) {
      target_x = 123.0f;
    }
    if (target_y < 4.0f) {
      target_y = 4.0f;
    }
    if (target_y > 59.0f) {
      target_y = 59.0f;
    }

    current_time = osKernelGetTickCount();
    if (current_time - last_refresh_time >= 30U) {
      prev_bubble_x = (int)last_x - 4;
      prev_bubble_y = (int)last_y - 4;
      next_bubble_x = (int)target_x - 4;
      next_bubble_y = (int)target_y - 4;

      OLED_GetBubbleUnionRegion(prev_bubble_x,
                                prev_bubble_y,
                                next_bubble_x,
                                next_bubble_y,
                                &refresh_x,
                                &refresh_y,
                                &refresh_w,
                                &refresh_h);

      OLED_Restore_BG(refresh_x, refresh_y, refresh_w, refresh_h);
      OLED_DrawBubble((int)target_x, (int)target_y, 1);
      OLED_Refresh_Region(refresh_x, refresh_y, refresh_w, refresh_h);

      last_x = target_x;
      last_y = target_y;
      last_refresh_time = current_time;
    }

    osDelay(10);
  }
}
