#include "OLED128x64_Fast.h"
#include "OLED_FONTs.h"

// 保存 I2C 實例指針
static I2C_HandleTypeDef *phi2c_oled = NULL;

// --- 服務函式實作 ---

/**
 * @brief 寫入指令到 OLED
 */
void OLEDWrCmd(uint8_t command) {
    if (phi2c_oled == NULL) return;
    HAL_I2C_Mem_Write(phi2c_oled, OLED_ADDRESS, OLED_Command_Stream, I2C_MEMADD_SIZE_8BIT, &command, 1, 100);
}

/**
 * @brief 寫入數據到 OLED
 */
void OLEDWrDat(uint8_t data) {
    if (phi2c_oled == NULL) return;
    HAL_I2C_Mem_Write(phi2c_oled, OLED_ADDRESS, OLED_Data_Stream, I2C_MEMADD_SIZE_8BIT, &data, 1, 100);
}

void OLED_ON(void) {
    OLEDWrCmd(0x8D); OLEDWrCmd(0x14); 
    OLEDWrCmd(0xAF); 
}

void OLED_OFF(void) {
    OLEDWrCmd(0x8D); OLEDWrCmd(0x10); 
    OLEDWrCmd(0xAE); 
}

/**
 * @brief 設定顯示位置 (Page Addressing Mode)
 */
void OLED_Set_Pos(uint8_t x, uint8_t y) {
    uint8_t cmds[3];
    cmds[0] = 0xb0 + y;
    cmds[1] = ((x & 0xf0) >> 4) | 0x10;
    cmds[2] = (x & 0x0f) | 0x00;
    
    HAL_I2C_Mem_Write(phi2c_oled, OLED_ADDRESS, OLED_Command_Stream, I2C_MEMADD_SIZE_8BIT, cmds, 3, 100);
}

/**
 * @brief 填充全螢幕
 */
void OLED_Fill(uint8_t fill_Data) {
    uint8_t data_val = (fill_Data == WHITE) ? 0xFF : (fill_Data == BLACK) ? 0x00 : fill_Data;
    uint8_t buffer[132]; // 稍微多一點以覆蓋某些驅動晶片的緩存邊界
    
    for(uint16_t i=0; i<sizeof(buffer); i++) buffer[i] = data_val;

    for(uint8_t y=0; y<8; y++) {
        OLED_Set_Pos(0, y);
        HAL_I2C_Mem_Write(phi2c_oled, OLED_ADDRESS, OLED_Data_Stream, I2C_MEMADD_SIZE_8BIT, buffer, 132, 500);
    }
}

void OLED_CLS(void) { OLED_Fill(BLACK); }

/**
 * @brief 初始化 OLED
 * @param hi2c 已配置好的 I2C 實例指針
 */
void OLED_Init(I2C_HandleTypeDef *hi2c) {
    phi2c_oled = hi2c;
    
    HAL_Delay(50); 
    
    static const uint8_t init_cmds[] = {
        0xAE, 0x20, 0x02, 0xA8, 0x3F, 0xD3, 0x00, 0x40, 
        0xA1, 0xC8, 0xDA, 0x12, 0x81, 0x7F, 0xA4, 0xA6, 
        0xD5, 0x80, 0x8D, 0x14, 0xAF
    };

    HAL_I2C_Mem_Write(phi2c_oled, OLED_ADDRESS, OLED_Command_Stream, I2C_MEMADD_SIZE_8BIT, (uint8_t*)init_cmds, sizeof(init_cmds), 500);
    
    OLED_CLS();
}

void OLED_Put6x8Str(uint8_t x, uint8_t y, const char ch[]) {
    uint8_t c, j = 0;
    while (ch[j] != '\0') {
        c = ch[j] - 32;
        if (x > (X_WIDTH - 6)) { x = 0; y++; }
        OLED_Set_Pos(x, y);
        HAL_I2C_Mem_Write(phi2c_oled, OLED_ADDRESS, OLED_Data_Stream, I2C_MEMADD_SIZE_8BIT, (uint8_t*)Font6x8[c], 6, 100);
        x += 6; j++;
    }
}

void OLED_Put8x16ASCII(uint8_t x, uint8_t y, uint8_t ch) {
    uint16_t c = (ch - 32);
    for(uint8_t page = 0; page < 2; page++) {
        OLED_Set_Pos(x, y + page);
        HAL_I2C_Mem_Write(phi2c_oled, OLED_ADDRESS, OLED_Data_Stream, I2C_MEMADD_SIZE_8BIT, (uint8_t*)&Font8x16[c][page * 8], 8, 100);
    }
}

void OLED_Put8x16Str(uint8_t x, uint8_t y, const char ch[]) {
    uint8_t j = 0;
    while (ch[j] != '\0') {
        if (x > (X_WIDTH - 8)) { x = 0; y += 2; }
        OLED_Put8x16ASCII(x, y, ch[j]);
        x += 8; j++;
    }
}

void Draw_BMP(uint8_t x0, uint8_t y0, uint8_t x1, uint8_t y1, const uint8_t BMP[]) {
    uint16_t j = 0;
    uint8_t width = x1 - x0;
    for(uint8_t y = y0; y < y1; y++) {
        OLED_Set_Pos(x0, y);
        HAL_I2C_Mem_Write(phi2c_oled, OLED_ADDRESS, OLED_Data_Stream, I2C_MEMADD_SIZE_8BIT, (uint8_t*)&BMP[j], width, 500);
        j += width;
    }
}
