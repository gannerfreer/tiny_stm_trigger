#ifndef OLED_H
#define OLED_H

#include "stm32f0xx_hal.h"
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define OLED_MODE 0U
#define SIZE 8U
#define XLevelL 0x00U
#define XLevelH 0x10U
#define Max_Column 128U
#define Max_Row 64U
#define Brightness 0xFFU
#define X_WIDTH 128U
#define Y_WIDTH 64U

#ifndef OLED_I2C_ADDR_7BIT
#define OLED_I2C_ADDR_7BIT 0x3CU
#endif

#ifndef OLED_I2C_TIMEOUT_MS
#define OLED_I2C_TIMEOUT_MS 20U
#endif

#define OLED_CMD 0U
#define OLED_DATA 1U

void OLED_Init(I2C_HandleTypeDef *hi2c);
void OLED_Display_On(void);
void OLED_Display_Off(void);
void OLED_Clear(void);
void OLED_Set_Pos(uint8_t x, uint8_t y);
void OLED_ShowChar(uint8_t x, uint8_t y, uint8_t chr, uint8_t char_size);
void OLED_ShowNum(uint8_t x, uint8_t y, uint32_t num, uint8_t len, uint8_t size);
void OLED_ShowString(uint8_t x, uint8_t y, const char *p, uint8_t char_size);
void OLED_ShowCHinese(uint8_t x, uint8_t y, uint8_t no);
void OLED_DrawBMP(uint8_t x0, uint8_t y0, uint8_t x1, uint8_t y1, const unsigned char *bmp);
void OLED_WR_Byte(uint8_t dat, uint8_t cmd);
uint8_t OLED_Debug_PrintLine(uint8_t line, const char *text);
void Delay_50ms(uint32_t delay_50ms);
void Delay_1ms(uint32_t delay_1ms);

#ifdef __cplusplus
}
#endif

#endif



