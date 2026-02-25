#include "Oled/oled.h"
#include "Oled/oledfont.h"

static I2C_HandleTypeDef *oled_i2c;

static HAL_StatusTypeDef OLED_I2C_Write(uint8_t control, uint8_t data)
{
  uint8_t buffer[2];

  if (oled_i2c == NULL)
  {
    return HAL_ERROR;
  }

  buffer[0] = control;
  buffer[1] = data;
  return HAL_I2C_Master_Transmit(oled_i2c,
                                 (uint16_t)(OLED_I2C_ADDR_7BIT << 1U),
                                 buffer,
                                 (uint16_t)sizeof(buffer),
                                 OLED_I2C_TIMEOUT_MS);
}

void OLED_WR_Byte(uint8_t dat, uint8_t cmd)
{
  if (cmd != 0U)
  {
    (void)OLED_I2C_Write(0x40U, dat);
  }
  else
  {
    (void)OLED_I2C_Write(0x00U, dat);
  }
}

void OLED_Display_On(void)
{
  OLED_WR_Byte(0x8DU, OLED_CMD);
  OLED_WR_Byte(0x14U, OLED_CMD);
  OLED_WR_Byte(0xAFU, OLED_CMD);
}

void OLED_Display_Off(void)
{
  OLED_WR_Byte(0x8DU, OLED_CMD);
  OLED_WR_Byte(0x10U, OLED_CMD);
  OLED_WR_Byte(0xAEU, OLED_CMD);
}

void OLED_Clear(void)
{
  uint8_t i;
  uint8_t n;

  for (i = 0U; i < 8U; i++)
  {
    OLED_WR_Byte((uint8_t)(0xB0U + i), OLED_CMD);
    OLED_WR_Byte(0x00U, OLED_CMD);
    OLED_WR_Byte(0x10U, OLED_CMD);
    for (n = 0U; n < 128U; n++)
    {
      OLED_WR_Byte(0x00U, OLED_DATA);
    }
  }
}

void OLED_Set_Pos(uint8_t x, uint8_t y)
{
  OLED_WR_Byte((uint8_t)(0xB0U + y), OLED_CMD);
  OLED_WR_Byte((uint8_t)(((x & 0xF0U) >> 4U) | 0x10U), OLED_CMD);
  OLED_WR_Byte((uint8_t)(x & 0x0FU), OLED_CMD);
}

void OLED_ShowChar(uint8_t x, uint8_t y, uint8_t chr, uint8_t char_size)
{
  uint8_t i;
  uint8_t c;

  c = (uint8_t)(chr - ' ');
  if (x > (Max_Column - 1U))
  {
    x = 0U;
    y = (uint8_t)(y + 2U);
  }

  if (char_size == 16U)
  {
    OLED_Set_Pos(x, y);
    for (i = 0U; i < 8U; i++)
    {
      OLED_WR_Byte(F8X16[(uint16_t)c * 16U + i], OLED_DATA);
    }
    OLED_Set_Pos(x, (uint8_t)(y + 1U));
    for (i = 0U; i < 8U; i++)
    {
      OLED_WR_Byte(F8X16[(uint16_t)c * 16U + i + 8U], OLED_DATA);
    }
  }
  else
  {
    OLED_Set_Pos(x, y);
    for (i = 0U; i < 6U; i++)
    {
      OLED_WR_Byte(F6x8[c][i], OLED_DATA);
    }
  }
}

static uint32_t oled_pow(uint8_t m, uint8_t n)
{
  uint32_t result = 1U;

  while (n--)
  {
    result *= m;
  }
  return result;
}

void OLED_ShowNum(uint8_t x, uint8_t y, uint32_t num, uint8_t len, uint8_t size)
{
  uint8_t t;
  uint8_t temp;
  uint8_t enshow = 0U;

  for (t = 0U; t < len; t++)
  {
    temp = (uint8_t)((num / oled_pow(10U, (uint8_t)(len - t - 1U))) % 10U);
    if ((enshow == 0U) && (t < (len - 1U)))
    {
      if (temp == 0U)
      {
        OLED_ShowChar((uint8_t)(x + (size / 2U) * t), y, ' ', size);
        continue;
      }
      else
      {
        enshow = 1U;
      }
    }
    OLED_ShowChar((uint8_t)(x + (size / 2U) * t), y, (uint8_t)(temp + '0'), size);
  }
}

void OLED_ShowString(uint8_t x, uint8_t y, const char *chr, uint8_t char_size)
{
  uint8_t j = 0U;

  if (chr == NULL)
  {
    return;
  }

  while (chr[j] != '\0')
  {
    OLED_ShowChar(x, y, (uint8_t)chr[j], char_size);
    x = (uint8_t)(x + 8U);
    if (x > 120U)
    {
      x = 0U;
      y = (uint8_t)(y + 2U);
    }
    j++;
  }
}

void OLED_ShowCHinese(uint8_t x, uint8_t y, uint8_t no)
{
  uint8_t t;

  OLED_Set_Pos(x, y);
  for (t = 0U; t < 16U; t++)
  {
    OLED_WR_Byte(Hzk[2U * no][t], OLED_DATA);
  }
  OLED_Set_Pos(x, (uint8_t)(y + 1U));
  for (t = 0U; t < 16U; t++)
  {
    OLED_WR_Byte(Hzk[2U * no + 1U][t], OLED_DATA);
  }
}

void OLED_DrawBMP(uint8_t x0, uint8_t y0, uint8_t x1, uint8_t y1, const unsigned char *bmp)
{
  uint32_t j = 0U;
  uint8_t x;
  uint8_t y;

  if (bmp == NULL)
  {
    return;
  }

  if ((y1 % 8U) == 0U)
  {
    y = (uint8_t)(y1 / 8U);
  }
  else
  {
    y = (uint8_t)(y1 / 8U + 1U);
  }

  for (y = y0; y < y1; y++)
  {
    OLED_Set_Pos(x0, y);
    for (x = x0; x < x1; x++)
    {
      OLED_WR_Byte(bmp[j++], OLED_DATA);
    }
  }
}

void OLED_Init(I2C_HandleTypeDef *hi2c)
{
  oled_i2c = hi2c;

  HAL_Delay(800U);
  OLED_WR_Byte(0xAEU, OLED_CMD);
  OLED_WR_Byte(0x00U, OLED_CMD);
  OLED_WR_Byte(0x10U, OLED_CMD);
  OLED_WR_Byte(0x40U, OLED_CMD);
  OLED_WR_Byte(0xB0U, OLED_CMD);
  OLED_WR_Byte(0x81U, OLED_CMD);
  OLED_WR_Byte(0xFFU, OLED_CMD);
  OLED_WR_Byte(0xA1U, OLED_CMD);
  OLED_WR_Byte(0xA6U, OLED_CMD);
  OLED_WR_Byte(0xA8U, OLED_CMD);
  OLED_WR_Byte(0x3FU, OLED_CMD);
  OLED_WR_Byte(0xC8U, OLED_CMD);
  OLED_WR_Byte(0xD3U, OLED_CMD);
  OLED_WR_Byte(0x00U, OLED_CMD);
  OLED_WR_Byte(0xD5U, OLED_CMD);
  OLED_WR_Byte(0x80U, OLED_CMD);
  OLED_WR_Byte(0xD8U, OLED_CMD);
  OLED_WR_Byte(0x05U, OLED_CMD);
  OLED_WR_Byte(0xD9U, OLED_CMD);
  OLED_WR_Byte(0xF1U, OLED_CMD);
  OLED_WR_Byte(0xDAU, OLED_CMD);
  OLED_WR_Byte(0x12U, OLED_CMD);
  OLED_WR_Byte(0xDBU, OLED_CMD);
  OLED_WR_Byte(0x30U, OLED_CMD);
  OLED_WR_Byte(0x8DU, OLED_CMD);
  OLED_WR_Byte(0x14U, OLED_CMD);
  OLED_WR_Byte(0xAFU, OLED_CMD);
}

void Delay_50ms(uint32_t delay_50ms)
{
  HAL_Delay(delay_50ms * 50U);
}

void Delay_1ms(uint32_t delay_1ms)
{
  HAL_Delay(delay_1ms);
}
