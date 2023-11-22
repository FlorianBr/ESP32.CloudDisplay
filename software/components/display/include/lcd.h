/**
 ******************************************************************************
 *  file           : lcd.h
 *  brief          : THe small I2C-LCD
 ******************************************************************************
 *  Copyright (C) 2021 Florian Brandner
 */

#ifndef LCD_H_
#define LCD_H_

#ifdef __cplusplus
extern "C"
{
#endif

/* Includes ------------------------------------------------------------------*/

/* Private includes ----------------------------------------------------------*/

/* Exported types ------------------------------------------------------------*/
typedef struct {
    __uint8_t Red;
    __uint8_t Green;
    __uint8_t Blue;
} lcd_rgb_color_t;

/* Exported constants --------------------------------------------------------*/

/* Exported macro ------------------------------------------------------------*/

/* Exported functions prototypes ---------------------------------------------*/
esp_err_t   LCD_Init(void);
esp_err_t   LCD_Start(void);
__uint16_t  LCD_WriteString(__uint16_t Row, __uint16_t Col, char *cBuffer, lcd_rgb_color_t fgcolor, lcd_rgb_color_t bgcolor);
esp_err_t   LCD_SetRawData(__uint8_t * pData, size_t offset, size_t length);

#ifdef __cplusplus
}
#endif

#endif // LCD_H_
