
#ifndef _LCD_DRIVER_H_
#define _LCD_DRIVER_H_
#include "stdint.h"
#include "../lvgl/lvgl.h"



#define u16 unsigned int
#define u8 unsigned char

//-----------------------------SPI ��������--------------------------------------//
#define USE_HARDWARE_SPI     0  //1:Enable Hardware SPI;0:USE Soft SPI

//-------------------------��Ļ������������--------------------------------------//
#define LCD_X_SIZE	        176
#define LCD_Y_SIZE	        220

/////////////////////////////////////�û�������///////////////////////////////////	 
//֧�ֺ��������ٶ����л�
#define USE_HORIZONTAL  		1	//�����Ƿ�ʹ�ú��� 		0,��ʹ��.1,ʹ��.

#ifdef USE_HORIZONTAL//��������˺��� 
#define X_MAX_PIXEL	        LCD_Y_SIZE
#define Y_MAX_PIXEL	        LCD_X_SIZE
#else
#define X_MAX_PIXEL	        LCD_X_SIZE
#define Y_MAX_PIXEL	        LCD_Y_SIZE
#endif
//////////////////////////////////////////////////////////////////////////////////
	 

/*
#define GPIO_OUTPUT_IO_LCD_BLK    18
#define GPIO_OUTPUT_IO_LCD_RS    19
#define GPIO_OUTPUT_IO_LCD_CS    4
#define GPIO_OUTPUT_IO_LCD_SDO    5
#define GPIO_OUTPUT_IO_LCD_SCL    16
#define GPIO_OUTPUT_IO_LCD_SDA   17
*/
#define GPIO_OUTPUT_IO_LCD_BLK    19
#define GPIO_OUTPUT_IO_LCD_CS    4
#define GPIO_OUTPUT_IO_LCD_SDO    16
#define GPIO_OUTPUT_IO_LCD_RS    17
#define GPIO_OUTPUT_IO_LCD_SCL    18
#define GPIO_OUTPUT_IO_LCD_SDA   5


#define GPIO_OUTPUT_PIN_SEL  ((1ULL<<GPIO_OUTPUT_IO_LCD_BLK) | (1ULL<<GPIO_OUTPUT_IO_LCD_RS)| (1ULL<<GPIO_OUTPUT_IO_LCD_CS)| (1ULL<<GPIO_OUTPUT_IO_LCD_SDO)| (1ULL<<GPIO_OUTPUT_IO_LCD_SCL)| (1ULL<<GPIO_OUTPUT_IO_LCD_SDA))





#define RED  	0xf800
#define GREEN	0x07e0
#define BLUE 	0x001f
#define WHITE	0xffff
#define BLACK	0x0000
#define YELLOW  0xFFE0
#define GRAY0   0xEF7D   	//��ɫ0 3165 00110 001011 00101
#define GRAY1   0x8410      	//��ɫ1      00000 000000 00000
#define GRAY2   0x4208      	//��ɫ2  1111111111011111




void LCD_GPIO_Init(void);
void Lcd_WriteIndex(u8 Index);
void Lcd_WriteData(u8 Data);
void Lcd_WriteReg(u8 Index,u8 Data);
u16 Lcd_ReadReg(u8 LCD_Reg);
void Lcd_Reset(void);
void Lcd_Init(void);
void Lcd_Clear(u16 Color);
void Lcd_SetXY(u16 x,u16 y);
void Gui_DrawPoint(u16 x,u16 y,u16 Data);
unsigned int Lcd_ReadPoint(u16 x,u16 y);
void Lcd_SetRegion(u8 x_start,u8 y_start,u8 x_end,u8 y_end);
void Lcd_WriteData_16Bit(u16 Data);

void TFT22LCD_Color_fill(int32_t x1, int32_t y1, int32_t x2, int32_t y2, lv_color_t  color);

void TFT22lcd_flush(int32_t x1, int32_t y1, int32_t x2, int32_t y2, const lv_color_t * color_map);
void TFT22LCD_Color_Fill1(unsigned int x1, unsigned int y1, unsigned int x2, unsigned int y2, int16_t *color);

void testfill(void);

/////////////////spi master///////////////////////
void TFT32lcdspi_flush(int32_t x1, int32_t y1, int32_t x2, int32_t y2, const lv_color_t * color_map);
void drawPixel(int16_t x, int16_t y, uint16_t color);
void fill_color(int16_t x0, int16_t y0, int16_t x1, int16_t y1,uint16_t color);

void lcdspi_init();
//////////////////////////////////

#endif
