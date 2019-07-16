#ifndef __Lcd_Driver_H
#define __Lcd_Driver_H

#include "stdint.h"
#include "../lvgl/lvgl.h"


/******************************************************************************
�ӿڶ�����Lcd_Driver.h�ڶ��壬����ݽ����޸Ĳ��޸���ӦIO��ʼ��LCD_GPIO_Init()


#define LCD_BLK         //18    	
#define LCD_RS         	//19 
#define LCD_CS        	//4
#define LCD_SDO     	//5 
#define LCD_SCL        	//16
#define LCD_SDA        	//17
*******************************************************************************/


#define GPIO_OUTPUT_IO_LCD_BLK    18
#define GPIO_OUTPUT_IO_LCD_RS    19
#define GPIO_OUTPUT_IO_LCD_CS    4
#define GPIO_OUTPUT_IO_LCD_SDO    5
#define GPIO_OUTPUT_IO_LCD_SCL    16
#define GPIO_OUTPUT_IO_LCD_SDA   17


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

#define TFT144lcd_HOR_RES X_MAX_PIXEL
#define TFT144lcd_VER_RES Y_MAX_PIXEL


/*
#define LCD_CTRL   	  	  GPIOB		   //����TFT���ݶ˿�
#define LCD_CTRL2   	  	GPIOC		   //����TFT���ݶ˿�
#define LCD_BLK        	GPIO_Pin_7   //PC7
#define LCD_RS         	GPIO_Pin_6	 //PC6
#define LCD_CS        	GPIO_Pin_12  //PB12

#define LCD_SCL        	GPIO_Pin_13	 //PB13
#define LCD_SDA        	GPIO_Pin_15	 //PB15



//Һ�����ƿ���1�������궨��
#define	LCD_CS_SET  	LCD_CTRL->BSRR=LCD_CS    
#define	LCD_SDA_SET  	LCD_CTRL->BSRR=LCD_SDA 
#define	LCD_SCL_SET  	LCD_CTRL->BSRR=LCD_SCL 

#define	LCD_RS_SET  	LCD_CTRL2->BSRR=LCD_RS 
#define	LCD_BLK_SET  	LCD_CTRL2->BSRR=LCD_BLK   

//Һ�����ƿ���0�������궨��
#define	LCD_CS_CLR  	LCD_CTRL->BRR=LCD_CS    
#define	LCD_SDA_CLR  	LCD_CTRL->BRR=LCD_SDA    
#define	LCD_SCL_CLR  	LCD_CTRL->BRR=LCD_SCL    

#define	LCD_RS_CLR  	LCD_CTRL2->BRR=LCD_RS 
#define	LCD_BLK_CLR  	LCD_CTRL2->BRR=LCD_BLK 


#define LCD_DATAOUT(x) LCD_DATA->ODR=x; //�������
#define LCD_DATAIN     LCD_DATA->IDR;   //��������
*/
void TFT144lcd_flush(int32_t x1, int32_t y1, int32_t x2, int32_t y2, const lv_color_t * color_map);
void TFT144lcd_fill(int32_t x1, int32_t y1, int32_t x2, int32_t y2, lv_color_t color);

void LCD_GPIO_Init(void);
void Lcd_WriteIndex(unsigned char Index);
void Lcd_WriteData(unsigned char Data);
void Lcd_WriteReg(unsigned char Index,unsigned char Data);
unsigned int Lcd_ReadReg(unsigned char LCD_Reg);
void Lcd_Reset(void);
void Lcd_Init(void);
void Lcd_Clear(unsigned int Color);
void Lcd_SetXY(unsigned int x,unsigned int y);
void Gui_DrawPoint(unsigned int x,unsigned int y,unsigned int Data);
//unsigned int Lcd_ReadPoint(unsigned int x,unsigned int y);
void Lcd_SetRegion(unsigned int x_start,unsigned int y_start,unsigned int x_end,unsigned int y_end);
void LCD_WriteData_16Bit(unsigned int Data);

#endif
