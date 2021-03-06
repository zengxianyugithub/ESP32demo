//#include "stm32f10x.h"
#include "Lcd_Driver.h"
#include "LCD_Config.h"
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

//#;include "delay.h"


//STM32F103核心板例程
//库函数版本例程
/************** 嵌入式开发网  **************/
/********** mcudev.taobao.com 出品  ********/

//static void TFT144lcd_send_cmd(unsigned char cmd);
//static void TFT144lcd_send_data(unsigned char * data, unsigned int length);

//本测试程序使用的是模拟SPI接口驱动
//可自由更改接口IO配置，使用任意最少4 IO即可完成本款液晶驱动显示
//娑叉櫠鎺у埗鍙ｇ疆1鎿嶄綔璇彞瀹忓畾涔?
#define	LCD_CS_SET  	gpio_set_level(GPIO_OUTPUT_IO_LCD_CS, 1)   
#define	LCD_SDA_SET  	gpio_set_level(GPIO_OUTPUT_IO_LCD_SDA, 1)   
#define	LCD_SCL_SET  	gpio_set_level(GPIO_OUTPUT_IO_LCD_SCL, 1)   

#define	LCD_RS_SET  	gpio_set_level(GPIO_OUTPUT_IO_LCD_RS, 1)  
#define	LCD_BLK_SET  	gpio_set_level(GPIO_OUTPUT_IO_LCD_BLK, 1)   
#define	  LCD_RST_SET	gpio_set_level(GPIO_OUTPUT_IO_LCD_SDO, 1)   

//娑叉櫠鎺у埗鍙ｇ疆0鎿嶄綔璇彞瀹忓畾涔?
#define	LCD_CS_CLR  	gpio_set_level(GPIO_OUTPUT_IO_LCD_CS, 0)   
#define	LCD_SDA_CLR  	gpio_set_level(GPIO_OUTPUT_IO_LCD_SDA, 0)  
#define	LCD_SCL_CLR  	gpio_set_level(GPIO_OUTPUT_IO_LCD_SCL, 0)  

#define	LCD_RS_CLR  	gpio_set_level(GPIO_OUTPUT_IO_LCD_RS, 0)
#define	LCD_BLK_CLR  	gpio_set_level(GPIO_OUTPUT_IO_LCD_BLK, 0)

#define   LCD_RST_CLR gpio_set_level(GPIO_OUTPUT_IO_LCD_SDO, 0)

//#define LCD_DATAOUT(x) gpio_set_level(GPIO_OUTPUT_IO_LCD_CS, x); //鏁版嵁杈撳嚭


//液晶IO初始化配置
void LCD_GPIO_Init(void)
{

	gpio_config_t io_conf;
    //disable interrupt
    io_conf.intr_type = GPIO_PIN_INTR_DISABLE;
    //set as output mode
    io_conf.mode = GPIO_MODE_OUTPUT;
    //bit mask of the pins that you want to set,e.g.GPIO18/19
    io_conf.pin_bit_mask = GPIO_OUTPUT_PIN_SEL;
    //disable pull-down mode
    io_conf.pull_down_en = 0;
    //disable pull-up mode
    io_conf.pull_up_en = 0;
    //configure GPIO with the given settings
    gpio_config(&io_conf);
	
	LCD_BLK_SET;//开启背光
	LCD_BLK_CLR;//关闭背光
	LCD_BLK_SET;//开启背光
      
}
//向SPI总线传输一个8位数据
void  SPI_WriteData(unsigned char Data)
{
	unsigned char i=0;
	for(i=8;i>0;i--)
	{
		if(Data&0x80)	
	  LCD_SDA_SET; //输出数据
      else LCD_SDA_CLR;
	   
      LCD_SCL_CLR;       
      LCD_SCL_SET;
      Data<<=1; 
	}
}

//向液晶屏写一个8位指令
void Lcd_WriteIndex(unsigned char Index)
{
   //SPI 写命令时序开始
   LCD_CS_CLR;
   LCD_RS_CLR;
	 SPI_WriteData(Index);
   LCD_CS_SET;
}

//向液晶屏写一个8位数据
void Lcd_WriteData(unsigned char Data)
{
   LCD_CS_CLR;
   LCD_RS_SET;
   SPI_WriteData(Data);
   LCD_CS_SET; 
}
//向液晶屏写一个16位数据
void LCD_WriteData_16Bit(unsigned int Data)
{
   LCD_CS_CLR;
   LCD_RS_SET;
	 SPI_WriteData(Data>>8); 	//写入高8位数据
	 SPI_WriteData(Data); 			//写入低8位数据
   LCD_CS_SET; 
}

void Lcd_WriteReg(unsigned char Index,unsigned char Data)
{
	Lcd_WriteIndex(Index);
  Lcd_WriteData(Data);
}

void Lcd_Reset(void)
{
	LCD_RST_CLR;
	//delay_ms(100);
	vTaskDelay(100 / portTICK_RATE_MS);
	LCD_RST_SET;
	//delay_ms(50);
	vTaskDelay(50 / portTICK_RATE_MS);
}

//LCD Init For 1.44Inch LCD Panel with ST7735R.
void Lcd_Init(void)
{	
	LCD_GPIO_Init();
	Lcd_Reset(); //Reset before LCD Init.

	//LCD Init For 1.44Inch LCD Panel with ST7735R.
	Lcd_WriteIndex(0x11);//Sleep exit 
	//delay_ms (120);
	vTaskDelay(120 / portTICK_RATE_MS);	
	//ST7735R Frame Rate
	Lcd_WriteIndex(0xB1); 
	Lcd_WriteData(0x01); 
	Lcd_WriteData(0x2C); 
	Lcd_WriteData(0x2D); 

	Lcd_WriteIndex(0xB2); 
	Lcd_WriteData(0x01); 
	Lcd_WriteData(0x2C); 
	Lcd_WriteData(0x2D); 

	Lcd_WriteIndex(0xB3); 
	Lcd_WriteData(0x01); 
	Lcd_WriteData(0x2C); 
	Lcd_WriteData(0x2D); 
	Lcd_WriteData(0x01); 
	Lcd_WriteData(0x2C); 
	Lcd_WriteData(0x2D); 
	
	Lcd_WriteIndex(0xB4); //Column inversion 
	Lcd_WriteData(0x07); 
	
	//ST7735R Power Sequence
	Lcd_WriteIndex(0xC0); 
	Lcd_WriteData(0xA2); 
	Lcd_WriteData(0x02); 
	Lcd_WriteData(0x84); 
	Lcd_WriteIndex(0xC1); 
	Lcd_WriteData(0xC5); 

	Lcd_WriteIndex(0xC2); 
	Lcd_WriteData(0x0A); 
	Lcd_WriteData(0x00); 

	Lcd_WriteIndex(0xC3); 
	Lcd_WriteData(0x8A); 
	Lcd_WriteData(0x2A); 
	Lcd_WriteIndex(0xC4); 
	Lcd_WriteData(0x8A); 
	Lcd_WriteData(0xEE); 
	
	Lcd_WriteIndex(0xC5); //VCOM 
	Lcd_WriteData(0x0E); 
	
	Lcd_WriteIndex(0x36); //MX, MY, RGB mode 
	Lcd_WriteData(0xC8); 
	
	//ST7735R Gamma Sequence
	Lcd_WriteIndex(0xe0); 
	Lcd_WriteData(0x0f); 
	Lcd_WriteData(0x1a); 
	Lcd_WriteData(0x0f); 
	Lcd_WriteData(0x18); 
	Lcd_WriteData(0x2f); 
	Lcd_WriteData(0x28); 
	Lcd_WriteData(0x20); 
	Lcd_WriteData(0x22); 
	Lcd_WriteData(0x1f); 
	Lcd_WriteData(0x1b); 
	Lcd_WriteData(0x23); 
	Lcd_WriteData(0x37); 
	Lcd_WriteData(0x00); 	
	Lcd_WriteData(0x07); 
	Lcd_WriteData(0x02); 
	Lcd_WriteData(0x10); 

	Lcd_WriteIndex(0xe1); 
	Lcd_WriteData(0x0f); 
	Lcd_WriteData(0x1b); 
	Lcd_WriteData(0x0f); 
	Lcd_WriteData(0x17); 
	Lcd_WriteData(0x33); 
	Lcd_WriteData(0x2c); 
	Lcd_WriteData(0x29); 
	Lcd_WriteData(0x2e); 
	Lcd_WriteData(0x30); 
	Lcd_WriteData(0x30); 
	Lcd_WriteData(0x39); 
	Lcd_WriteData(0x3f); 
	Lcd_WriteData(0x00); 
	Lcd_WriteData(0x07); 
	Lcd_WriteData(0x03); 
	Lcd_WriteData(0x10);  
	
	Lcd_WriteIndex(0x2a);
	Lcd_WriteData(0x00);
	Lcd_WriteData(0x00);
	Lcd_WriteData(0x00);
	Lcd_WriteData(0x7f);

	Lcd_WriteIndex(0x2b);
	Lcd_WriteData(0x00);
	Lcd_WriteData(0x00);
	Lcd_WriteData(0x00);
	Lcd_WriteData(0x9f);
	
	Lcd_WriteIndex(0xF0); //Enable test command  
	Lcd_WriteData(0x01); 
	Lcd_WriteIndex(0xF6); //Disable ram power save mode 
	Lcd_WriteData(0x00); 
	
	Lcd_WriteIndex(0x3A); //65k mode 
	Lcd_WriteData(0x05); 
	
	
	Lcd_WriteIndex(0x29);//Display on	 
}


/*************************************************
函数名：LCD_Set_Region
功能：设置lcd显示区域，在此区域写点数据自动换行
入口参数：xy起点和终点
返回值：无
*************************************************/
void Lcd_SetRegion(unsigned int x_start,unsigned int y_start,unsigned int x_end,unsigned int y_end)
{		
	Lcd_WriteIndex(0x2a);
	Lcd_WriteData(0x00);
	Lcd_WriteData(x_start+2);
	Lcd_WriteData(0x00);
	Lcd_WriteData(x_end+2);

	Lcd_WriteIndex(0x2b);
	Lcd_WriteData(0x00);
	Lcd_WriteData(y_start+3);
	Lcd_WriteData(0x00);
	Lcd_WriteData(y_end+3);
	
	Lcd_WriteIndex(0x2c);

}

/*************************************************
函数名：LCD_Set_XY
功能：设置lcd显示起始点
入口参数：xy坐标
返回值：无
*************************************************/
void Lcd_SetXY(unsigned int x,unsigned int y)
{
  	Lcd_SetRegion(x,y,x,y);
}

	
/*************************************************
函数名：LCD_DrawPoint
功能：画一个点
入口参数：无
返回值：无
*************************************************/
void Gui_DrawPoint(unsigned int x,unsigned int y,unsigned int Data)
{
	Lcd_SetRegion(x,y,x+1,y+1);
	LCD_WriteData_16Bit(Data);

}    

/*****************************************
 函数功能：读TFT某一点的颜色                          
 出口参数：color  点颜色值                                 
*****************************************
unsigned int Lcd_ReadPoint(unsigned int x,unsigned int y)
{
  unsigned int Data;
  Lcd_SetXY(x,y);

  //Lcd_ReadData();//丢掉无用字节
  //Data=Lcd_ReadData();
  Lcd_WriteData(Data);
  return Data;
}*/
/*************************************************
函数名：Lcd_Clear
功能：全屏清屏函数
入口参数：填充颜色COLOR
返回值：无
*************************************************/
void Lcd_Clear(unsigned int Color)               
{	
   unsigned int i,m;
   Lcd_SetRegion(0,0,X_MAX_PIXEL-1,Y_MAX_PIXEL-1);
   Lcd_WriteIndex(0x2C);
   for(i=0;i<X_MAX_PIXEL;i++)
    for(m=0;m<Y_MAX_PIXEL;m++)
    {	
	  	LCD_WriteData_16Bit(Color);
    }   
}

void TFT144LCD_Color_Fill(int16_t x1, int16_t y1, int16_t x2, int16_t y2, int16_t *color)
{  
	unsigned int height,width;
	unsigned int i,m;
	width=x2-x1+1; 			//寰楀埌濉厖鐨勫搴�
	height=y2-y1+1;			//楂樺害
	Lcd_SetRegion((unsigned int)x1,(unsigned int)y1,(unsigned int)x2,(unsigned int)y2);
   Lcd_WriteIndex(0x2C);
   for(i=0;i<height;i++)
    for(m=0;m<width;m++)
    {	
	  	LCD_WriteData_16Bit((unsigned int)color[i*width+m]);
    }   

}  



void TFT144lcd_fill(int32_t x1, int32_t y1, int32_t x2, int32_t y2, lv_color_t color)
{
	unsigned int i,m;
   Lcd_SetRegion((unsigned int)x1,(unsigned int)y1,(unsigned int)x2,(unsigned int)y2);
   Lcd_WriteIndex(0x2C);
   for(i=0;i<x2-x1+1;i++)
    for(m=0;m<y2-y1+1;m++)
    {	
	  	LCD_WriteData_16Bit((unsigned int)color.full);
    }   
}


void TFT144lcd_flush(int32_t x1, int32_t y1, int32_t x2, int32_t y2, const lv_color_t * color_map)
{
	TFT144LCD_Color_Fill((unsigned int)x1,(unsigned int)y1,(unsigned int)x2,(unsigned int)y2,color_map);
	lv_flush_ready();

}

/**********************
 *   STATIC FUNCTIONS
 *********************

static void TFT144lcd_send_cmd(unsigned char cmd)
{

	Lcd_WriteIndex(cmd);
	//gpio_set_level(ILI9341_DC, 0);	Command mode
	//disp_spi_send(&cmd, 1);
}

static void TFT144lcd_send_data(unsigned char * data, unsigned int length)
{
	unsigned int i = 0 ;
	if (length == 0) return;           //no need to send anything
	for(i=0;i<length;i++)
	{
		Lcd_WriteData((unsigned char)data[i]);
	}
}
*/

