/* ADC1 Example

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "demo.h"
	
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "freertos/queue.h"
#include "esp_freertos_hooks.h"
	
#include "esp_log.h"
	
#include "driver/gpio.h"
//#include "Lcd_Driver.h"
//#include "GUI.h"
#include "LcdTFT22_Driver.h"
#include "../lvgl/lvgl.h"

static const char* TAG = "LVGL";

lv_obj_t *title, *txt;

void vgltest( char mode)
{

	printf("testvlgstasrt  mode =%d \r\n",mode);

	if(mode)
	{
	//screen1 = lv_label_create(lv_scr_act(), NULL);
	title = lv_label_create(lv_scr_act(), NULL);
	lv_label_set_text(title, "Title Label");
	lv_obj_align(title, NULL, LV_ALIGN_IN_TOP_RIGHT, 0, 0);  /*Align to the top*/

	/*Create anew style*/
	static lv_style_t style_txt;
	lv_style_copy(&style_txt, &lv_style_plain);
	style_txt.text.font = &lv_font_dejavu_20;
	style_txt.text.letter_space = 2;
	style_txt.text.line_space = 1;
	style_txt.text.color = LV_COLOR_HEX(YELLOW);


	/*Create a new label*/
	txt = lv_label_create(lv_scr_act(), NULL);
	//lv_obj_set_style(title, &style_txt);  
	lv_obj_set_style(txt, &style_txt);                    /*Set the created style*/
	lv_label_set_long_mode(txt, LV_LABEL_LONG_BREAK);     /*Break the long lines*/
	lv_label_set_recolor(txt, true);                      /*Enable re-coloring by commands in the text*/
	lv_label_set_align(txt, LV_LABEL_ALIGN_CENTER);       /*Center aligned lines*/
    //lv_label_set_align(title, LV_ALIGN_IN_TOP_LEFT);  
	lv_label_set_text(txt, "Align\n"
	                       "ext\n"
	                       "AA");
	lv_obj_set_width(txt, 64);                           /*Set a width*/

	lv_obj_align(txt, NULL, LV_ALIGN_CENTER, 0, 0);      /*Align to center*/
	}
	else
	{
		lv_obj_clean(title);
		lv_obj_clean(txt);
		lv_obj_del(txt);
		lv_obj_del(title);
		//group = lv_group_create();
		//lv_indev_set_group(keypad_indev, group);
		//lv_group_set_style_mod_cb(group, style_mod_cb);
	}
	printf("testvlgsend\r\n");
}

// Task to be created.
	void vTaskCode( void * pvParameters )
	{
		unsigned int cnt1 = 0 ;
	 for( ;; )
	 {
	     
		 if(cnt1%2==0)
		 {
			vgltest(1);
		 }
		 else
		 {
			vgltest(0);
		 }
		 cnt1++;
		 vTaskDelay(2000 / portTICK_RATE_MS);
		 // Task code goes here.
	 }
	}
	
	// Function that creates a task.
	void vOtherFunction( void )
	{
	static uint8_t ucParameterToPass;
	TaskHandle_t xHandle = NULL;
	
	 // Create the task, storing the handle.  Note that the passed parameter ucParameterToPass
	 // must exist for the lifetime of the task, so in this case is declared static.  If it was just an
	 // an automatic stack variable it might no longer exist, or at least have been corrupted, by the time
	 // the new task attempts to access it.
	if(xTaskCreate( vTaskCode, "NAME1", 8000, &ucParameterToPass, tskIDLE_PRIORITY|3, &xHandle ) == pdPASS)
    	ESP_LOGI(TAG, "NAME1 task created.");
  	else ESP_LOGE(TAG, "Failed to create NAME1 task.");
	 
		configASSERT( xHandle );
	
	 // Use the handle to delete the task.
		//if( xHandle != NULL )
		//{
		// vTaskDelete( xHandle );
		//}
	}


//static void lv_tick_task(void);

static void lv_tick_task(void) {
  lv_tick_inc(portTICK_RATE_MS);
}

void a123app1_main111()
{


	char arr[] =  "   STM32 LED    ";
	
	Lcd_Init();	 //液晶屏--初始化配置
	Lcd_Clear(GRAY1);//清屏
	//testfill();
	//Gui_Circle(64,64,20,GRAY1);
	// Gui_DrawFont_GBK16(0,16,RED,GRAY0,(unsigned char *)arr);
   //Gui_DrawFont_GBK16(0,32,GREEN,GRAY0,(unsigned char *)"ceshi");	 
	//Gui_DrawFont_GBK16(0,48,BLUE,GRAY0,(unsigned char *)"mcudev.taobao.com"); 
	vTaskDelay(1000 / portTICK_RATE_MS);
	Lcd_Clear(GRAY0);//清屏
	//littlevgl init
  	lv_init();
  	//disp_spi_init();
  	//ili9431_init();
  lv_disp_drv_t disp;
  lv_disp_drv_init(&disp);
  disp.disp_flush = TFT22lcd_flush;
  disp.disp_fill = TFT22LCD_Color_fill;
  //disp.
  //disp.disp_map = 
  lv_disp_drv_register(&disp);
	
	 
	vTaskDelay(50 / portTICK_RATE_MS);
	
  	//vOtherFunction ();
    unsigned int cnt = 0;
	demo_create();
	esp_register_freertos_tick_hook(lv_tick_task);
	 
    while(1) {
        //printf("cnt: %d\n", cnt++);
        vTaskDelay(5 / portTICK_RATE_MS);
		//lv_tick_inc(10);
		lv_task_handler();

		cnt++;
		if(cnt==500)

		{
			printf("cnt: %d\n", cnt++);
			//printf("lv_task_run: %d\n", lv_task_run);
			
			//vgltest(); 
		}
        //gpio_set_level(GPIO_OUTPUT_IO_0, cnt % 2);
        //gpio_set_level(GPIO_OUTPUT_IO_1, cnt % 2);
    }//
}

