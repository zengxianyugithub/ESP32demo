#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "freertos/queue.h"
#include "soc/gpio_struct.h"
#include "driver/gpio.h"
#include "driver/adc.h"
#include "esp_system.h"
#include "esp_err.h"
#include "esp_log.h"

#include "esp_bt.h"
#include "bt_app_core.h"
#include "bt_app_av.h"
#include "esp_bt_main.h"
#include "esp_bt_device.h"
#include "esp_gap_bt_api.h"
#include "esp_a2dp_api.h"
#include "esp_avrc_api.h"
#include "driver/i2s.h"
#include "keypad_control.h"


QueueHandle_t Queue_Key;

static const char* TAG = "KEYPAD";


TickType_t key_last_tick;

#define GPIO_INPUT_IO_5     23
#define GPIO_INPUT_IO_4     27
#define GPIO_INPUT_IO_3     32
#define GPIO_INPUT_IO_2     33
#define GPIO_INPUT_IO_1     34
#define GPIO_INPUT_IO_0     35
#define GPIO_INPUT_PIN_SEL  ((1ULL<<GPIO_INPUT_IO_0) | (1ULL<<GPIO_INPUT_IO_1)| (1ULL<<GPIO_INPUT_IO_2)| (1ULL<<GPIO_INPUT_IO_3)| (1ULL<<GPIO_INPUT_IO_4)| (1ULL<<GPIO_INPUT_IO_5))
#define ESP_INTR_FLAG_DEFAULT 0

uint8_t ble1conadrsave[6] = {} ;




void taskScanKey(void *patameter) {
	int data;
	ESP_LOGI(TAG, "ENTER SCAN KEY");
	gpio_config_t io_conf;
	esp_err_t err = ESP_OK;
    /*//disable interrupt
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
*/
    //interrupt of rising edge        		    GPIO_PIN_INTR_POSEDGE
    io_conf.intr_type =GPIO_PIN_INTR_DISABLE ;
    //bit mask of the pins, use GPIO4/5 here
    io_conf.pin_bit_mask = GPIO_INPUT_PIN_SEL;
    //set as input mode    
    io_conf.mode = GPIO_MODE_INPUT;
    //enable pull-up mode
    io_conf.pull_up_en = 0;
	io_conf.pull_down_en = 1 ;
    gpio_config(&io_conf);

	uint32_t io_num;
    for(;;) {
        if(gpio_get_level(GPIO_INPUT_IO_0)==1) {
			vTaskDelay(10 / portTICK_RATE_MS);
			if(gpio_get_level(GPIO_INPUT_IO_0)==1) {//35
            	printf("GPIO[%d] intr, val 1: \n", GPIO_INPUT_IO_0);
				periph_bluetooth_next();
				
				while(gpio_get_level(GPIO_INPUT_IO_0)) vTaskDelay(10 / portTICK_RATE_MS);
				printf("GPIO[%d] intr, val 0: \n", GPIO_INPUT_IO_0);
				vTaskDelay(10 / portTICK_RATE_MS);
			}
        }
        if(gpio_get_level(GPIO_INPUT_IO_1)==1) {
			vTaskDelay(10 / portTICK_RATE_MS);
			if(gpio_get_level(GPIO_INPUT_IO_1)==1) {//34
            	printf("GPIO[%d] intr, val 1: \n", GPIO_INPUT_IO_1);
				
				periph_bluetooth_play();
				while(gpio_get_level(GPIO_INPUT_IO_1)) vTaskDelay(10 / portTICK_RATE_MS);
			
				printf("GPIO[%d] intr, val 0: \n", GPIO_INPUT_IO_1);
				vTaskDelay(10 / portTICK_RATE_MS);
			}
        }
        if(gpio_get_level(GPIO_INPUT_IO_2)==1) {
			vTaskDelay(10 / portTICK_RATE_MS);
			if(gpio_get_level(GPIO_INPUT_IO_2)==1) {//33
            	printf("GPIO[%d] intr, val 1: \n", GPIO_INPUT_IO_2);
				//periph_bluetooth_pause();
				while(gpio_get_level(GPIO_INPUT_IO_2)) vTaskDelay(10 / portTICK_RATE_MS);
				printf("/**************************start**************************/\r\n");

				if ((err = esp_bt_gap_set_scan_mode(ESP_BT_CONNECTABLE, ESP_BT_GENERAL_DISCOVERABLE)) != ESP_OK) {
        				ESP_LOGE(TAG, "=======open ble failed=======");

				}
				 bt_app_task_start_up();
				// 
				//printf("%s\r\n",InfoBuffer);					//通过串口打印所有任务的信息
				printf("/**************************end**************************/\r\n");
				
				printf("GPIO[%d] intr, val 0: \n", GPIO_INPUT_IO_2);
				vTaskDelay(10 / portTICK_RATE_MS);
			}
        }
        if(gpio_get_level(GPIO_INPUT_IO_3)==1) {
			vTaskDelay(10 / portTICK_RATE_MS);
			if(gpio_get_level(GPIO_INPUT_IO_3)==1) {//32
            	printf("GPIO[%d] intr, val 1: \n", GPIO_INPUT_IO_3);
				//periph_bluetooth_stop();
				//bt_app_task_start_up();

				ESP_LOGE(TAG, " consta bleconsta ***%d*****",bleconsta);
				
				while(gpio_get_level(GPIO_INPUT_IO_3)) vTaskDelay(10 / portTICK_RATE_MS);
				
				
				if ((err = esp_bt_gap_set_scan_mode(ESP_BT_NON_CONNECTABLE, ESP_BT_NON_DISCOVERABLE)) != ESP_OK) {
        				ESP_LOGE(TAG, "=======close ble failed=======");

				}
				

				//esp_bluedroid_disable();
 				//esp_bluedroid_deinit();
 				//esp_bt_controller_disable();
 				//esp_bt_controller_deinit();
 				//esp_bt_mem_release(ESP_BT_MODE_BLE);
				
				bt_app_task_shut_down();
				
				ESP_LOGE(TAG, " consta bleconsta ***%d*****",bleconsta);
				printf("GPIO[%d] intr, val 0: \n", GPIO_INPUT_IO_3);
				vTaskDelay(10 / portTICK_RATE_MS);
			}
        }
		if(gpio_get_level(GPIO_INPUT_IO_4)==1) {
			vTaskDelay(10 / portTICK_RATE_MS);
			if(gpio_get_level(GPIO_INPUT_IO_4)==1) {//27
            	printf("GPIO[%d] intr, val 1: \n", GPIO_INPUT_IO_4);
				//periph_bluetooth_prev();
				//bt_app_task_shut_down();
				ESP_LOGE(TAG, " consta bleconsta ***%d*****",bleconsta);

				//char InfoBuffer[1000];				//保存信息的数组
				//vTaskList(InfoBuffer);							//获取所有任务的信息
				
				while(gpio_get_level(GPIO_INPUT_IO_4)) vTaskDelay(10 / portTICK_RATE_MS);
				
				ble1conadrsave[0] = 0xb0;
				ble1conadrsave[1] = 0x55 ;
				ble1conadrsave[2] = 0x08 ;
				ble1conadrsave[3] = 0xb2 ;
				ble1conadrsave[4] = 0x07 ;
				ble1conadrsave[5] = 0x9c ;
						
				ESP_LOGE(TAG, "---ble1conadrsave [%02x:%02x:%02x:%02x:%02x:%02x] ",  ble1conadrsave[0], ble1conadrsave[1], ble1conadrsave[2], ble1conadrsave[3], ble1conadrsave[4], ble1conadrsave[5] );
					
				
				if ((err = esp_a2d_sink_disconnect(ble1conadr )) != ESP_OK) {
        			ESP_LOGE(TAG, "%s disconnect [%02x:%02x:%02x:%02x:%02x:%02x] failed: %s\n", __func__,ble1conadr[0], ble1conadr[1], ble1conadr[2], ble1conadr[3], ble1conadr[4], ble1conadr[5], esp_err_to_name(err));
    			}
				ESP_LOGE(TAG, " consta bleconsta ***%d*****",bleconsta);
				printf("GPIO[%d] intr, val 0: \n", GPIO_INPUT_IO_4);
				vTaskDelay(10 / portTICK_RATE_MS);
			}
        }
		
		vTaskDelay(10 / portTICK_RATE_MS);
    }


	

}


