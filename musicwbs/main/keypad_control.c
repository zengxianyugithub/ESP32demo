#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "freertos/queue.h"
#include "soc/gpio_struct.h"
#include "driver/gpio.h"
#include "driver/adc.h"
#include "driver/i2s.h"

#include "esp_system.h"
#include "esp_err.h"
#include "esp_log.h"
#include "../lvgl/lv_core/lv_group.h"
#include "../lvgl/lv_core/lv_indev.h"
#include "i2s_dac.h"
#include "ledc.h"

#include "keypad_control.h"
#include "esp_bt.h"
#include "bt_app_core.h"
#include "bt_app_av.h"
#include "esp_bt_main.h"
#include "esp_bt_device.h"
#include "esp_gap_bt_api.h"
#include "esp_a2dp_api.h"
#include "esp_avrc_api.h"

#include "i2s_dac.h"
#include "ui.h"
#include "keypad_control.h"
#include "mp3dec.h"


#define ADCCHANNEL ADC1_CHANNEL_6

lv_indev_t *keypad_indev;
static uint32_t last_key;
static lv_indev_state_t state;
QueueHandle_t Queue_Key;

static const char* TAG = "KEYPAD";

key_event_t keyEvent;
TickType_t key_last_tick;

#define GPIO_INPUT_IO_5     23
#define GPIO_INPUT_IO_4     27
#define GPIO_INPUT_IO_3     32
#define GPIO_INPUT_IO_2     33
#define GPIO_INPUT_IO_1     34
#define GPIO_INPUT_IO_0     35
#define GPIO_INPUT_PIN_SEL  ((1ULL<<GPIO_INPUT_IO_0) | (1ULL<<GPIO_INPUT_IO_1)| (1ULL<<GPIO_INPUT_IO_2)| (1ULL<<GPIO_INPUT_IO_3)| (1ULL<<GPIO_INPUT_IO_4)| (1ULL<<GPIO_INPUT_IO_5))
#define ESP_INTR_FLAG_DEFAULT 0

uint8_t keymode = 0 ;
TaskHandle_t keyHandle1 = NULL;

xTaskHandle sd_music_task_handle = NULL;

void taskScanKey(void *patameter) {
	int data;

	bool bleinitsta = 0 ;
	esp_err_t err = ESP_OK;
	ESP_LOGI(TAG, "backlight_timeout %d  .", backlight_timeout);

	//blesink_i2s_init();
	//ble_a2dp_sinkinit();
/*	i2s_init();
 //  blesink_i2s_init();

    player_pause(false);

    playerState.started = true;
	if(xTaskCreatePinnedToCore(taskPlay, "taskPlay", 10240, NULL, 3, &keyHandle1, tskNO_AFFINITY)== pdPASS)
	  							ESP_LOGI(TAG, "Music Player task created.");
							else ESP_LOGE(TAG, "Failed to create Player task.");

  */
	gpio_config_t io_conf;
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
    io_conf.pull_up_en = 1;
	io_conf.pull_down_en = 0 ;
    gpio_config(&io_conf);

	uint32_t io_num;
    for(;;) {
        if(gpio_get_level(GPIO_INPUT_IO_0)==0) {
			vTaskDelay(10 / portTICK_RATE_MS);
			if(gpio_get_level(GPIO_INPUT_IO_0)==0) {
            	printf("GPIO[%d] intr, val 1: \n", GPIO_INPUT_IO_0);
				keyEvent.key_name = LV_GROUP_KEY_ENTER;
				keyEvent.state = LV_INDEV_STATE_PR;
				ESP_LOGI(TAG, "key %i pressed.", keyEvent.key_name);
				last_key = keyEvent.key_name;
				state = LV_INDEV_STATE_PR;
				xQueueSend(Queue_Key, (void*)(&keyEvent), (TickType_t) 10);
				while(!gpio_get_level(GPIO_INPUT_IO_0)) vTaskDelay(10 / portTICK_RATE_MS);
				ESP_LOGI(TAG, "key %i released.", keyEvent.key_name);
				keyEvent.state = KEY_RELEASED;
				state = LV_INDEV_STATE_REL;
				xQueueSend(Queue_Key, (void*)(&keyEvent), (TickType_t) 10);
				//key_last_tick = xTaskGetTickCount();
				vTaskDelay(10 / portTICK_RATE_MS);
			}
        }

		if(gpio_get_level(GPIO_INPUT_IO_1)==0) {
			vTaskDelay(10 / portTICK_RATE_MS);
			if(gpio_get_level(GPIO_INPUT_IO_1)==0) {
            	printf("GPIO[%d] intr, val 1: \n", GPIO_INPUT_IO_1);
				keyEvent.key_name = LV_GROUP_KEY_UP;
				keyEvent.state = LV_INDEV_STATE_PR;
				ESP_LOGI(TAG, "key %i pressed.", keyEvent.key_name);
				last_key = keyEvent.key_name;
				state = LV_INDEV_STATE_PR;
				xQueueSend(Queue_Key, (void*)(&keyEvent), (TickType_t) 10);
				while(!gpio_get_level(GPIO_INPUT_IO_1)) vTaskDelay(10 / portTICK_RATE_MS);
				ESP_LOGI(TAG, "key %i released.", keyEvent.key_name);
				keyEvent.state = KEY_RELEASED;
				state = LV_INDEV_STATE_REL;
				xQueueSend(Queue_Key, (void*)(&keyEvent), (TickType_t) 10);
				//key_last_tick = xTaskGetTickCount();
				vTaskDelay(10 / portTICK_RATE_MS);
				}
        }
		if(gpio_get_level(GPIO_INPUT_IO_2)==0) {
			vTaskDelay(10 / portTICK_RATE_MS);
			if(gpio_get_level((GPIO_INPUT_IO_2))==0) {
            	printf("GPIO[%d] intr, val 1: \n", (GPIO_INPUT_IO_2));
				keyEvent.key_name = LV_GROUP_KEY_DOWN;
				keyEvent.state = LV_INDEV_STATE_PR;
				ESP_LOGI(TAG, "key %i pressed.", keyEvent.key_name);
				last_key = keyEvent.key_name;
				state = LV_INDEV_STATE_PR;
				xQueueSend(Queue_Key, (void*)(&keyEvent), (TickType_t) 10);
				while(!gpio_get_level((GPIO_INPUT_IO_2))) vTaskDelay(10 / portTICK_RATE_MS);
				ESP_LOGI(TAG, "key %i released.", keyEvent.key_name);
				keyEvent.state = KEY_RELEASED;
				state = LV_INDEV_STATE_REL;
				xQueueSend(Queue_Key, (void*)(&keyEvent), (TickType_t) 10);
				//key_last_tick = xTaskGetTickCount();
				vTaskDelay(10 / portTICK_RATE_MS);
				}
        }
		/*
		if(gpio_get_level(GPIO_INPUT_IO_3)==1) {
			vTaskDelay(10 / portTICK_RATE_MS);
			if(gpio_get_level((GPIO_INPUT_IO_3))==1) {
            	printf("GPIO[%d] intr, val 1: \n", (GPIO_INPUT_IO_3));
				keyEvent.key_name = LV_GROUP_KEY_PREV;
				keyEvent.state = LV_INDEV_STATE_PR;
				ESP_LOGI(TAG, "key %i pressed.", keyEvent.key_name);
				last_key = keyEvent.key_name;
				state = LV_INDEV_STATE_PR;
				xQueueSend(Queue_Key, (void*)(&keyEvent), (TickType_t) 10);
				while(gpio_get_level((GPIO_INPUT_IO_3))) vTaskDelay(10 / portTICK_RATE_MS);
				ESP_LOGI(TAG, "key %i released.", keyEvent.key_name);
				keyEvent.state = KEY_RELEASED;
				state = LV_INDEV_STATE_REL;
				xQueueSend(Queue_Key, (void*)(&keyEvent), (TickType_t) 10);
				//key_last_tick = xTaskGetTickCount();
				vTaskDelay(10 / portTICK_RATE_MS);
				}
        }
		if(gpio_get_level(GPIO_INPUT_IO_4)==1) {
			vTaskDelay(10 / portTICK_RATE_MS);
			if(gpio_get_level(((GPIO_INPUT_IO_4)))==1) {
            	printf("GPIO[%d] intr, val 1: \n", ((GPIO_INPUT_IO_4)));
				keyEvent.key_name = LV_GROUP_KEY_NEXT;
				keyEvent.state = LV_INDEV_STATE_PR;
				ESP_LOGI(TAG, "key %i pressed.", keyEvent.key_name);
				last_key = keyEvent.key_name;
				state = LV_INDEV_STATE_PR;
				xQueueSend(Queue_Key, (void*)(&keyEvent), (TickType_t) 10);
				while(gpio_get_level(((GPIO_INPUT_IO_4)))) vTaskDelay(10 / portTICK_RATE_MS);
				ESP_LOGI(TAG, "key %i released.", keyEvent.key_name);
				keyEvent.state = KEY_RELEASED;
				state = LV_INDEV_STATE_REL;
				xQueueSend(Queue_Key, (void*)(&keyEvent), (TickType_t) 10);
				//key_last_tick = xTaskGetTickCount();
				vTaskDelay(10 / portTICK_RATE_MS);
				}
        }
        */
        if(gpio_get_level(GPIO_INPUT_IO_3)==0) {
			vTaskDelay(10 / portTICK_RATE_MS);
			if(gpio_get_level(GPIO_INPUT_IO_3)==0) {//32
            	printf("GPIO[%d] intr, val 1: \n", GPIO_INPUT_IO_3);


				
				
				keymode ++ ;
				if(keymode >7)keymode = 0 ;
				ESP_LOGE(TAG, "   ***=========keymode = %d*****",keymode);
				
				while(!gpio_get_level(GPIO_INPUT_IO_3)) vTaskDelay(10 / portTICK_RATE_MS);
				
				
				
				//ESP_LOGE(TAG, " consta bleconsta ***%d*****",bleconsta);
				printf("GPIO[%d] intr, val 0: \n", GPIO_INPUT_IO_3);
				vTaskDelay(10 / portTICK_RATE_MS);
			}
        }
		if(gpio_get_level(GPIO_INPUT_IO_4)==0) {
			vTaskDelay(10 / portTICK_RATE_MS);
			if(gpio_get_level(GPIO_INPUT_IO_4)==0) {//27
            	printf("GPIO[%d] intr, val 1: \n", GPIO_INPUT_IO_4);
				
				ESP_LOGE(TAG, "   ***=========keymode = %d*****",keymode);
				switch(keymode)
				{
					case 0 : //playerState.started = true;
						
					break;

					case 1 : //playerState.started = false;
						if(save_musicmode(1)==ESP_OK)
						{
							ESP_LOGI(TAG, "save_musicmode ok sd.");
							if(playerState.musicmode_1 != 1)
							{
								esp_restart();   
							}
						}
					break;

					case 2 :// playerState.paused = false;
						if(save_musicmode(2)==ESP_OK)
						{
							ESP_LOGI(TAG, "save_musicmode ok ble.");
							if(playerState.musicmode_1 != 2)
							{
								esp_restart();   
							}
						}
					break;

					case 3 ://playerState.paused = true;
						if(save_musicmode(3)==ESP_OK)
						{
							ESP_LOGI(TAG, "save_musicmode ok wifi.");
							if(playerState.musicmode_1 != 3)
							{
								esp_restart();   
							}
						}
					break;

					case 4 :
						//i2s_init();
						playerState.started = true;
						if( xTaskCreate(taskPlay, "taskPlay", 10240, NULL, 3, sd_music_task_handle) ==pdPASS)
							ESP_LOGI(TAG, "file_task_example created.");

						   else ESP_LOGE(TAG, "Failed to create file_task_example.");
						
					break;

					case 5 :
							//playerState.started = false;
							if( keyHandle1 != NULL )
						    {
						     vTaskDelete( keyHandle1 );
							 ESP_LOGI(TAG, "delete taskplay -----------------.");

							if ((err = i2s_driver_uninstall(0)) != ESP_OK) {
        						ESP_LOGE(TAG, "=======i2s_driver_uninstall failed=======");

							}
							 
						  }
					break;

					case 6 :
						/*if(!bleinitsta)
							{
								blesink_i2s_init();
								ble_a2dp_sinkinit();
								bleinitsta =1 ;
							}
							else
							{*/
								//blesink_i2s_init();
								esp_bt_gap_set_scan_mode(ESP_BT_CONNECTABLE, ESP_BT_GENERAL_DISCOVERABLE);
							//}
							ESP_LOGE(TAG, "=======bleinitsta=%d======",bleinitsta);
					break;

					case 7 : if ((err = esp_a2d_sink_disconnect(ble1conadr )) != ESP_OK) {
        							ESP_LOGE(TAG, "%s disconnect [%02x:%02x:%02x:%02x:%02x:%02x] failed: %s\n", __func__,ble1conadr[0], ble1conadr[1], ble1conadr[2], ble1conadr[3], ble1conadr[4], ble1conadr[5], esp_err_to_name(err));
    							}
								vTaskDelay(1000 / portTICK_RATE_MS);
								if ((err = esp_bt_gap_set_scan_mode(ESP_BT_NON_CONNECTABLE, ESP_BT_NON_DISCOVERABLE)) != ESP_OK) {
        							ESP_LOGE(TAG, "=======close ble failed=======");

								}
								if ((err = i2s_driver_uninstall(0)) != ESP_OK) {
        							ESP_LOGE(TAG, "=======i2s_driver_uninstall failed=======");

								}
								
								
					break;
					
							
					default:break;
				}
				
				while(gpio_get_level(GPIO_INPUT_IO_4)) vTaskDelay(10 / portTICK_RATE_MS);
				
				
				ESP_LOGE(TAG, " consta bleconsta ***%d*****",bleconsta);
				printf("GPIO[%d] intr, val 0: \n", GPIO_INPUT_IO_4);
				vTaskDelay(10 / portTICK_RATE_MS);
			}
        }
		if(gpio_get_level(GPIO_INPUT_IO_5)==0) {
			vTaskDelay(10 / portTICK_RATE_MS);
			if(gpio_get_level((((GPIO_INPUT_IO_5))))==0) {
            	printf("GPIO[%d] intr, val 1: \n", (((GPIO_INPUT_IO_5))));
				keyEvent.key_name = LV_GROUP_KEY_ESC;
				keyEvent.state = LV_INDEV_STATE_PR;
				ESP_LOGI(TAG, "key %i pressed.", keyEvent.key_name);
				last_key = keyEvent.key_name;
				state = LV_INDEV_STATE_PR;
				xQueueSend(Queue_Key, (void*)(&keyEvent), (TickType_t) 10);
				while(!gpio_get_level((((GPIO_INPUT_IO_5))))) vTaskDelay(10 / portTICK_RATE_MS);
				ESP_LOGI(TAG, "key %i released.", keyEvent.key_name);
				keyEvent.state = KEY_RELEASED;
				state = LV_INDEV_STATE_REL;
				xQueueSend(Queue_Key, (void*)(&keyEvent), (TickType_t) 10);
				//key_last_tick = xTaskGetTickCount();
				vTaskDelay(10 / portTICK_RATE_MS);
				}
        }
		vTaskDelay(10 / portTICK_RATE_MS);
    }


	/*
	while(1) {
		adc1_config_width(ADC_WIDTH_BIT_12);
		adc1_config_channel_atten(ADC1_CHANNEL_3,ADC_ATTEN_DB_6);
		data = adc1_get_raw(ADC1_CHANNEL_3);
		data = (int)((double)data / 4096.0 * 2200);

		if(data > 0 && data < 225) keyEvent.key_name = LV_GROUP_KEY_NEXT;
		else if(data >= 225 && data < 476) keyEvent.key_name = LV_GROUP_KEY_UP;
		else if(data >= 476 && data < 700) keyEvent.key_name = LV_GROUP_KEY_PREV;
		else if(data >= 700 && data < 952) {
			player_pause(!isPaused());
			while(adc1_get_raw(ADC1_CHANNEL_3) != 0) vTaskDelay(10 / portTICK_RATE_MS);
			key_last_tick = xTaskGetTickCount();
			continue;
		}
		else if(data >= 952 && data < 1296) keyEvent.key_name = LV_GROUP_KEY_DOWN;
		else if(data >= 1296 && data < 1673) keyEvent.key_name = LV_GROUP_KEY_ENTER;
		else if(data >= 1673) keyEvent.key_name = LV_GROUP_KEY_ESC;
		else {
			vTaskDelay(10 / portTICK_RATE_MS);
			continue;
		}
		
		vTaskDelay(1000 / portTICK_RATE_MS);
		
		if(xTaskGetTickCount() - key_last_tick < backlight_timeout || backlight_timeout == 0) {
			keyEvent.state = LV_INDEV_STATE_PR;
			ESP_LOGI(TAG, "key %i pressed.", keyEvent.key_name);
			last_key = keyEvent.key_name;
			state = LV_INDEV_STATE_PR;
			xQueueSend(Queue_Key, (void*)(&keyEvent), (TickType_t) 10);
			while(adc1_get_raw(ADC1_CHANNEL_3) != 0) vTaskDelay(10 / portTICK_RATE_MS);
			ESP_LOGI(TAG, "key %i released.", keyEvent.key_name);
			keyEvent.state = KEY_RELEASED;
			state = LV_INDEV_STATE_REL;
			xQueueSend(Queue_Key, (void*)(&keyEvent), (TickType_t) 10);
			key_last_tick = xTaskGetTickCount();
			vTaskDelay(10 / portTICK_RATE_MS);
		}
		key_last_tick = xTaskGetTickCount();
	}
	*/

}


esp_err_t keyQueueCreate() {
	key_last_tick = xTaskGetTickCount();
	Queue_Key = xQueueCreate(16, sizeof(key_event_t));
	if(Queue_Key == 0) return ESP_FAIL;

	return ESP_OK;
}

bool keypad_read(lv_indev_data_t *data) {
	key_event_t key_event;
	if(xQueueReceive(Queue_Key, &key_event, 10) == pdPASS) {
		data->state = key_event.state;
		data->key = key_event.key_name;
	} else {
		data->state = state;
		data->key = last_key;
	}

	return false;
}


esp_err_t save_musicmode(int8_t nvs_i8)
{
    nvs_handle my_handle;
    esp_err_t err;

    // Open
    err = nvs_open(STORAGE_NAMESPACE, NVS_READWRITE, &my_handle);
    if (err != ESP_OK) return err;

    // Read
    //int32_t restart_counter = 0; // value will default to 0, if not set yet in NVS
    //err = nvs_get_i32(my_handle, "mywifissid", &restart_counter);
   //if (err != ESP_OK && err != ESP_ERR_NVS_NOT_FOUND) return err;

    // Write
    //restart_counter++;
    err = nvs_set_i8(my_handle, MUSICMODESTR, nvs_i8);
    if (err != ESP_OK) return err;

    // Commit written value.
    // After setting any values, nvs_commit() must be called to ensure changes are written
    // to flash storage. Implementations may write to storage at other times,
    // but this is not guaranteed.
    err = nvs_commit(my_handle);
    if (err != ESP_OK) return err;

    // Close
    nvs_close(my_handle);
    return ESP_OK;
}


void get_musicmode()
{
	nvs_handle my_handle1; 

	esp_err_t err;

			
	err = nvs_open(STORAGE_NAMESPACE, NVS_READWRITE, &my_handle1);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "opening NVS Error (%s)!\n", esp_err_to_name(err));
    } else {
		ESP_LOGI(TAG, "NVS open OK");

		 err = nvs_get_i8(my_handle1, MUSICMODESTR, &playerState.musicmode_1);
		
		if(err==ESP_OK)
		{
			ESP_LOGI(TAG,"playerState.musicmode_1 = (%d)  !\n", playerState.musicmode_1);
		}
		else
		{
			
			ESP_LOGI(TAG,"Error (%s) reading!\n", esp_err_to_name(err));
		}

		
    }
	nvs_close(my_handle1);
}



