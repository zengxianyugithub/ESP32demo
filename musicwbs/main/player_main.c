#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "freertos/queue.h"

#include "esp_system.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_event_loop.h"
#include "esp_err.h"
#include "esp_log.h"
#include "esp_spiffs.h"
#include "esp_vfs.h"
#include "nvs_flash.h"
#include "soc/gpio_struct.h"
#include "driver/gpio.h"
#include "esp_freertos_hooks.h"

#include "esp_bt.h"
#include "bt_app_core.h"
#include "bt_app_av.h"
#include "esp_bt_main.h"
#include "esp_bt_device.h"
#include "esp_gap_bt_api.h"
#include "esp_a2dp_api.h"
#include "esp_avrc_api.h"

#include "driver/i2s.h"
#include "../lvgl/lvgl.h"

//#include "LcdTFT22_Driver.h"
#include "lcdTFT32.h"

#include "sd_card.h"
#include "dirent.h"
#include "i2s_dac.h"
#include "ui.h"
#include "keypad_control.h"
#include "mp3dec.h"
#include "ledc.h"
#include "demo.h"
#include "tcp_client.h"

static EventGroupHandle_t wifi_event_group;
const int WIFI_CONNECTED_BIT = BIT0;

static const char *TAG = "APP_MAIN";
static esp_err_t wifi_event_handler(void *ctx, system_event_t *event);
static void lv_tick_task(void);

typedef FILE* pc_file_t;
static lv_fs_res_t pcfs_open(void * file_p, const char * fn, lv_fs_mode_t mode);
static lv_fs_res_t pcfs_close(void * file_p);
static lv_fs_res_t pcfs_read(void * file_p, void * buf, uint32_t btr, uint32_t * br);
static lv_fs_res_t pcfs_seek(void * file_p, uint32_t pos);
static lv_fs_res_t pcfs_tell(void * file_p, uint32_t * pos_p);

TaskHandle_t keyHandle = NULL;
TaskHandle_t playerHandle = NULL;
TaskHandle_t uiHandle = NULL;
TaskHandle_t batHandle = NULL;

sdmmc_card_t card;






void gpio_task_example(void* arg)
{
     
	FILE *filePtr;
	const char *mp3basePath = "/sdcard/MUSIC/Beyd.mp3";
	filePtr = fopen(mp3basePath, "rb");

	if(filePtr == NULL) {
	   ESP_LOGE(TAG, "Failed to mp3file: %s", mp3basePath);
		
        for(;;) {

	
		vTaskDelay(1000 / portTICK_RATE_MS);
		ESP_LOGI(TAG, "****test*****");
		//mp3Play(FILE *mp3File);
        }
   }
	
   ESP_LOGE(TAG, "success to mp3file: %s", mp3basePath);
	mp3Play1(filePtr);
	 
   
    
}


void file_task_example(void* arg)
{
     
		
    ESP_LOGI(TAG, "Opening file");
    //FILE* f = fopen("/sdcard/hello.txt", "w");
	FILE* f = fopen("/sdcard/list3.db", "w");
    if (f == NULL) {
        ESP_LOGE(TAG, "Failed to open file for writing");
        //return;
        vTaskDelete( NULL );
    }
	else
	{
		ESP_LOGE(TAG, "Failed to open file for writing success");
	}

	
    fprintf(f, "Hello %s!\n", "file");
    fclose(f);
    ESP_LOGI(TAG, "File written");

	vTaskDelay(1000 / portTICK_RATE_MS);

	
     for(;;) {

		vTaskDelay(1000 / portTICK_RATE_MS);
		
		ESP_LOGI(TAG, "****test*****");
		//mp3Play(FILE *mp3File);

	 }
   
	
   
   
    
}



void task_test(void *patameter) {
	
	
	playerState.musicmode_1 = 3 ;
	
	get_musicmode();
	if(playerState.musicmode_1 == 1 )
	{
		ESP_LOGI(TAG,"11111111111111111111111111111111111111111111111111\n");
		player_pause(false);
		
		if( xTaskCreate(taskPlay, "taskPlay", 10240, NULL, 3, sd_music_task_handle) ==pdPASS)
			ESP_LOGI(TAG, "file_task_example created.");

		else ESP_LOGE(TAG, "Failed to create file_task_example.");
		
	}
	else if(playerState.musicmode_1 == 2)
	{
		 //  
		//蓝牙初始化
		ESP_LOGI(TAG,"222222222222222222222222222222222222222222222222222222\n");
		ble_a2dp_sinkinit();

	}
	else if(playerState.musicmode_1 == 3)
	{
		   
		 ESP_LOGI(TAG,"33333333333333333333333333333\n");
		mywifiinit1();

	}
	
	

 /*   
  if(xTaskCreatePinnedToCore(taskPlay, "taskPlay", 10240, NULL, 3, NULL, tskNO_AFFINITY)== pdPASS)
	  ESP_LOGI(TAG, "Music Player task created.");
	else ESP_LOGE(TAG, "Failed to create Player task.");

  if(xTaskCreatePinnedToCore(taskPlay,"Player",10000,NULL,(portPRIVILEGE_BIT | 3),&uiHandle,1) == pdPASS)
    ESP_LOGI(TAG, "Music Player task created.");
  else ESP_LOGE(TAG, "Failed to create Player task.");
*/
  


	while(1)
	{
		
		
		vTaskDelay(2000 / portTICK_RATE_MS);
		
		ESP_LOGI("== ==", "[APP] Free memory: %d bytes", esp_get_free_heap_size());
	}
	
}		




void app_main() {
  esp_err_t ret;
  
  esp_err_t err = nvs_flash_init();
	 if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
		 ESP_ERROR_CHECK(nvs_flash_erase());
		 err = nvs_flash_init();
	 }
	 ESP_ERROR_CHECK(err);
  /*
  gpio_set_direction(PIN_PD, GPIO_MODE_OUTPUT);
  gpio_set_level(PIN_PD, 1);

   
  //spiffs mount
  esp_vfs_spiffs_conf_t conf = {
    .base_path = "/spiffs",
    .partition_label = NULL,
    .max_files = 5,
    .format_if_mount_failed = false
  };

  // Use settings defined above to initialize and mount SPIFFS filesystem.
  // Note: esp_vfs_spiffs_register is an all-in-one convenience function.
  ret = esp_vfs_spiffs_register(&conf);

  if (ret != ESP_OK) {
    if (ret == ESP_FAIL) {
        ESP_LOGE(TAG, "Failed to mount or format filesystem");
    } else if (ret == ESP_ERR_NOT_FOUND) {
        ESP_LOGE(TAG, "Failed to find SPIFFS partition");
    } else {
        ESP_LOGE(TAG, "Failed to initialize SPIFFS (%d)", ret);
    }
    return;
  }

  

  uint32_t tot=0, used=0;
  esp_spiffs_info(NULL, &tot, &used);
  ESP_LOGI(TAG, "SPIFFS: free %d KB of %d KB\n", (tot-used) / 1024, tot / 1024);
  
  ESP_LOGI(TAG, "Mount spiffs succeeded.");
  */
  //sdcard init
  sdmmc_mount(&card);
/***********************/
  //littlevgl init
  
  #if 0
  lv_init();
  LCD_Init();
//////  Lcd_Init();	 //液晶屏--初始化配置
////	Lcd_Clear(GRAY1);//清屏
//lcdspi_init();
	//testfill();
	//Gui_Circle(64,64,20,GRAY1);
	// Gui_DrawFont_GBK16(0,16,RED,GRAY0,(unsigned char *)arr);
   //Gui_DrawFont_GBK16(0,32,GREEN,GRAY0,(unsigned char *)"ceshi");	 
	//Gui_DrawFont_GBK16(0,48,BLUE,GRAY0,(unsigned char *)"mcudev.taobao.com"); 
	//vTaskDelay(1000 / portTICK_RATE_MS);
	//Lcd_Clear(RED);//清屏
	 
  lv_disp_drv_t disp;
  lv_disp_drv_init(&disp);
  ////  disp.disp_flush = TFT22lcd_flush;
disp.disp_flush = TFT32lcd_flush;
  lv_disp_drv_register(&disp);
  /************/
	vTaskDelay(50 / portTICK_RATE_MS);
	//demo_create();
  /* *//*


  lv_fs_drv_t pcfs_drv;
  memset(&pcfs_drv, 0, sizeof(lv_fs_drv_t));
  pcfs_drv.file_size = sizeof(pc_file_t);       
  pcfs_drv.letter = 'S';
  pcfs_drv.open = pcfs_open;
  pcfs_drv.close = pcfs_close;
  pcfs_drv.read = pcfs_read;
  pcfs_drv.seek = pcfs_seek;
  pcfs_drv.tell = pcfs_tell;
  lv_fs_add_drv(&pcfs_drv);
	
	*/
	/***********************/
  //set up littlevgl input device
  lv_indev_drv_t indev_drv;
  lv_indev_drv_init(&indev_drv);
  indev_drv.type = LV_INDEV_TYPE_KEYPAD;
  indev_drv.read = keypad_read;
  keypad_indev = lv_indev_drv_register(&indev_drv);
  /***********************/
 /*  *//*

  //WiFi Init
  nvs_flash_init();
  tcpip_adapter_init();
  tcpip_adapter_set_hostname(TCPIP_ADAPTER_IF_STA, "mPlayer");
  wifi_event_group = xEventGroupCreate();
  ESP_ERROR_CHECK(esp_event_loop_init(wifi_event_handler, NULL));
  wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
  ESP_ERROR_CHECK( esp_wifi_init(&cfg) );
  ESP_ERROR_CHECK( esp_wifi_set_storage(WIFI_STORAGE_RAM) );
  ESP_ERROR_CHECK( esp_wifi_set_mode(WIFI_MODE_STA) );
  ESP_ERROR_CHECK( esp_wifi_set_auto_connect(1) );
  wifi_config_t sta_config = {
      .sta = {
          .ssid = "HH",
          .password = "12345678",
          .bssid_set = false
      }
  };
  ESP_ERROR_CHECK( esp_wifi_set_config(WIFI_IF_STA, &sta_config) );
  ESP_ERROR_CHECK( esp_wifi_start() );
  ESP_ERROR_CHECK( esp_wifi_connect() );
*/

 //////////////////////////////////////////////////////////////////////////////
	if(xTaskCreatePinnedToCore(taskUI_Char, "taskUI_Char", 6144, NULL, 9, NULL, tskNO_AFFINITY)== pdPASS)
  //if(xTaskCreatePinnedToCore(taskUI_Char,"UI",8000,NULL,(portPRIVILEGE_BIT | 4),&uiHandle,0) == pdPASS)
    ESP_LOGI(TAG, "UI_Char task created.");
  else ESP_LOGE(TAG, "Failed to create UI_Char task.");
  //////////////////////////////////////

#endif

	
	xTaskCreatePinnedToCore(task_test, "task_test1", 4096, NULL, 10, NULL, tskNO_AFFINITY);
  //keypad init
  ESP_ERROR_CHECK(keyQueueCreate());
	if(xTaskCreatePinnedToCore(taskScanKey, "KEYSCAN", 2048, NULL, 8, NULL, tskNO_AFFINITY)== pdPASS)
  //if(xTaskCreatePinnedToCore(taskScanKey,"KEYSCAN",2000,NULL,(portPRIVILEGE_BIT | 3),&keyHandle,1) == pdPASS)
    ESP_LOGI(TAG, "KeyScan task created.");
  else ESP_LOGE(TAG, "Failed to create KeyScan task.");



/*
  //battery task init
  if(xTaskCreatePinnedToCore(taskBattery,"BATTERY",2000,NULL,(portPRIVILEGE_BIT | 2),&batHandle,1) == pdPASS)
    ESP_LOGI(TAG, "Battery voltage scanning task created.");
  else ESP_LOGE(TAG, "Failed to create Battery voltage scanning task.");
 
*/
  
     
 /* 
  ledc_init();
  if(xTaskCreatePinnedToCore(taskBacklight,"backlight",2000,NULL,(portPRIVILEGE_BIT | 2),NULL,0) == pdPASS)
    ESP_LOGI(TAG, "Backlight control task created.");
  else ESP_LOGE(TAG, "Failed to create backlight control task.");
*/
	
  //i2s init
 i2s_init();
   //blesink_i2s_init();
  //ESP_LOGI(TAG, "Music i2s clk ===========%f=.",0);
	
  //  player_pause(false);i2s_get_clk(0)

 
 
  
  const char *basePath11 = "/sdcard";
	ESP_LOGI(TAG,"isok:%d\r\n", scan_music_file2(basePath11));

	
	
  //esp_register_freertos_tick_hook(lv_tick_task);
  
  while(1) {
    vTaskDelay(5000 / portTICK_RATE_MS);
	//lv_task_handler();
    //ESP_LOGI(TAG, "got ip:444,");
  }
}


static void lv_tick_task(void) {
  lv_tick_inc(portTICK_RATE_MS);
}

/*
static esp_err_t wifi_event_handler(void *ctx, system_event_t *event) {
    switch(event->event_id) {
      case SYSTEM_EVENT_STA_START:
          esp_wifi_connect();
          break;
      case SYSTEM_EVENT_STA_GOT_IP:
          wifi_set_stat(true);
          ESP_LOGI(TAG, "got ip:%s",
                   ip4addr_ntoa(&event->event_info.got_ip.ip_info.ip));
          wifi_ap_record_t wifidata;
          if (esp_wifi_sta_get_ap_info(&wifidata)==0){
            ESP_LOGI(TAG, "rssi:%d", wifidata.rssi);
          }
          xEventGroupSetBits(wifi_event_group, WIFI_CONNECTED_BIT);
          break;
      case SYSTEM_EVENT_AP_STACONNECTED:
          ESP_LOGI(TAG, "station:"MACSTR" join, AID=%d",
                   MAC2STR(event->event_info.sta_connected.mac),
                   event->event_info.sta_connected.aid);
          break;
      case SYSTEM_EVENT_AP_STADISCONNECTED:
          ESP_LOGI(TAG, "station:"MACSTR"leave, AID=%d",
                   MAC2STR(event->event_info.sta_disconnected.mac),
                   event->event_info.sta_disconnected.aid);
          break;
      case SYSTEM_EVENT_STA_DISCONNECTED:
          wifi_set_stat(false);
          esp_wifi_connect();
          xEventGroupClearBits(wifi_event_group, WIFI_CONNECTED_BIT);
          break;
      default:
          break;
    }
    return ESP_OK;
}


*/
static lv_fs_res_t pcfs_open(void * file_p, const char * fn, lv_fs_mode_t mode)
{
    errno = 0;

    const char * flags = "";

    if(mode == LV_FS_MODE_WR) flags = "wb";
    else if(mode == LV_FS_MODE_RD) flags = "rb";
    else if(mode == (LV_FS_MODE_WR | LV_FS_MODE_RD)) flags = "a+";

    //Make the path relative to the current directory (the projects root folder)
    // char buf[256];
    // sprintf(buf, "./%s", fn);

    pc_file_t f = fopen(fn, flags);
    if((long int)f <= 0) return LV_FS_RES_UNKNOWN;
    else {
        fseek(f, 0, SEEK_SET);

         //'file_p' is pointer to a file descriptor and
         // we need to store our file descriptor here
        pc_file_t * fp = file_p;        //Just avoid the confusing casings
        *fp = f;
    }

    return LV_FS_RES_OK;
}

static lv_fs_res_t pcfs_close(void * file_p)
{
    pc_file_t * fp = file_p;        //Just avoid the confusing casings
    fclose(*fp);
    return LV_FS_RES_OK;
}

static lv_fs_res_t pcfs_read(void * file_p, void * buf, uint32_t btr, uint32_t * br)
{
    pc_file_t * fp = file_p;        //Just avoid the confusing casings
    *br = fread(buf, 1, btr, *fp);
    return LV_FS_RES_OK;
}

static lv_fs_res_t pcfs_seek(void * file_p, uint32_t pos)
{
    pc_file_t * fp = file_p;        //Just avoid the confusing casings
    fseek(*fp, pos, SEEK_SET);
    return LV_FS_RES_OK;
}

static lv_fs_res_t pcfs_tell(void * file_p, uint32_t * pos_p)
{
    pc_file_t * fp = file_p;       //Just avoid the confusing casings
    *pos_p = ftell(*fp);
    return LV_FS_RES_OK;
}



