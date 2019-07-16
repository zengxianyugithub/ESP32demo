/* BSD Socket API Example

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/
#include <string.h>
#include <sys/param.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "esp_freertos_hooks.h"

#include "esp_task_wdt.h"

#include "esp_system.h"
#include "esp_wifi.h"
#include "esp_event_loop.h"
#include "esp_log.h"
#include "nvs_flash.h"

#include "lwip/err.h"
#include "lwip/sockets.h"
#include "lwip/sys.h"
#include <lwip/netdb.h>
#include "driver/mcpwm.h"
#include "soc/mcpwm_reg.h"
#include "soc/mcpwm_struct.h"
#include "driver/ledc.h"

#include "../lvgl/lvgl.h"
#include "driver/i2s.h"

#include "lcdTFT32.h"

//#include "LcdTFT22_Driver.h"
//#include "dataHandle.h"

#include "dirent.h"


#include "tcpip_adapter.h"
#include "esp_smartconfig.h"

#include "nvs_flash.h"
#include "nvs.h"
#include "driver/gpio.h"

#include "keypad_control.h"
//#include "i2s_dac_demo.h"

#include "cJSON.h"
//#include "myui.h"
//#include "ota_test.h"
#include "tcp_client.h"

/* The examples use simple WiFi configuration that you can set via
   'make menuconfig'.
   If you'd rather not, just change the below entries to strings with
   the config you want - ie #define EXAMPLE_WIFI_SSID "mywifissid"

#define EXAMPLE_WIFI_SSID "Tenda_105"
#define EXAMPLE_WIFI_PASS "c105123456"
*/
#define EXAMPLE_WIFI_SSID "tap4fun-guest"
#define EXAMPLE_WIFI_PASS "tap4fun99"

unsigned char wifi_flag_nvs = 0 ;

#define HOST_IP_ADDR "172.20.200.42"
/*
#ifdef CONFIG_EXAMPLE_IPV4
//#define HOST_IP_ADDR "192.168.1.100"
//#define HOST_IP_ADDR "120.26.97.45"

//#define HOST_IP_ADDR CONFIG_EXAMPLE_IPV4_ADDR

#else
	
#define HOST_IP_ADDR CONFIG_EXAMPLE_IPV6_ADDR

#endif
*/
//#define PORT CONFIG_EXAMPLE_PORT
#define PORT 8102
//#define PORT 5000

/* FreeRTOS event group to signal when we are connected & ready to make a request */
/////////static EventGroupHandle_t wifi_event_group;

const int IPV4_GOTIP_BIT = BIT0;
const int IPV6_GOTIP_BIT = BIT1;

static const char *TAG = "TCP_c";
//static const char *payload = "Message from ESP32 ";

///////////////////////////
/* FreeRTOS event group to signal when we are connected & ready to make a request */
static EventGroupHandle_t s_wifi_event_group;

/* The event group allows multiple bits for each event,
   but we only care about one event - are we connected
   to the AP with an IP? */
static const int CONNECTED_BIT = BIT0;
static const int ESPTOUCH_DONE_BIT = BIT1;
static const char *TAGsc = "sc";

void smartconfig_example_task(void * parm);
void ledpwm(void *pvParameters);


#define STORAGE_NAMESPACE "storage"
char *strwifissid="w12id";
char *strwifipass="f34ss";

/* Save the number of module restarts in NVS
   by first reading and then incrementing
   the number that has been saved previously.
   Return an error if anything goes wrong
   during this process.
 */
esp_err_t save_wifi_ssid(char *wifissid)
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
    err = nvs_set_str(my_handle, strwifissid, wifissid);
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

esp_err_t save_wifi_pass(char *wifipass)
{
    nvs_handle my_handle;
    esp_err_t err;

    // Open
    err = nvs_open(STORAGE_NAMESPACE, NVS_READWRITE, &my_handle);
    if (err != ESP_OK) return err;

    // Read
    //int32_t restart_counter = 0; // value will default to 0, if not set yet in NVS
    //err = nvs_get_i32(my_handle, "mywifipass", &restart_counter);
   //if (err != ESP_OK && err != ESP_ERR_NVS_NOT_FOUND) return err;

    // Write
    //restart_counter++;
    err = nvs_set_str(my_handle, strwifipass, wifipass);
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


static esp_err_t event_handler12(void *ctx, system_event_t *event);
static EventGroupHandle_t wifi_event_group;
static void wait_for_ip();


static esp_err_t event_handler(void *ctx, system_event_t *event)
{
    switch(event->event_id) {
    case SYSTEM_EVENT_STA_START:

		 ESP_LOGI(TAG, "SYSTEM_EVENT_STA_START nvs flag = %d",wifi_flag_nvs);
	
		//if(wifi_flag_nvs == 2)esp_wifi_connect();
       	 xTaskCreate(smartconfig_example_task, "smartconfig_example_task", 4096, NULL, 3, NULL);
		
		break;
    case SYSTEM_EVENT_STA_GOT_IP:
        xEventGroupSetBits(s_wifi_event_group, CONNECTED_BIT);
		/////////wifi_set_stat(true);
		if(wifi_flag_nvs == 2)
			xEventGroupSetBits(s_wifi_event_group, ESPTOUCH_DONE_BIT);
        break;
    case SYSTEM_EVENT_STA_DISCONNECTED:
        esp_wifi_connect();
        xEventGroupClearBits(s_wifi_event_group, CONNECTED_BIT);
		//wifi_set_stat(false);
        break;
    default:
        break;
    }
    return ESP_OK;
}

char str_valuessid[64]="hello";
char str_valuepass[64]="hello";
uint32_t len=64;
char * mywifissid1;
char * mywifipass1;
bool wifi_datasave = false;

static void initialise_wifi(void)
{
	nvs_handle my_handle1; 
	
	

	esp_err_t err;
	wifi_datasave = false;
	
      
            
			
			
	err = nvs_open(STORAGE_NAMESPACE, NVS_READWRITE, &my_handle1);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "opening NVS Error (%s)!\n", esp_err_to_name(err));
    } else {
		ESP_LOGI(TAG, "NVS open OK");

		size_t required_size;
		err = nvs_get_str(my_handle1, strwifissid, NULL, &required_size);
		if(err==ESP_OK)
		{
			mywifissid1 = malloc(required_size);
			nvs_get_str(my_handle1, strwifissid, mywifissid1, &required_size);
			wifi_flag_nvs = 1 ;
		}
		else
		{
			wifi_flag_nvs = 0 ;
			 ESP_LOGI(TAG,"Error (%s) reading!\n", esp_err_to_name(err));
		}

		vTaskDelay(50 / portTICK_RATE_MS);	
		
		err = nvs_get_str(my_handle1, strwifipass, NULL, &required_size);
		if(err==ESP_OK)
		{
			mywifipass1 = malloc(required_size);
			nvs_get_str(my_handle1, strwifipass, mywifipass1, &required_size);
			if(wifi_flag_nvs==1)wifi_flag_nvs = 2 ;//2
			
			//memset(majiang.wifissid,0,sizeof(majiang.wifissid));
			//sprintf( majiang.wifissid,"%s",mywifissid1);
			//ESP_LOGI(TAG,"majiang.wifissid :%s\n",majiang.wifissid);
		}
		
		else
		{
			wifi_flag_nvs = 0 ;
			wifi_datasave = false;
			 ESP_LOGI(TAG,"Error (%s) reading!\n", esp_err_to_name(err));
		}
    }
	nvs_close(my_handle1);
	

	//ESP_ERR_NVS_NOT_FOUND

////////
	if(wifi_flag_nvs == 2)
	{
		wifi_datasave = true;
		tcpip_adapter_init();
		wifi_event_group = xEventGroupCreate();
		ESP_ERROR_CHECK( esp_event_loop_init(event_handler12, NULL) );
		wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
		ESP_ERROR_CHECK( esp_wifi_init(&cfg) );
		ESP_ERROR_CHECK( esp_wifi_set_storage(WIFI_STORAGE_RAM) );
		wifi_config_t wifi_config1 = {
			.sta = {
				.ssid = "1",
				.password = "2",
			},
		};
		memcpy(wifi_config1.sta.ssid, mywifissid1, strlen(mywifissid1));
			memcpy(wifi_config1.sta.password, mywifipass1, strlen(mywifipass1));
			memset(&wifi_config1.sta.ssid[strlen(mywifissid1)], 0, sizeof(wifi_config1.sta.ssid)-strlen(mywifissid1));
			memset(&wifi_config1.sta.password[strlen(mywifipass1)], 0, sizeof(wifi_config1.sta.password)-strlen(mywifipass1));

			ESP_LOGI(TAGsc, "nvswifiSSID:%d", strlen(mywifissid1));
			ESP_LOGI(TAGsc, "nvswifiSSID:%s+++", wifi_config1.sta.ssid);
			
			ESP_LOGI(TAGsc, "nvswifiPASSWORD:%s+++", wifi_config1.sta.password);
		
		//ESP_LOGI(TAG, "Setting WiFi configuration SSID %s...", wifi_config.sta.ssid);
		//ESP_LOGI(TAG, "con WIFI SSID:[%s] password:[%s]", wifi_config.sta.ssid, wifi_config.sta.password);
		ESP_ERROR_CHECK( esp_wifi_set_mode(WIFI_MODE_STA) );
		ESP_ERROR_CHECK( esp_wifi_set_config(ESP_IF_WIFI_STA, &wifi_config1) );
		ESP_ERROR_CHECK( esp_wifi_start() );
		free(mywifissid1);
		free(mywifipass1);
/*
	
		tcpip_adapter_init();
	    wifi_event_group = xEventGroupCreate();
	    ESP_ERROR_CHECK( esp_event_loop_init(event_handler12, NULL) );
	    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
	    ESP_ERROR_CHECK( esp_wifi_init(&cfg) );
	    ESP_ERROR_CHECK( esp_wifi_set_storage(WIFI_STORAGE_RAM) );
		
			wifi_config_t wifi_config1 ;

			memcpy(wifi_config1.sta.ssid, mywifissid1, strlen(mywifissid1));
			memcpy(wifi_config1.sta.password, mywifipass1, strlen(mywifipass1));
			memset(&wifi_config1.sta.ssid[strlen(mywifissid1)], 0, sizeof(wifi_config1.sta.ssid)-strlen(mywifissid1)-1);
			//memset(wifi_config1.sta.password[strlen(mywifissid1)], 0, sizeof(wifi_config1.sta.password));

			ESP_LOGI(TAGsc, "nvswifiSSID:%d", strlen(mywifissid1));
			ESP_LOGI(TAGsc, "nvswifiSSID:%s+++", wifi_config1.sta.ssid);
			
			ESP_LOGI(TAGsc, "nvswifiPASSWORD:%s+++", wifi_config1.sta.password);
	        //ESP_ERROR_CHECK( esp_wifi_set_config(ESP_IF_WIFI_STA, &wifi_config1) );
		
	    ESP_LOGI(TAG, "Setting WiFi configuration SSID %s...", wifi_config1.sta.ssid);
		ESP_LOGI(TAG, "con WIFI SSID:[%s] password:[%s]", wifi_config1.sta.ssid, wifi_config1.sta.password);
	    ESP_ERROR_CHECK( esp_wifi_set_mode(WIFI_MODE_STA) );
	    ESP_ERROR_CHECK( esp_wifi_set_config(ESP_IF_WIFI_STA, &wifi_config1) );
	    ESP_ERROR_CHECK( esp_wifi_start() );
	    */
	}
	else
	{
		tcpip_adapter_init();
	    s_wifi_event_group = xEventGroupCreate();
	    ESP_ERROR_CHECK( esp_event_loop_init(event_handler, NULL) );

	    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();

	    ESP_ERROR_CHECK( esp_wifi_init(&cfg) );
		
	    ESP_ERROR_CHECK( esp_wifi_set_mode(WIFI_MODE_STA) );
		
		
	    ESP_ERROR_CHECK( esp_wifi_start() );
	}
////////////
	
    
}

static void sc_callback(smartconfig_status_t status, void *pdata)
{
	
    switch (status) {
        case SC_STATUS_WAIT:
            ESP_LOGI(TAGsc, "SC_STATUS_WAIT");
            break;
        case SC_STATUS_FIND_CHANNEL:
            ESP_LOGI(TAGsc, "SC_STATUS_FINDING_CHANNEL");
            break;
        case SC_STATUS_GETTING_SSID_PSWD:
            ESP_LOGI(TAGsc, "SC_STATUS_GETTING_SSID_PSWD");
            break;
        case SC_STATUS_LINK:
            ESP_LOGI(TAGsc, "SC_STATUS_LINK");
            wifi_config_t *wifi_config = pdata;
            ESP_LOGI(TAGsc, "SSID:%s", wifi_config->sta.ssid);
            ESP_LOGI(TAGsc, "PASSWORD:%s", wifi_config->sta.password);
			memset(str_valuessid, 0, 32);
			memset(str_valuepass, 0, 32);
			
			
			//memset(majiang.wifissid,0,sizeof(majiang.wifissid));
			//sprintf( majiang.wifissid,"%s",wifi_config->sta.ssid);

			//ESP_LOGI(TAG, "majiang.wifissid!!!!%s",majiang.wifissid);
            	
			
			sprintf( str_valuessid,"%s",wifi_config->sta.ssid);
			sprintf(str_valuepass,"%s",wifi_config->sta.password);
			ESP_LOGI(TAG, "save_valuepass len = %d",strlen(str_valuepass));

			
			str_valuepass[strlen(str_valuepass)] = 0 ;
			if(save_wifi_pass((char *)str_valuepass) == ESP_OK)
			{
				ESP_LOGI(TAG, "save wifi pass ok!!!!%s",str_valuepass);
            	
			}
			else
			{
				ESP_LOGI(TAG, "save wifi pass faile!!!!!!!!!");
            	
			}
			str_valuessid[strlen(str_valuessid)] = 0 ;
			if(save_wifi_ssid((char *)str_valuessid) == ESP_OK)
			{
				ESP_LOGI(TAG, "save wifi ssid ok!!!!%s",str_valuessid);
            	wifi_datasave = true;
			}
			else
			{
				ESP_LOGI(TAG, "save wifi ssid faile!!!!!!!!!");
            
			}
			
			 
            ESP_ERROR_CHECK(esp_wifi_disconnect());
            ESP_ERROR_CHECK(esp_wifi_set_config(ESP_IF_WIFI_STA, wifi_config)  );
            ESP_ERROR_CHECK(esp_wifi_connect()  );
            break;
        case SC_STATUS_LINK_OVER:
            ESP_LOGI(TAGsc, "SC_STATUS_LINK_OVER");
            if (pdata != NULL) {
                uint8_t phone_ip[4] = { 0 };
                memcpy(phone_ip, (uint8_t* )pdata, 4);
                ESP_LOGI(TAGsc, "Phone ip: %d.%d.%d.%d\n", phone_ip[0], phone_ip[1], phone_ip[2], phone_ip[3]);
            }

			
			
            xEventGroupSetBits(s_wifi_event_group, ESPTOUCH_DONE_BIT);
            break;
        default:
            break;
    }
}

void smartconfig_example_task(void * parm)
{
    EventBits_t uxBits;
	if(wifi_flag_nvs != 2)
	{
   		ESP_ERROR_CHECK( esp_smartconfig_set_type(SC_TYPE_ESPTOUCH) );
    	ESP_ERROR_CHECK( esp_smartconfig_start(sc_callback) );
	}
	/*
	else
	{
		xEventGroupSetBits(s_wifi_event_group, ESPTOUCH_DONE_BIT);
	
	}
	*/
    while (1) {

		if(wifi_flag_nvs != 2)
		{
		
	        uxBits = xEventGroupWaitBits(s_wifi_event_group, CONNECTED_BIT | ESPTOUCH_DONE_BIT, true, false, portMAX_DELAY); 
	        if(uxBits & CONNECTED_BIT) {
	            ESP_LOGI(TAGsc, "WiFi Connected to ap");
	        }
	        if(uxBits & ESPTOUCH_DONE_BIT) {
	            ESP_LOGI(TAGsc, "smartconfig over");
				//if(wifi_flag_nvs != 2)
	            	esp_smartconfig_stop();
				wifi_flag_nvs = 3;
				
				vTaskDelete(NULL);
       		}
		}
		else
		{
			 wait_for_ip();
			 wifi_flag_nvs = 3 ;
			
			vTaskDelete(NULL);

		
            
        }
    }
}

///////////////////////////




static esp_err_t event_handler12(void *ctx, system_event_t *event)
{
    switch (event->event_id) {
    case SYSTEM_EVENT_STA_START:
        esp_wifi_connect();
		xTaskCreate(smartconfig_example_task, "smartconfig_example_task", 4096, NULL, 3, NULL);
		
        ESP_LOGI(TAG, "SYSTEM_EVENT_STA_START");
        break;
    case SYSTEM_EVENT_STA_CONNECTED:
         //enable ipv6 
        tcpip_adapter_create_ip6_linklocal(TCPIP_ADAPTER_IF_STA);
        break;
    case SYSTEM_EVENT_STA_GOT_IP:
        xEventGroupSetBits(wifi_event_group, IPV4_GOTIP_BIT);
        ESP_LOGI(TAG, "SYSTEM_EVENT_STA_GOT_IP");
		ESP_LOGI(TAG, "got ip:%s",
        ip4addr_ntoa(&event->event_info.got_ip.ip_info.ip));
		//wifi_set_stat(true);
        break;
    case SYSTEM_EVENT_STA_DISCONNECTED:
       //This is a workaround as ESP32 WiFi libs don't currently auto-reassociate. 
        esp_wifi_connect();
        xEventGroupClearBits(wifi_event_group, IPV4_GOTIP_BIT);
        xEventGroupClearBits(wifi_event_group, IPV6_GOTIP_BIT);
		//wifi_set_stat(false);
        break;
    case SYSTEM_EVENT_AP_STA_GOT_IP6:
        xEventGroupSetBits(wifi_event_group, IPV6_GOTIP_BIT);
        ESP_LOGI(TAG, "SYSTEM_EVENT_STA_GOT_IP6");

        char *ip6 = ip6addr_ntoa(&event->event_info.got_ip6.ip6_info.ip);
        ESP_LOGI(TAG, "IPv6: %s", ip6);
    default:
        break;
    }
    return ESP_OK;
}
/*
static void initialise_wifi12(void)
{
    tcpip_adapter_init();
    wifi_event_group = xEventGroupCreate();
    ESP_ERROR_CHECK( esp_event_loop_init(event_handler12, NULL) );
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK( esp_wifi_init(&cfg) );
    ESP_ERROR_CHECK( esp_wifi_set_storage(WIFI_STORAGE_RAM) );
    wifi_config_t wifi_config = {
        .sta = {
            .ssid = EXAMPLE_WIFI_SSID,
            .password = EXAMPLE_WIFI_PASS,
        },
    };
    ESP_LOGI(TAG, "Setting WiFi configuration SSID %s...", wifi_config.sta.ssid);
	ESP_LOGI(TAG, "con WIFI SSID:[%s] password:[%s]", wifi_config.sta.ssid, wifi_config.sta.password);
    ESP_ERROR_CHECK( esp_wifi_set_mode(WIFI_MODE_STA) );
    ESP_ERROR_CHECK( esp_wifi_set_config(ESP_IF_WIFI_STA, &wifi_config) );
    ESP_ERROR_CHECK( esp_wifi_start() );
}
*/


static void wait_for_ip()
{
    uint32_t bits = IPV4_GOTIP_BIT | IPV6_GOTIP_BIT ;

    ESP_LOGI(TAG, "Waiting for AP connection...");
    xEventGroupWaitBits(wifi_event_group, bits,  true, false,portMAX_DELAY);
    //	xEventGroupWaitBits(s_wifi_event_group, CONNECTED_BIT | ESPTOUCH_DONE_BIT, true, false, portMAX_DELAY);
    ESP_LOGI(TAG, "Connected to AP11");
}

int sock;
/*
static void tcp_client_send_task(void *pvParameters)
{
	char tx_buffer[512];
	int num=0;
	vTaskDelay(1000 / portTICK_PERIOD_MS);
	
	while(wifi_connected)
	{
		while(server_connected)
		{
			if(xQueueReceive(Queue_Tcpsenddata, tx_buffer, 10) == pdPASS) {
					ESP_LOGI(TAG, "senddata==========%s", tx_buffer);
					int err = send(sock, tx_buffer, strlen(tx_buffer), 0);
					if (err < 0) {
						ESP_LOGE(TAG, "Error occured during sending: errno %d", errno);
						shutdown(sock, 0);
						close(sock);
						break;
					}
				
				}
				
			num ++ ;
			if(num == 15)
			{
				tcppostsenddata();
				//tcpdataretrun("A006");
				num =0 ;
			}
				
			vTaskDelay(50 / portTICK_PERIOD_MS);
		}
		vTaskDelay(1000 / portTICK_PERIOD_MS);
	}
	
}
*/
static void tcp_client_task(void *pvParameters)
{
    char rx_buffer[1024*5];
	//char tx_buffer[512];
    char addr_str[128];
	char musicmymame[100];
    int addr_family;
    int ip_protocol;
	FILE* fdb;
	
    while (1) {

	//while (wifi_connected) {

//#ifdef CONFIG_EXAMPLE_IPV4
        struct sockaddr_in destAddr;
        destAddr.sin_addr.s_addr = inet_addr(HOST_IP_ADDR);
        destAddr.sin_family = AF_INET;
        destAddr.sin_port = htons(PORT);
        addr_family = AF_INET;
        ip_protocol = IPPROTO_IP;
        inet_ntoa_r(destAddr.sin_addr, addr_str, sizeof(addr_str) - 1);

		ESP_LOGE(TAG, "socket---ip  %s  port%d ",HOST_IP_ADDR,PORT);
/*#else // IPV6
        struct sockaddr_in6 destAddr;
        inet6_aton(HOST_IP_ADDR, &destAddr.sin6_addr);
        destAddr.sin6_family = AF_INET6;
        destAddr.sin6_port = htons(PORT);
        addr_family = AF_INET6;
        ip_protocol = IPPROTO_IPV6;
        inet6_ntoa_r(destAddr.sin6_addr, addr_str, sizeof(addr_str) - 1);
#endif*/
		
		
 
        sock =  socket(addr_family, SOCK_STREAM, ip_protocol);
        if (sock < 0) {
            ESP_LOGE(TAG, "Unable to create socket: errno %d", errno);
            break;
        }
        ESP_LOGI(TAG, "Socket created");

        int err = connect(sock, (struct sockaddr *)&destAddr, sizeof(destAddr));
        if (err != 0) {
            ESP_LOGE(TAG, "Socket unable to connect: errno %d", errno);
            break;
        }
        ESP_LOGI(TAG, "Successfully connected");
		////////server_set_stat(true);//连接状态
		//tcpdataretrun("A006");
		vTaskDelay(10 / portTICK_PERIOD_MS);
		//tcpdataretrun("A003");
		//tcppostsenddata();
		int32_t filelen = 0 ;
		
		struct stat st;
		unsigned int indexmp3 = 0 ;
		while(1)
		{
			indexmp3 ++ ;
			memset(musicmymame,0,sizeof(musicmymame));
			
			sprintf(musicmymame,"/sdcard/music/MYM%d.mp3",indexmp3);
			
			if (stat(musicmymame, &st) == 0) {
				// Delete it if it exists
				//unlink("/sdcard/playlist.db");
				
				ESP_LOGI(TAG, "MYM%d.mp3",indexmp3);
			}
			else
			{
				ESP_LOGI(TAG, "Opening file");
				fdb = fopen(musicmymame, "wb");
				if (fdb == NULL) {
					ESP_LOGE(TAG, "Failed to open file for writing");
					//return;
				}
				ESP_LOGI(TAG, "MYM%d.mp3",indexmp3);
				break;
			}
			
			
		}
		
		 
		
        while (1) {
			 /*
			if(xQueueReceive(Queue_Tcpsenddata, tx_buffer, 10) == pdPASS) {
				int err = send(sock, tx_buffer, strlen(tx_buffer), 0);
				if (err < 0) {
					ESP_LOGE(TAG, "Error occured during sending: errno %d", errno);
					break;
				}
			}
			 */
            int len = 0 ;
			
			 	//消息队列剩余大小
			//uint8_t msgq_remain_size=uxQueueSpacesAvailable(Queue_Tcprecvdata);//得到队列剩余大小
			
			//ESP_LOGI(TAG,"@@@@@@@@@tcprx_size = %d\n",msgq_remain_size);
			//if(msgq_remain_size>0)
			//{
				len = recv(sock, rx_buffer, sizeof(rx_buffer) - 1, 0);
				// Error occured during receiving
				if (len < 0) {
					ESP_LOGE(TAG, "recv failed: errno %d", errno);
					break;
				}
				// Data received
				else {
					//rx_buffer[len] = 0; // Null-terminate whatever we received and treat like a string
					ESP_LOGI(TAG, "Received %d bytes from %s:", len, addr_str);
					///ESP_LOGI(TAG, "%s", rx_buffer);
					//接收到的数据发送到接收消息队列
					//if(strstr(rx_buffer, "HTTP/1.1 200") >0)
					//fprintf(f, "Hello %s!\n", card->cid.name);
					if(len==0)
					{
						fclose(fdb);
						ESP_LOGI(TAG, "File written");	
						break;
					}
					else
					{
						fwrite(rx_buffer, 1, len, fdb);
						filelen = filelen + len ;
						ESP_LOGI(TAG, "Received (%d) ",filelen);
						//size_t bytes_written;
						//i2s_write(0, rx_buffer, len, &bytes_written, portMAX_DELAY);
					}
					
					
					
					
					//if(len>0)
					//	xQueueSend(Queue_Tcprecvdata, (void*)rx_buffer, (TickType_t) 10);
					//else 
					//	tcppostsenddata();
						
				}
			//}

            vTaskDelay(100 / portTICK_PERIOD_MS);
        }
		
		//server_set_stat(false);
		ESP_LOGE(TAG, "sock------>%d...",sock);
        //if (sock != -1) {
            ESP_LOGE(TAG, "Shutting down socket and restarting...");
            shutdown(sock, 0);
            close(sock);
        }
    //}
	 vTaskDelay(3000 / portTICK_PERIOD_MS);

	
	ESP_LOGE(TAG, "vTaskDelete...tcptask===");
	vTaskDelete(NULL);
}


void mywifiinit1()
{
	ESP_LOGI(TAG,"--------------------------------\r\n");
	initialise_wifi();
    while(wifi_flag_nvs != 3)
	{
		ESP_LOGI(TAG, "======.");
		vTaskDelay(1000 / portTICK_RATE_MS);
	}
	if(	xTaskCreate(tcp_client_task, "tcp_client",10240 , NULL, 5, NULL)==pdPASS)//8192
  		ESP_LOGI(TAG, "tcp_client create created....");
	else
		ESP_LOGI(TAG, "tcp_client create faile....");
	
    

	ESP_LOGI("== ==2", "memory: %d bytes", esp_get_free_heap_size());
	
	
}
