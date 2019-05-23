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

#include "LcdTFT22_Driver.h"
#include "dataHandle.h"

#include "dirent.h"
#include "i2s_dac.h"

//#include "keypad_control.h"
#include "mp3dec.h"

#include "lvgldemo.h"
#include "cJSON.h"
#include "myui.h"

#define LEDC_TEST_CH_NUM       (1)
#define LEDC_TEST_DUTY         (500)
#define LEDC_TEST_FADE_TIME    (10000)

#define LEDC_HS_TIMER          LEDC_TIMER_0
#define LEDC_HS_MODE           LEDC_HIGH_SPEED_MODE
#define LEDC_HS_CH0_GPIO       (18)
#define LEDC_HS_CH0_CHANNEL    LEDC_CHANNEL_0
#define LEDC_LS_TIMER          LEDC_TIMER_1

#define LEDC_LS_MODE           LEDC_LOW_SPEED_MODE

/* The examples use simple WiFi configuration that you can set via
   'make menuconfig'.
   If you'd rather not, just change the below entries to strings with
   the config you want - ie #define EXAMPLE_WIFI_SSID "mywifissid"

#define EXAMPLE_WIFI_SSID "Tenda_105"
#define EXAMPLE_WIFI_PASS "c105123456"
*/
#define EXAMPLE_WIFI_SSID "tap4fun-guest"
#define EXAMPLE_WIFI_PASS "tap4fun99"

#ifdef CONFIG_EXAMPLE_IPV4
#define HOST_IP_ADDR "172.20.200.40"
//#define HOST_IP_ADDR CONFIG_EXAMPLE_IPV4_ADDR

#else
#define HOST_IP_ADDR CONFIG_EXAMPLE_IPV6_ADDR
#endif

//#define PORT CONFIG_EXAMPLE_PORT
#define PORT 5000


/* FreeRTOS event group to signal when we are connected & ready to make a request */
static EventGroupHandle_t wifi_event_group;

const int IPV4_GOTIP_BIT = BIT0;
const int IPV6_GOTIP_BIT = BIT1;

static const char *TAG = "example";
static const char *payload = "Message from ESP32 ";

uint16_t pwmduty = 0 ;

uint16_t freq_hz1 = 5000;



static esp_err_t event_handler(void *ctx, system_event_t *event)
{
    switch (event->event_id) {
    case SYSTEM_EVENT_STA_START:
        esp_wifi_connect();
        ESP_LOGI(TAG, "SYSTEM_EVENT_STA_START");
        break;
    case SYSTEM_EVENT_STA_CONNECTED:
        /* enable ipv6 */
        tcpip_adapter_create_ip6_linklocal(TCPIP_ADAPTER_IF_STA);
        break;
    case SYSTEM_EVENT_STA_GOT_IP:
        xEventGroupSetBits(wifi_event_group, IPV4_GOTIP_BIT);
        ESP_LOGI(TAG, "SYSTEM_EVENT_STA_GOT_IP");
		ESP_LOGI(TAG, "got ip:%s",
                 ip4addr_ntoa(&event->event_info.got_ip.ip_info.ip));
		wifi_set_stat(true);
        break;
    case SYSTEM_EVENT_STA_DISCONNECTED:
        /* This is a workaround as ESP32 WiFi libs don't currently auto-reassociate. */
        esp_wifi_connect();
        xEventGroupClearBits(wifi_event_group, IPV4_GOTIP_BIT);
        xEventGroupClearBits(wifi_event_group, IPV6_GOTIP_BIT);
		wifi_set_stat(false);
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

static void initialise_wifi(void)
{
    tcpip_adapter_init();
    wifi_event_group = xEventGroupCreate();
    ESP_ERROR_CHECK( esp_event_loop_init(event_handler, NULL) );
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



static void wait_for_ip()
{
    uint32_t bits = IPV4_GOTIP_BIT | IPV6_GOTIP_BIT ;

    ESP_LOGI(TAG, "Waiting for AP connection...");
    xEventGroupWaitBits(wifi_event_group, bits, false, true, portMAX_DELAY);
    ESP_LOGI(TAG, "Connected to AP");
}


static void tcp_client_task(void *pvParameters)
{
    char rx_buffer[512];
    char addr_str[128];
    int addr_family;
    int ip_protocol;

    while (1) {

#ifdef CONFIG_EXAMPLE_IPV4
        struct sockaddr_in destAddr;
        destAddr.sin_addr.s_addr = inet_addr(HOST_IP_ADDR);
        destAddr.sin_family = AF_INET;
        destAddr.sin_port = htons(PORT);
        addr_family = AF_INET;
        ip_protocol = IPPROTO_IP;
        inet_ntoa_r(destAddr.sin_addr, addr_str, sizeof(addr_str) - 1);

		ESP_LOGE(TAG, "socket---ip  %s  port%d ",HOST_IP_ADDR,PORT);
#else // IPV6
        struct sockaddr_in6 destAddr;
        inet6_aton(HOST_IP_ADDR, &destAddr.sin6_addr);
        destAddr.sin6_family = AF_INET6;
        destAddr.sin6_port = htons(PORT);
        addr_family = AF_INET6;
        ip_protocol = IPPROTO_IPV6;
        inet6_ntoa_r(destAddr.sin6_addr, addr_str, sizeof(addr_str) - 1);
#endif

        int sock =  socket(addr_family, SOCK_STREAM, ip_protocol);
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
		server_set_stat(true);//连接状态
        while (1) {
			
			if(xQueueReceive(Queue_Tcpsenddata, rx_buffer, 10) == pdPASS) {
				int err = send(sock, rx_buffer, strlen(rx_buffer), 0);
				if (err < 0) {
					ESP_LOGE(TAG, "Error occured during sending: errno %d", errno);
					break;
				}
			}
            
 
            int len = recv(sock, rx_buffer, sizeof(rx_buffer) - 1, 0);
            // Error occured during receiving
            if (len < 0) {
                ESP_LOGE(TAG, "recv failed: errno %d", errno);
                break;
            }
            // Data received
            else {
                rx_buffer[len] = 0; // Null-terminate whatever we received and treat like a string
                ESP_LOGI(TAG, "Received %d bytes from %s:", len, addr_str);
                ESP_LOGI(TAG, "%s", rx_buffer);
				//接收到的数据发送到接收消息队列
				xQueueSend(Queue_Tcprecvdata, (void*)rx_buffer, (TickType_t) 10);
				//cjson_to_struct_info(rx_buffer);
				
				freq_hz1 = rx_buffer[0]*1000+rx_buffer[1]*100 +rx_buffer[2]*10+rx_buffer[3];
				
				if(freq_hz1>9999) freq_hz1 = 9999;
				
				//pwmduty = rx_buffer[0]*100;
				//if(pwmduty>1000)pwmduty = 1000;
				//if(pwmduty>10000) = 10000
				//pwmduty = rx_buffer[0]*100;
				//if(pwmduty>1000)pwmduty = 1000;
				//if(pwmduty>10000) = 10000
            }
             

            vTaskDelay(100 / portTICK_PERIOD_MS);
        }
		
		server_set_stat(false);
		
        if (sock != -1) {
            ESP_LOGE(TAG, "Shutting down socket and restarting...");
            shutdown(sock, 0);
            close(sock);
        }
    }
    server_set_stat(false);
	ESP_LOGE(TAG, "vTaskDelete...tcptask===");
	vTaskDelete(NULL);
}
void ledpwm(void *pvParameters)
{
	int ch;
	printf("--------------------------------\r\n");
    /*
     * Prepare and set configuration of timers
     * that will be used by LED Controller
     */
    ledc_timer_config_t ledc_timer = {
        .duty_resolution = LEDC_TIMER_10_BIT, // resolution of PWM duty
        .freq_hz = freq_hz1,                      // frequency of PWM signal
        .speed_mode = LEDC_HS_MODE,           // timer mode
        .timer_num = LEDC_HS_TIMER            // timer index
    };
    // Set configuration of timer0 for high speed channels
    ledc_timer_config(&ledc_timer);

    // Prepare and set configuration of timer1 for low speed channels
    //ledc_timer.speed_mode = LEDC_LS_MODE;
   // ledc_timer.timer_num = LEDC_LS_TIMER;
   // ledc_timer_config(&ledc_timer);

    /*
     * Prepare individual configuration
     * for each channel of LED Controller
     * by selecting:
     * - controller's channel number
     * - output duty cycle, set initially to 0
     * - GPIO number where LED is connected to
     * - speed mode, either high or low
     * - timer servicing selected channel
     *   Note: if different channels use one timer,
     *         then frequency and bit_num of these channels
     *         will be the same
     */
    ledc_channel_config_t ledc_channel = {
        
            .channel    = LEDC_HS_CH0_CHANNEL,
            .duty       = 0,
            .gpio_num   = LEDC_HS_CH0_GPIO,
            .speed_mode = LEDC_HS_MODE,
            .hpoint     = 0,
            .timer_sel  = LEDC_HS_TIMER
        
    };

    // Set LED Controller with previously prepared configuration
    //for (ch = 0; ch < LEDC_TEST_CH_NUM; ch++) {
        ledc_channel_config(&ledc_channel);
   // }

	ledc_fade_func_install(0);

 
	ESP_LOGI(TAG, "wait_for_ip");
	wait_for_ip();

	if(	xTaskCreate(tcp_client_task, "tcp_client", 8192, NULL, 5, NULL)==pdPASS)
  		ESP_LOGI(TAG, "tcp_client create created....");
	else
		ESP_LOGI(TAG, "tcp_client create faile....");
	 
    while (1) {
        
        /*

		printf("1. LEDC fade up to duty = %d\n", LEDC_TEST_DUTY);
        //for (ch = 0; ch < LEDC_TEST_CH_NUM; ch++) {
            ledc_set_fade_with_time(ledc_channel.speed_mode,
                    ledc_channel.channel, LEDC_TEST_DUTY, LEDC_TEST_FADE_TIME);
            ledc_fade_start(ledc_channel.speed_mode,
                    ledc_channel.channel, LEDC_FADE_NO_WAIT);
        //}
        vTaskDelay(LEDC_TEST_FADE_TIME / portTICK_PERIOD_MS);
		
		printf("2. LEDC fade down to duty = 0\n");
       //for (ch = 0; ch < LEDC_TEST_CH_NUM; ch++) {
            ledc_set_fade_with_time(ledc_channel.speed_mode,
                    ledc_channel.channel, 0, LEDC_TEST_FADE_TIME);
            ledc_fade_start(ledc_channel.speed_mode,
                    ledc_channel.channel, LEDC_FADE_NO_WAIT);
        //}
        vTaskDelay(LEDC_TEST_FADE_TIME / portTICK_PERIOD_MS);

		*/		
		ledc_timer.freq_hz = freq_hz1;
		ledc_timer_config(&ledc_timer);
		pwmduty = 500 ;
        //printf("4. =====LEDC set duty = %d \n",pwmduty);
        //for (ch = 0; ch < LEDC_TEST_CH_NUM; ch++) {
            ledc_set_duty(ledc_channel.speed_mode, ledc_channel.channel, pwmduty);
            ledc_update_duty(ledc_channel.speed_mode, ledc_channel.channel);
        //}
		vTaskDelay(5000 / portTICK_PERIOD_MS);
		
		
    }
}
static void lv_tick_task(void) {
  lv_tick_inc(portTICK_RATE_MS);
}

void app_main()
{
    ESP_ERROR_CHECK( nvs_flash_init() );
    initialise_wifi();
	lv_init();
	Lcd_Init();
    //Lcd_Clear(GRAY0);
	Lcd_Clear(RED);//清屏
	 
	lv_disp_drv_t disp;
	lv_disp_drv_init(&disp);
	disp.disp_flush = TFT22lcd_flush;
	lv_disp_drv_register(&disp);
	
	vTaskDelay(50 / portTICK_RATE_MS);
	//demo_create();
	//testfill();
	xTaskCreate(taskUI_Char, "UI_task", 2048, NULL, 5, NULL);
	if (TcprecvdataQueueCreate() == ESP_OK)
		ESP_LOGI(TAG, "TcpdataQueueCreate  created.");
	else ESP_LOGE(TAG, "Failed to TcpdataQueueCreate.");
	
	if (TcpsenddataQueueCreate() == ESP_OK)
		ESP_LOGI(TAG, "TcpsenddataQueueCreate  created.");
	else ESP_LOGE(TAG, "Failed to TcpsenddataQueueCreate.");
	
	if(	xTaskCreate(dataHandle_task, "tcp_client", 8192, NULL, 5, NULL)==pdPASS)
  		ESP_LOGI(TAG, "dataHandle_task create created....");
	else
		ESP_LOGI(TAG, "dataHandle_task create faile....");
	
	vTaskDelay(50 / portTICK_RATE_MS);
	xTaskCreate(ledpwm, "ledpwmtask", 2048, NULL, 6, NULL);
    //wait_for_ip();
	
    //xTaskCreate(tcp_client_task, "tcp_client", 4096, NULL, 5, NULL);
	esp_register_freertos_tick_hook(lv_tick_task);
  
	while(1) {
		vTaskDelay(5 / portTICK_RATE_MS);
		lv_task_handler();
		//ESP_LOGI(TAG, "got ip:444,");
	}
}
