#include <stdio.h>
#include <stdint.h>
#include <stddef.h>
#include <string.h>
#include "esp_wifi.h"
#include "esp_system.h"
#include "nvs_flash.h"
#include "esp_event_loop.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "freertos/queue.h"
#include "freertos/event_groups.h"

#include "lwip/sockets.h"
#include "lwip/dns.h"
#include "lwip/netdb.h"

#include "esp_log.h"
#include "mqtt_client.h"

static const char *TAG = "MQTTS_EXAMPLE";

static EventGroupHandle_t wifi_event_group;
const static int CONNECTED_BIT = BIT0;
//KEY
#define KEY_IO          35
unsigned char key_status[2];

//LED
#define LED    19
#define GPIO_OUTPUT_PIN_SEL  (1ULL<<LED)

#define LED_ON()		gpio_set_level(LED, 1)
#define LED_OFF()		gpio_set_level(LED, 0)					
unsigned char led_status;

//mqtt连上事件
static EventGroupHandle_t mqtt_event_group;


esp_mqtt_client_handle_t client;

static esp_err_t wifi_event_handler(void *ctx, system_event_t *event)
{
    switch (event->event_id) {
        case SYSTEM_EVENT_STA_START:
            esp_wifi_connect();
            break;
        case SYSTEM_EVENT_STA_GOT_IP:
            xEventGroupSetBits(wifi_event_group, CONNECTED_BIT);

            break;
        case SYSTEM_EVENT_STA_DISCONNECTED:
            esp_wifi_connect();
            xEventGroupClearBits(wifi_event_group, CONNECTED_BIT);
            break;
        default:
            break;
    }
    return ESP_OK;
}

static void wifi_init(void)
{
    tcpip_adapter_init();
    wifi_event_group = xEventGroupCreate();
	mqtt_event_group = xEventGroupCreate();
    ESP_ERROR_CHECK(esp_event_loop_init(wifi_event_handler, NULL));
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));
    ESP_ERROR_CHECK(esp_wifi_set_storage(WIFI_STORAGE_RAM));
    wifi_config_t wifi_config = {
        .sta = {
            .ssid = "tap4fun-guest",
            .password = "tap4fun99",
        },
    };
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(ESP_IF_WIFI_STA, &wifi_config));
    ESP_LOGI(TAG, "start the WIFI SSID:[%s]", CONFIG_WIFI_SSID);
    ESP_ERROR_CHECK(esp_wifi_start());
    ESP_LOGI(TAG, "Waiting for wifi");
    xEventGroupWaitBits(wifi_event_group, CONNECTED_BIT, false, true, portMAX_DELAY);
}

#if CONFIG_BROKER_CERTIFICATE_OVERRIDDEN == 1
static const uint8_t iot_eclipse_org_pem_start[]  = "-----BEGIN CERTIFICATE-----\n" CONFIG_BROKER_CERTIFICATE_OVERRIDE "\n-----END CERTIFICATE-----";
#else
extern const uint8_t iot_eclipse_org_pem_start[]   asm("_binary_iot_eclipse_org_pem_start");
#endif
extern const uint8_t iot_eclipse_org_pem_end[]   asm("_binary_iot_eclipse_org_pem_end");

static esp_err_t mqtt_event_handler(esp_mqtt_event_handle_t event)
{
    esp_mqtt_client_handle_t client = event->client;
    int msg_id;
    // your_context_t *context = event->context;
    switch (event->event_id) {
        case MQTT_EVENT_CONNECTED:  //MQTT连上事件
            ESP_LOGI(TAG, "MQTT_EVENT_CONNECTED");
            //订阅主题函数
			xEventGroupSetBits(mqtt_event_group, CONNECTED_BIT);
			msg_id = esp_mqtt_client_subscribe(client, "$baidu/iot/shadow/eps32_demo1/update/accepted", 0);
            ESP_LOGI(TAG, "sent subscribe successful, msg_id=%d", msg_id);

            msg_id = esp_mqtt_client_subscribe(client, "$baidu/iot/general/leds", 0);
            ESP_LOGI(TAG, "sent subscribe successful, msg_id=%d", msg_id);
			//取消订阅主题函数
            //msg_id = esp_mqtt_client_unsubscribe(client, "/topic/qos1");
            //ESP_LOGI(TAG, "sent unsubscribe successful, msg_id=%d", msg_id);
            break;
        case MQTT_EVENT_DISCONNECTED:  //MQTT断开连接事件
            ESP_LOGI(TAG, "MQTT_EVENT_DISCONNECTED");
			xEventGroupClearBits(mqtt_event_group, CONNECTED_BIT);
            
            break;

        case MQTT_EVENT_SUBSCRIBED:  //MQTT发送订阅事件
            ESP_LOGI(TAG, "MQTT_EVENT_SUBSCRIBED, msg_id=%d", event->msg_id);
            //发布主题函数
			//msg_id = esp_mqtt_client_publish(client, "/topic/qos0", "data", 0, 0, 0);
            //ESP_LOGI(TAG, "sent publish successful, msg_id=%d", msg_id);
            break;
        case MQTT_EVENT_UNSUBSCRIBED: //MQTT取消订阅事件
            ESP_LOGI(TAG, "MQTT_EVENT_UNSUBSCRIBED, msg_id=%d", event->msg_id);
            break;
        case MQTT_EVENT_PUBLISHED: //MQTT发布事件
            ESP_LOGI(TAG, "MQTT_EVENT_PUBLISHED, msg_id=%d", event->msg_id);
            break;
        case MQTT_EVENT_DATA: //MQTT接收数据事件
            ESP_LOGI(TAG, "MQTT_EVENT_DATA");
            printf("TOPIC=%.*s\r\n", event->topic_len, event->topic);
            printf("DATA=%.*s\r\n", event->data_len, event->data);
            break;
        case MQTT_EVENT_ERROR: //MQTT错误事件
            xEventGroupClearBits(mqtt_event_group, CONNECTED_BIT);
            
			ESP_LOGI(TAG, "MQTT_EVENT_ERROR");
            break;
        default:
            ESP_LOGI(TAG, "Other event id:%d", event->event_id);
            
			break;
    }
    return ESP_OK;
}


static void mqtt_app_start(void)
{
    const esp_mqtt_client_config_t mqtt_cfg = {
        //.uri = "mjjba76.mqtt.iot.gz.baidubce.com:1884",
		//.host = "ssl://mjjba76.mqtt.iot.gz.baidubce.com",
		.client_id = "esp32-iot-2019",
		.host = "mjjba76.mqtt.iot.gz.baidubce.com",            
		.port = 1883,
		.username =  "mjjba76/eps32_demo1",
		.password = "d7ny0i4fuvra6s6s",
        .event_handle = mqtt_event_handler,
        .cert_pem = (const char *)iot_eclipse_org_pem_start,
    };

	//ESP_LOGI(TAG, "username: %s bytes", mqtt_cfg.username);
    //ESP_LOGI(TAG, "username: %s bytes", mqtt_cfg.password);
    
	
    ESP_LOGI(TAG, "[APP] Free memory: %d bytes", esp_get_free_heap_size());
    client = esp_mqtt_client_init(&mqtt_cfg);
    esp_mqtt_client_start(client);
}

#define DATASTRon "{\"requestId\": \"esp32-iot-2019\",  \"reported\": {\"LED0\": true,\"temperature\": %d,\"humidity\": %d}}"
#define DATASTRoff "{\"requestId\": \"esp32-iot-2019\",  \"reported\": {\"LED0\": false,\"temperature\": %d,\"humidity\": %d}}"

unsigned int temp = 0 ;
unsigned char humi = 0 ;

bool led0 = 1;
//按键发布主题
void key_read1(void)
{
	
	char *jsondata;
	temp += 1 ;
	humi += 5 ;
    //按键识别
    if(gpio_get_level(KEY_IO)==0){
        key_status[0] = 0;
    }
    else{
       key_status[0] = 1; 
    }
    if(key_status[0]!=key_status[1]) {
        key_status[1] = key_status[0];
        if(key_status[1]==0){//按键按下
            
			ESP_LOGI(TAG, "Key Pressed");
			
			
			jsondata = malloc(128);
			if(led0) 
			{
				led0 = 0;
				LED_ON();
				sprintf(jsondata,DATASTRon,temp,humi);
			}
			else 
			{
				led0 = 1;
				LED_OFF();
				sprintf(jsondata,DATASTRoff,temp,humi);
			}
			
			
            ESP_LOGI(TAG, "data:=%s=",jsondata);
			esp_mqtt_client_publish(client, "$baidu/iot/shadow/eps32_demo1/update", jsondata, 0, 0, 0);
			//esp_mqtt_client_publish(client, "/topic/qos1", "Hello MQTT ,I am HongXu", 0, 0, 0);
			//vTaskDelay(1000 / portTICK_PERIOD_MS);
			free(jsondata);
		}
    }

	else
	{
		if(temp%3000==0)
		{
			//temp =  0 ;
			jsondata = malloc(128);
			if(!led0) 
			{
				//LED_ON();
				sprintf(jsondata,DATASTRon,temp,humi);
			}
			else 
			{
				//LED_OFF();
				sprintf(jsondata,DATASTRoff,temp,humi);
			}
			
            ESP_LOGI(TAG, "data:=%s=",jsondata);
			esp_mqtt_client_publish(client, "$baidu/iot/shadow/eps32_demo1/update", jsondata, 0, 0, 0);
			free(jsondata);
		}
	}
	
}

void app_main()
{
    ESP_LOGI(TAG, "[APP] Startup..");
    ESP_LOGI(TAG, "[APP] Free memory: %d bytes", esp_get_free_heap_size());
    ESP_LOGI(TAG, "[APP] IDF version: %s", esp_get_idf_version());

    esp_log_level_set("*", ESP_LOG_INFO);
    esp_log_level_set("MQTT_CLIENT", ESP_LOG_VERBOSE);
    esp_log_level_set("TRANSPORT_TCP", ESP_LOG_VERBOSE);
    esp_log_level_set("TRANSPORT_SSL", ESP_LOG_VERBOSE);
    esp_log_level_set("TRANSPORT", ESP_LOG_VERBOSE);
    esp_log_level_set("OUTBOX", ESP_LOG_VERBOSE);

    nvs_flash_init();
    wifi_init();
    mqtt_app_start();
	
	    //配置led
    gpio_config_t io_conf;
    io_conf.intr_type = GPIO_PIN_INTR_DISABLE;
    io_conf.mode = GPIO_MODE_OUTPUT;
    io_conf.pin_bit_mask = GPIO_OUTPUT_PIN_SEL;
    io_conf.pull_down_en = 0;
    io_conf.pull_up_en = 0;
    gpio_config(&io_conf);
	LED_OFF();
	//等mqtt连上
    xEventGroupWaitBits(mqtt_event_group, CONNECTED_BIT, false, true, portMAX_DELAY);

	
	gpio_set_direction(KEY_IO, GPIO_MODE_INPUT);

    while (1) {
        //发布主题
        //自己又订阅了
        //mqtt帮我们做了一个回发测试
        //所以会收到这条信息
        //esp_mqtt_client_publish(client, "/topic/qos1", "LED", 0, 0, 0);
		key_read1();
        vTaskDelay(10 / portTICK_PERIOD_MS);

    }
	
}
