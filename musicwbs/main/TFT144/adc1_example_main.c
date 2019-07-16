/* ADC1 Example

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/
#include <stdio.h>
#include <stdlib.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "driver/adc.h"
#include "driver/dac.h"

#include "esp_adc_cal.h"


#include "string.h"

#include "freertos/queue.h"
#include "esp_attr.h"
#include "soc/rtc.h"
#include "driver/mcpwm.h"
#include "soc/mcpwm_reg.h"
#include "soc/mcpwm_struct.h"
#include "driver/ledc.h"
#include "esp_err.h"

#include "Lcd_Driver.h"

#include "../lvgl/lvgl.h"


#define DEFAULT_VREF    1100        //Use adc2_vref_to_gpio() to obtain a better estimate
#define NO_OF_SAMPLES   64          //Multisampling

#define LEDC_HS_TIMER          LEDC_TIMER_0
#define LEDC_HS_MODE           LEDC_HIGH_SPEED_MODE
#define LEDC_HS_CH0_GPIO       (18)
#define LEDC_HS_CH0_CHANNEL    LEDC_CHANNEL_0
#define LEDC_HS_CH1_GPIO       (19)
#define LEDC_HS_CH1_CHANNEL    LEDC_CHANNEL_1

#define LEDC_LS_TIMER          LEDC_TIMER_1
#define LEDC_LS_MODE           LEDC_LOW_SPEED_MODE
#define LEDC_LS_CH2_GPIO       (4)
#define LEDC_LS_CH2_CHANNEL    LEDC_CHANNEL_2
#define LEDC_LS_CH3_GPIO       (5)
#define LEDC_LS_CH3_CHANNEL    LEDC_CHANNEL_3

#define LEDC_TEST_CH_NUM       (1)
#define LEDC_TEST_DUTY         (7000)
#define LEDC_TEST_FADE_TIME    (10000)


static esp_adc_cal_characteristics_t *adc_chars;
static const adc_channel_t channel = ADC_CHANNEL_6;     //GPIO34 if ADC1, GPIO14 if ADC2
static const adc_channel_t channe2 = ADC_CHANNEL_7;     //GPIO35 if ADC1, GPIO27 if ADC2

static const adc_atten_t atten = ADC_ATTEN_DB_0;//ADC_ATTEN_DB_0
static const adc_unit_t unit = ADC_UNIT_1;

static void check_efuse()
{
    //Check TP is burned into eFuse
    if (esp_adc_cal_check_efuse(ESP_ADC_CAL_VAL_EFUSE_TP) == ESP_OK) {
        printf("eFuse Two Point: Supported\n");
    } else {
        printf("eFuse Two Point: NOT supported\n");
    }

    //Check Vref is burned into eFuse
    if (esp_adc_cal_check_efuse(ESP_ADC_CAL_VAL_EFUSE_VREF) == ESP_OK) {
        printf("eFuse Vref: Supported\n");
    } else {
        printf("eFuse Vref: NOT supported\n");
    }
}

static void print_char_val_type(esp_adc_cal_value_t val_type)
{
    if (val_type == ESP_ADC_CAL_VAL_EFUSE_TP) {
        printf("Characterized using Two Point Value\n");
    } else if (val_type == ESP_ADC_CAL_VAL_EFUSE_VREF) {
        printf("Characterized using eFuse Vref\n");
    } else {
        printf("Characterized using Default Vref\n");
    }
}

void app_main()
{
	int ch;

    /*
     * Prepare and set configuration of timers
     * that will be used by LED Controller
     */
    ledc_timer_config_t ledc_timer = {
        .duty_resolution = LEDC_TIMER_13_BIT, // resolution of PWM duty
        .freq_hz = 500,                      // frequency of PWM signal
        .speed_mode = LEDC_HS_MODE,           // timer mode
        .timer_num = LEDC_HS_TIMER            // timer index
    };
    // Set configuration of timer0 for high speed channels
    ledc_timer_config(&ledc_timer);

    // Prepare and set configuration of timer1 for low speed channels
    ledc_timer.speed_mode = LEDC_LS_MODE;
    ledc_timer.timer_num = LEDC_LS_TIMER;
    ledc_timer_config(&ledc_timer);

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
    ledc_channel_config_t ledc_channel[LEDC_TEST_CH_NUM] = {
        {
            .channel    = LEDC_HS_CH0_CHANNEL,
            .duty       = 0,
            .gpio_num   = LEDC_HS_CH0_GPIO,
            .speed_mode = LEDC_HS_MODE,
            .hpoint     = 0,
            .timer_sel  = LEDC_HS_TIMER
        },
        {
            .channel    = LEDC_HS_CH1_CHANNEL,
            .duty       = 0,
            .gpio_num   = LEDC_HS_CH1_GPIO,
            .speed_mode = LEDC_HS_MODE,
            .hpoint     = 0,
            .timer_sel  = LEDC_HS_TIMER
        },
        {
            .channel    = LEDC_LS_CH2_CHANNEL,
            .duty       = 0,
            .gpio_num   = LEDC_LS_CH2_GPIO,
            .speed_mode = LEDC_LS_MODE,
            .hpoint     = 0,
            .timer_sel  = LEDC_LS_TIMER
        },
        {
            .channel    = LEDC_LS_CH3_CHANNEL,
            .duty       = 0,
            .gpio_num   = LEDC_LS_CH3_GPIO,
            .speed_mode = LEDC_LS_MODE,
            .hpoint     = 0,
            .timer_sel  = LEDC_LS_TIMER
        },
    };

    // Set LED Controller with previously prepared configuration
    for (ch = 0; ch < LEDC_TEST_CH_NUM; ch++) {
        ledc_channel_config(&ledc_channel[ch]);
    }

	ledc_fade_func_install(0);
	
    //Check if Two Point or Vref are burned into eFuse
    check_efuse();

    //Configure ADC
    if (unit == ADC_UNIT_1) {
        adc1_config_width(ADC_WIDTH_BIT_12);
        adc1_config_channel_atten(channel, atten);
		adc1_config_channel_atten(channe2, atten);
    } else {
        adc2_config_channel_atten((adc2_channel_t)channel, atten);
    }

    //Characterize ADC
    adc_chars = calloc(1, sizeof(esp_adc_cal_characteristics_t));
    esp_adc_cal_value_t val_type = esp_adc_cal_characterize(unit, atten, ADC_WIDTH_BIT_12, DEFAULT_VREF, adc_chars);
    print_char_val_type(val_type);
	dac_output_enable(DAC_CHANNEL_1);// DAC channel 1 is GPIO25
	dac_output_enable(DAC_CHANNEL_2);
	dac_output_voltage(DAC_CHANNEL_2, 100);
	dac_output_voltage(DAC_CHANNEL_1, 200);
	printf("dac1value: %d\n", 200);
    //Continuously sample ADC1
    while (1) {
        uint32_t adc_reading = 0;
		uint32_t adc_reading2 = 0;
        //Multisampling
        for (int i = 0; i < NO_OF_SAMPLES; i++) {
            if (unit == ADC_UNIT_1) {
                adc_reading += adc1_get_raw((adc1_channel_t)channel);
				adc_reading2 += adc1_get_raw((adc1_channel_t)channe2);
            } else {
                int raw;
                adc2_get_raw((adc2_channel_t)channel, ADC_WIDTH_BIT_12, &raw);
                adc_reading += raw;
            }
        }
        adc_reading /= NO_OF_SAMPLES;
		adc_reading2 /= NO_OF_SAMPLES;
        //Convert adc_reading to voltage in mV
        uint32_t voltage = esp_adc_cal_raw_to_voltage(adc_reading, adc_chars);
        printf("Raw6: %d\tVoltage6: %dmV\n", adc_reading, voltage);
		voltage = esp_adc_cal_raw_to_voltage(adc_reading2, adc_chars);
        printf("Raw7: %d\tVoltage7: %dmV\n", adc_reading2, voltage);
								
		int sersorval = hall_sensor_read();//ADC1的通道0和3（GPIO 36和39）
											//内部使用 这两个引脚不能接任何东西
		printf("sersorval: %d\n", sersorval);

		printf("1. LEDC fade up to duty = %d\n", LEDC_TEST_DUTY);
        for (ch = 0; ch < LEDC_TEST_CH_NUM; ch++) {
            ledc_set_fade_with_time(ledc_channel[ch].speed_mode,
                    ledc_channel[ch].channel, LEDC_TEST_DUTY, LEDC_TEST_FADE_TIME);
            ledc_fade_start(ledc_channel[ch].speed_mode,
                    ledc_channel[ch].channel, LEDC_FADE_NO_WAIT);
        }
        vTaskDelay(LEDC_TEST_FADE_TIME / portTICK_PERIOD_MS);
		dac_output_voltage(DAC_CHANNEL_1, 100);
	    printf("dacvalue: %d\n", 100);
       // vTaskDelay(pdMS_TO_TICKS(1000));
		printf("2. LEDC fade down to duty = 0\n");
        for (ch = 0; ch < LEDC_TEST_CH_NUM; ch++) {
            ledc_set_fade_with_time(ledc_channel[ch].speed_mode,
                    ledc_channel[ch].channel, 0, LEDC_TEST_FADE_TIME);
            ledc_fade_start(ledc_channel[ch].speed_mode,
                    ledc_channel[ch].channel, LEDC_FADE_NO_WAIT);
        }
        vTaskDelay(LEDC_TEST_FADE_TIME / portTICK_PERIOD_MS);

		dac_output_voltage(DAC_CHANNEL_1, 50);
	    printf("dacvalue: %d\n", 50);
        //printf("3. LEDC set duty = %d without fade\n", LEDC_TEST_DUTY);
        //for (ch = 0; ch < LEDC_TEST_CH_NUM; ch++) {
        //    ledc_set_duty(ledc_channel[ch].speed_mode, ledc_channel[ch].channel, LEDC_TEST_DUTY);
        //    ledc_update_duty(ledc_channel[ch].speed_mode, ledc_channel[ch].channel);
        //}
        //vTaskDelay(1000 / portTICK_PERIOD_MS);

        printf("4. LEDC set duty = 0 without fade\n");
        for (ch = 0; ch < LEDC_TEST_CH_NUM; ch++) {
            ledc_set_duty(ledc_channel[ch].speed_mode, ledc_channel[ch].channel, 0);
            ledc_update_duty(ledc_channel[ch].speed_mode, ledc_channel[ch].channel);
        }
		vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
}




