/* BI_LC_SIM 800
test
*/
#include "freertos/FreeRTOS.h"

#include <stdio.h>
#include <string.h>
#include <sys/unistd.h>
#include <sys/stat.h>
#include "esp_wifi.h"
#include "esp_system.h"
#include "esp_event.h"
#include "esp_event_loop.h"
#include "esp_err.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "driver/gpio.h"
#include "sdkconfig.h"


#include "esp_console.h"
#include "gsm.h"
#define BUF_SIZE (1024)



#include <time.h>
#include <sys/time.h>






/* sntp */
#include "esp_sntp.h"

/* i2c */
#include <ads111x.h>
#include <ds3231.h>
#define I2C_PORT 0
#define SDA_GPIO 21
#define SCL_GPIO 22

#define GAIN ADS111X_GAIN_4V096		// +-4.096V
static float gain_val;				// Gain value

// I2C addresses - ���������� ������������� ������ ��� �������, ������� ������������
#define DEV_COUNT 1   // 1 ICs
static const uint8_t addr[DEV_COUNT] = {ADS111X_ADDR_GND, ADS111X_ADDR_VCC, ADS111X_ADDR_SDA, ADS111X_ADDR_SCL};


// Descriptors
static i2c_dev_t devices[DEV_COUNT];



// ����� ��� �������� ������, ������������� �� sd-�����


// setup datetime: 2022-01-19 13:50:10
struct tm time_DS3231 = {
    .tm_year = 122, //since 1900 (2022 - 1900)
    .tm_mon  = 0,  // 0-based
    .tm_mday = 19,
    .tm_hour = 13,
    .tm_min  = 50,
    .tm_sec  = 10
};
float tempature_DS3231;

/* GPIO */
#define GPIO_Pwr_On		25
#define Pwr_On			GPIO_Pwr_On,1
#define Pwr_Off			GPIO_Pwr_On,0

/* LED */
#define GPIO_Led_1		26
#define Led_1_On		GPIO_Led_1,0
#define Led_1_Off		GPIO_Led_1,1

/* SIM800 */
#define GPIO_SIM_Stat	35
#define GPIO_SIM_Rst	23


/* GPIO */
#define set_GPIO()	do { \
	gpio_reset_pin(CONFIG_K1_GPIO);\
	gpio_reset_pin(CONFIG_K2_GPIO);\
	gpio_set_direction(CONFIG_K1_GPIO, GPIO_MODE_OUTPUT);\
	gpio_set_direction(CONFIG_K2_GPIO, GPIO_MODE_OUTPUT);\
	gpio_set_level(CONFIG_K1_GPIO, 1);\
	gpio_set_level(CONFIG_K2_GPIO, 1);\
    } while(0)



/* mb */
//#include "mbcontroller.h"       // for mbcontroller defines and api
//#include "modbus_params.h"      // for modbus parameters structures
//#include "mb_master.h"      // for modbus parameters structures

extern esp_err_t master_init(void);
extern void master_operation_func(void *arg);




holding_reg_params_t holding_reg_params = { 0 };
input_reg_params_t input_reg_params = { 0 };
coil_reg_params_t coil_reg_params = { 0 };
discrete_reg_params_t discrete_reg_params = { 0 };


#ifndef APP_CPU_NUM
	#define APP_CPU_NUM PRO_CPU_NUM
#endif
#define portTICK_PERIOD_MS			( ( TickType_t ) 1000 / configTICK_RATE_HZ )
SemaphoreHandle_t print_mux = NULL;




#include <expat.h>
#include <sdcard.h>

extern void sdcard_task(void *pvParameters);


//static const char *TAG_BI = "BI_LC";
//static const char *TAG_SNTP = "SNTP";
static const char *TAG_SD = "SD";
//static const char *TAG_MB = "MB";
static const char *TAG_SIM = "SIM800";



esp_err_t event_handler(void *ctx, system_event_t *event){
    return ESP_OK;
}





void ds3231_task(void *pvParameters) {
    i2c_dev_t dev;
    memset(&dev, 0, sizeof(i2c_dev_t));

    ESP_ERROR_CHECK(ds3231_init_desc(&dev, 0, SDA_GPIO, SCL_GPIO));
    ESP_ERROR_CHECK(ds3231_set_time(&dev, &time_DS3231));

    while (1) {
        if (ds3231_get_temp_float(&dev, &tempature_DS3231) != ESP_OK) {
            printf("Could not get temperature\n");
            continue;
        }

        if (ds3231_get_time(&dev, &time_DS3231) != ESP_OK) {
            printf("Could not get time\n");
            continue;
        }

        /* float is used in printf(). you need non-default configuration in
         * sdkconfig for ESP8266, which is enabled by default for this
         * example. see sdkconfig.defaults.esp8266 todo
         */
        printf("%04d-%02d-%02d %02d:%02d:%02d, %.2f deg Cel\n", time_DS3231.tm_year + 1900 /*Add 1900 for better readability*/, time_DS3231.tm_mon + 1,
            time_DS3231.tm_mday, time_DS3231.tm_hour, time_DS3231.tm_min, time_DS3231.tm_sec, tempature_DS3231);
        vTaskDelay(pdMS_TO_TICKS(500));
    }
}

static void measure(size_t n) {
    bool busy;
    do {	// wait for conversion end
        ads111x_is_busy(&devices[n], &busy);
    }
    while (busy);

    int16_t raw = 0;
    if (ads111x_get_value(&devices[n], &raw) == ESP_OK) {
        float voltage = gain_val / ADS111X_MAX_VALUE * raw;
        printf("[%u] Raw ADC value: %d, voltage: %.04f volts\n", n, raw, voltage);
    }else
        printf("[%u] Cannot read ADC value\n", n);
}

void ads111x_task(void *pvParameters){
    gain_val = ads111x_gain_values[GAIN];
    // Setup ICs
    for (size_t i = 0; i < DEV_COUNT; i++) {
        ESP_ERROR_CHECK(ads111x_init_desc(&devices[i], addr[i], I2C_PORT, SDA_GPIO, SCL_GPIO));

        ESP_ERROR_CHECK(ads111x_set_mode(&devices[i], ADS111X_MODE_CONTINUOUS));    // Continuous conversion mode
        ESP_ERROR_CHECK(ads111x_set_data_rate(&devices[i], ADS111X_DATA_RATE_860));
        ESP_ERROR_CHECK(ads111x_set_input_mux(&devices[i], ADS111X_MUX_1_3));
        ESP_ERROR_CHECK(ads111x_set_gain(&devices[i], GAIN));
    }

    while (1) {
        for (size_t i = 0; i < DEV_COUNT; i++) measure(i);
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

void sim800_task(void *pvParameters){
	char *data_SIM = (char *) malloc(BUF_SIZE);
    while (1) {
        vTaskDelay(pdMS_TO_TICKS(10));
		data_SIM = gsmRead();
	    if (data_SIM != NULL){
	    	esp_log_write(ESP_LOG_INFO,TAG_SIM, data_SIM, esp_log_timestamp());
	    }
    }
}



void app_main(void)
{
	// ���������� GPIO
	// ��������� ����������� ��
	gpio_reset_pin(GPIO_Pwr_On);	gpio_set_direction(GPIO_Pwr_On, GPIO_MODE_OUTPUT);		gpio_set_level(Pwr_On);
	// ���������
	gpio_reset_pin(GPIO_Led_1);		gpio_set_direction(GPIO_Led_1, GPIO_MODE_OUTPUT);		gpio_set_level(GPIO_Led_1, 1);
	// ���������� SIM800
	gpio_reset_pin(GPIO_SIM_Stat);	gpio_set_direction(GPIO_SIM_Stat, GPIO_MODE_INPUT);		gpio_set_level(GPIO_SIM_Stat, 1); gpio_set_pull_mode(GPIO_SIM_Stat, GPIO_PULLUP_ONLY);
	gpio_reset_pin(GPIO_SIM_Rst);	gpio_set_direction(GPIO_SIM_Rst, GPIO_MODE_OUTPUT_OD);	gpio_set_level(GPIO_SIM_Rst, 1);

	// ������������� SIM
//	gsmInit(17, 16, 115200, 1);
//    xTaskCreate(sim800_task, "sim800_task", configMINIMAL_STACK_SIZE * 8, NULL, 7, NULL);
//    gsmWrite("AT\n");
//	vTaskDelay(100 / portTICK_PERIOD_MS);
//    gsmWrite("AT+CSQ\n");
//	vTaskDelay(100 / portTICK_PERIOD_MS);
//    gsmWrite("AT+CGATT?\n");
//	vTaskDelay(100 / portTICK_PERIOD_MS);
//    gsmWrite("AT+CGATT=1\n");
//	vTaskDelay(100 / portTICK_PERIOD_MS);
//    gsmWrite("AT+CIPQSEND=1\n");
//	vTaskDelay(100 / portTICK_PERIOD_MS);
//    gsmWrite("AT+CIPHEAD=1\n");
//	vTaskDelay(100 / portTICK_PERIOD_MS);
//    gsmWrite("AT+CIPHEAD=1\n");
//	vTaskDelay(100 / portTICK_PERIOD_MS);
//    gsmWrite("AT+CIPSRIP=0\n");
//	vTaskDelay(100 / portTICK_PERIOD_MS);
//    gsmWrite("AT+CSTT=\"VPN12.VOLGA\",\"\",\"\"\n");
//	vTaskDelay(100 / portTICK_PERIOD_MS);
//    gsmWrite("AT+CIICR\n");
//	vTaskDelay(1000 / portTICK_PERIOD_MS);
//    gsmWrite("AT+CIFSR\n");
//	vTaskDelay(1000 / portTICK_PERIOD_MS);
//    gsmWrite("AT+CIPSERVER=1,502\n");
//	vTaskDelay(1000 / portTICK_PERIOD_MS);
//    gsmWrite("AT+CIPSTATUS\n");
//	vTaskDelay(100 / portTICK_PERIOD_MS);
//    gsmWrite("AT\n");
//	vTaskDelay(100 / portTICK_PERIOD_MS);
//    gsmWrite("AT\n");
//	vTaskDelay(100 / portTICK_PERIOD_MS);




//    ESP_ERROR_CHECK(i2cdev_init());
//    memset(devices, 0, sizeof(devices));	// Clear device descriptors

    ESP_ERROR_CHECK(i2cdev_init());
    xTaskCreate(sdcard_task, "sdcard_task", configMINIMAL_STACK_SIZE * 8, NULL, 6, NULL);
//    xTaskCreate(ds3231_task, "ds3231_task", configMINIMAL_STACK_SIZE * 3, NULL, 5, NULL);
//    xTaskCreatePinnedToCore(ads111x_task, "ads111x_task", configMINIMAL_STACK_SIZE * 8, NULL, 4, NULL, APP_CPU_NUM);


//
//    esp_console_repl_t *repl = NULL;
//    esp_console_repl_config_t repl_config = ESP_CONSOLE_REPL_CONFIG_DEFAULT();
//    esp_console_dev_uart_config_t uart_config = ESP_CONSOLE_DEV_UART_CONFIG_DEFAULT();
//    ESP_ERROR_CHECK(esp_console_new_repl_uart(&uart_config, &repl_config, &repl));
//    // start console REPL
//    ESP_ERROR_CHECK(esp_console_start_repl(repl));

    // Initialization of device peripheral and objects
    ESP_ERROR_CHECK(master_init());
    vTaskDelay(10);

    master_operation_func(NULL);


//	// ������, ���� ���� �����. ����� ���� ��������� ������. todo
	int level = 0;
	while (true) {
		gpio_set_level(GPIO_Led_1, level);
		level = !level;
		vTaskDelay(300 / portTICK_PERIOD_MS);
    }
}
