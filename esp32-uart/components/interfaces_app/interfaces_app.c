#include <stdio.h>
#include <string.h>

#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"

#include "esp_system.h"
#include "esp_log.h"

#include "driver/gpio.h"
#include "driver/timer.h"
#include "driver/uart.h"
#include <driver/adc.h>
#include "driver/ledc.h"

#include "math.h"

#include "interfaces_app.h"

/********************************************************************************************
*                     Global defines - Handles and Function prototypes                      *
*********************************************************************************************/
static const char *TAG = "Interfaces_app";
static const char *LedCtrl_ON = "LED=0";

bool LED_STATUS = false;
bool app_ctrl = false;

/********************************************************************************************
*             FreeRTOS section - Structures, variables and Function prototypes              *
*********************************************************************************************/
// Queue handles
extern QueueHandle_t UART_rx_data_queue;
extern QueueHandle_t UART_tx_data_queue;

//Semaphore
extern SemaphoreHandle_t UartSemaphore;

/**
 * UART Read data freeRTOS function prototype 
 */
void uart_read_app_task(void *pvParameter)
{
    // Configure a temporary buffer for the incoming data
    uint8_t *data = (uint8_t *) malloc(BUF_SIZE);

    while (true)
    {    
        if (xSemaphoreTake(UartSemaphore, portMAX_DELAY))
        {
            memset(data, 0, BUF_SIZE);
            
            // Read data from the UART
            int len = uart_read_bytes(ECHO_UART_PORT_NUM, data, (BUF_SIZE - 1), 20 / portTICK_RATE_MS);
            // Write data back to the UART
            //uart_write_bytes(ECHO_UART_PORT_NUM, (const char *) data, len);
            
            if (len >=5) {
                data[len] = '\0';
                xQueueSend(UART_rx_data_queue, &data, 10);
            }
            vTaskDelay(1 / portTICK_RATE_MS);
            xSemaphoreGive(UartSemaphore);
        }
    }
}

/**
 * UART write data freeRTOS function prototype 
 */
void uart_write_app_task(void *pvParameter)
{
    while (true)
    {    
        if (xSemaphoreTake(UartSemaphore, portMAX_DELAY))
        {
            char AppStatus[BUF_SIZE] = "ESP32 STATUS: ";

            // Configure a temporary buffer for the incoming data
            uint8_t *data = (uint8_t *) malloc(BUF_SIZE);

            if (xQueueReceive(UART_tx_data_queue, &data, 10))
            {
                if (LED_STATUS)
                {
                    strcat(AppStatus, " LED [1] - ");
                } else {
                    strcat(AppStatus, " LED [0] - ");
                }
                
                if (gpio_get_level(TACTILE_SW_GPIO))
                {
                    strcat(AppStatus, "BUTTON [1]");
                } else {
                    strcat(AppStatus, "BUTTON [0]");
                }
                
                ESP_LOGI(TAG, "%s\n", AppStatus);
                // Write data back to the UART
                uart_write_bytes(ECHO_UART_PORT_NUM, (const char *) AppStatus, BUF_SIZE);
            }
            xSemaphoreGive(UartSemaphore);
        }
    }
}


/**
 * Led status freeRTOS function prototype 
 */
void LedCtrlTask(void *pvParameter)
{
    while (true)
    {    
        // Configure a temporary buffer for the incoming data
        uint8_t *data = (uint8_t *) malloc(BUF_SIZE);
        
        if (xQueueReceive(UART_rx_data_queue, &data, 10))
        {
            if (strcmp(LedCtrl_ON, (const char *) data) == 0)
            {
                ESP_LOGI(TAG, "*** LedCtrl(ON) ***\n");
                gpio_set_level(ESP32_BLINK_GPIO, 1);
                LED_STATUS = true;
            } else {
                ESP_LOGI(TAG, "*** LedCtrl(OFF) ***\n");
                gpio_set_level(ESP32_BLINK_GPIO, 0);
                LED_STATUS = false;
            }
        }
    }
}


/********************************************************************************************
*                         Interface inicialization section                                  *
*********************************************************************************************/
void USER_GPIO_INIT(void)
{
    //insert atributes for tactile button 
    gpio_config_t sw_user_config = {
        .intr_type = GPIO_INTR_DISABLE,
        .mode = GPIO_MODE_INPUT,
        .pin_bit_mask = (1ull << TACTILE_SW_GPIO),
        .pull_down_en = 0,
        .pull_up_en = 1
    };
    //initilize tactile button 
    gpio_config(&sw_user_config);

    //insert atributes for tactile button 
    gpio_config_t led_user_config = {
        .intr_type = GPIO_INTR_DISABLE,
        .mode = GPIO_MODE_OUTPUT,
        .pin_bit_mask = (1ULL << ESP32_BLINK_GPIO) | (1ULL << RED_LED_GPIO),
        .pull_down_en = 0
    };
    //initilize LEDs (System and status)
    gpio_config(&led_user_config);

    /* Configure parameters of an UART driver,
     * communication pins and install the driver */
    uart_config_t uart_config = {
        .baud_rate = ECHO_UART_BAUD_RATE,
        .data_bits = UART_DATA_8_BITS,
        .parity    = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_APB,
    };
    int intr_alloc_flags = 0;

    #if CONFIG_UART_ISR_IN_IRAM
        intr_alloc_flags = ESP_INTR_FLAG_IRAM;
    #endif

    ESP_ERROR_CHECK(uart_driver_install(ECHO_UART_PORT_NUM, BUF_SIZE * 2, 0, 0, NULL, intr_alloc_flags));
    ESP_ERROR_CHECK(uart_param_config(ECHO_UART_PORT_NUM, &uart_config));
    ESP_ERROR_CHECK(uart_set_pin(ECHO_UART_PORT_NUM, ECHO_TEST_TXD, ECHO_TEST_RXD, ECHO_TEST_RTS, ECHO_TEST_CTS));
    
    ESP_LOGI(TAG, "\n*** USER_GPIO INITIALIZE (CONCLUDED) ***\n");
};

/**
 * TIMER callback function
 */
static bool IRAM_ATTR timer_group_isr_callback(void *args){
    BaseType_t high_task_awoken = pdFALSE;
    esp_timer_info_t *info = (esp_timer_event_t *) args;
    
    uint64_t timer_counter_value = timer_group_get_counter_value_in_isr(info->timer_group, info->timer_idx);

    if (!info->auto_reload){
        timer_counter_value += info->alarm_interval * TIMER_SCALE;
        timer_group_set_alarm_value_in_isr (info->timer_group, info->timer_idx, timer_counter_value);
    }

    app_ctrl = !app_ctrl;
    gpio_set_level(RED_LED_GPIO, app_ctrl);
    
    xQueueSend(UART_tx_data_queue, "0", NULL);

    return high_task_awoken == pdTRUE;
}

/**
 * Timer initialize
 */
void USER_TIMER_INIT(void){
    timer_config_t config = {
        .divider = TIMER_DIVIDER,        
        .counter_dir = TIMER_COUNT_UP,   
        .counter_en = TIMER_PAUSE,       
        .alarm_en = TIMER_ALARM_EN,      
        .auto_reload = true,             
    };
    timer_init(TIMER_GROUP_0, TIMER_0, &config);

    uint32_t intervalo_em_segundos = 1;

    timer_set_counter_value(TIMER_GROUP_0, TIMER_0, 0); 
    timer_set_alarm_value(TIMER_GROUP_0, TIMER_0, intervalo_em_segundos * TIMER_SCALE);
    timer_enable_intr(TIMER_GROUP_0, TIMER_0); 

    esp_timer_info_t *timer_info = calloc(1, sizeof(esp_timer_info_t));
    timer_info->timer_group = TIMER_GROUP_0;
    timer_info->timer_idx = TIMER_0;
    timer_info->auto_reload = true;
    timer_info->alarm_interval = intervalo_em_segundos; 
    timer_isr_callback_add(TIMER_GROUP_0, TIMER_0, timer_group_isr_callback, timer_info, 0);

    timer_start(TIMER_GROUP_0, TIMER_0);

    ESP_LOGI(TAG, "\n*** TIMER INITIALIZE (CONCLUDED) ***\n");
}




