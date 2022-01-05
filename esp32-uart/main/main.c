/**
* Company: 
* Engineer:      Leandro Santos
* Create Date:   12/03/2021 
* Design Name:   ESP32-UART communication

* Target Devices: ESP32
* Tool versions:  ESP-IDF(v4.3.1) 
* Description:  ESP32 uart communication according ESP-IDF documentation 
*               with LED status (controlled by TIMER and UART command)
*               and Tactile switch status
*               
* Dependencies: interface_app
*
* Revision: 
* Revision 0.01 - File Created
* Additional Comments: 
 **/

#include <string.h>
#include <stdio.h>
#include "nvs_flash.h"
#include "esp_spi_flash.h"


#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"

#include "driver/uart.h"
#include "driver/gpio.h"

#include "esp_system.h"
#include "esp_event.h"          
#include "esp_log.h"            

#include "sdkconfig.h"

#include "interfaces_app.h"


/********************************************************************************************
*                   FreeRTOS section - Handles and Function prototypes                      *
*********************************************************************************************/
// #define BUF_SIZE (1024)

//Semaphore
SemaphoreHandle_t UartSemaphore;

// Queue handles
QueueHandle_t UART_rx_data_queue = NULL;
QueueHandle_t UART_tx_data_queue = NULL;

QueueHandle_t tactile_sw_evt_queue = NULL;
QueueHandle_t led_evt_queue = NULL;

/********************************************************************************************
*                                  Application section                                      *
*********************************************************************************************/
void app_main(void)
{
    /* Print chip information */
    esp_chip_info_t chip_info;
    esp_chip_info(&chip_info);
    printf("This is ESP32 chip with %d CPU cores, WiFi%s%s, ",
            chip_info.cores,
            (chip_info.features & CHIP_FEATURE_BT) ? "/BT" : "",
            (chip_info.features & CHIP_FEATURE_BLE) ? "/BLE" : "");

    printf("silicon revision %d, ", chip_info.revision);

    printf("%dMB %s flash\n", spi_flash_get_chip_size() / (1024 * 1024),
            (chip_info.features & CHIP_FEATURE_EMB_FLASH) ? "embedded" : "external");
        
    // Initialize NVS
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        // NVS partition was truncated and needs to be erased
        // Retry nvs_flash_init
        ESP_ERROR_CHECK(nvs_flash_erase());
        err = nvs_flash_init();
    }
    ESP_ERROR_CHECK( err );

    // Initialize TIMER and GPIOs    
    USER_GPIO_INIT();
    USER_TIMER_INIT();
    
    //create a queue to handle Serial received data
    UART_rx_data_queue = xQueueCreate(10, sizeof(BUF_SIZE));
    UART_tx_data_queue = xQueueCreate(10, sizeof(uint32_t));
    
    //Initialize FreeRTOS Semaphore
    UartSemaphore = xSemaphoreCreateBinary();
    xSemaphoreGive(UartSemaphore);

    // Serial(RX) task
    if (xTaskCreate(uart_read_app_task, "uart_read_app_task", ECHO_TASK_STACK_SIZE, NULL, 5, NULL) != pdTRUE)
    {
        ESP_LOGE("ERROR", "*** uart_read_app_task error ***\n");
    }

    // Led status task controlled by serial command event
    if (xTaskCreate(LedCtrlTask, "LedCtrlTask", configMINIMAL_STACK_SIZE + 2048, NULL, 6, NULL) != pdTRUE)
    {
        ESP_LOGE("ERROR", "*** LedCtrlTask error ***\n");
    }

    // Serial(TX) task
    if (xTaskCreate(uart_write_app_task, "uart_write_app_task", ECHO_TASK_STACK_SIZE, NULL, 5, NULL) != pdTRUE)
    {
        ESP_LOGE("ERROR", "*** uart_write_app_task error ***\n");
    }
}
