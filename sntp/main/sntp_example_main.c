/**
* Company: 
* Engineer:      Leandro Santos
* Create Date:   22/02/2022 
* Design Name:   ESP32 System Time 

* Target Devices: ESP32
* Tool versions:  ESP-IDF(v4.3.1) 
* Description:  ESP32 SNTP simple example with time zone configured to Sao Paulo (BRA)
*
*               
* Dependencies: esp_sntp
*
* Revision: 
* Revision 0.01 - File Created
* Additional Comments: 
 **/

#include <string.h>
#include <time.h>
#include <sys/time.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "esp_system.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_attr.h"
#include "esp_sleep.h"
#include "nvs_flash.h"
#include "protocol_examples_common.h"
#include "esp_sntp.h"



/********************************************************************************************
*                              Handles and Function prototypes                              *
*********************************************************************************************/
static const char *TAG = "SNTP_APP";

#define strftime_buf_size (64)

time_t now;
struct tm timeinfo;
char strftime_buf[strftime_buf_size];

void get_sntp_time(char *strftime_buf, time_t *now, struct tm *timeinfo);
void initialize_sntp(void);


/**
 * @brief SNTP calback function
 * 
 * @param tv - Structure returned by gettimeofday(2) system call, and used in other calls.
 */
static void time_sync_notification_cb(struct timeval *tv)
{
    ESP_LOGI(TAG, "Notification of a time synchronization event");
}


/**
 * @brief Initialize SNTP service
 * 
 */
void initialize_sntp(void)
{
    int retry = 0;
    const int retry_count = 10;

    ESP_LOGI(TAG, "Initializing SNTP");
    sntp_setoperatingmode(SNTP_OPMODE_POLL);
    sntp_setservername(0, "pool.ntp.org");
    sntp_set_time_sync_notification_cb(time_sync_notification_cb);
    sntp_init();

    // wait for time to be set
    while (sntp_get_sync_status() == SNTP_SYNC_STATUS_RESET && ++retry < retry_count) {
        ESP_LOGI(TAG, "Waiting for system time to be set... (%d/%d)", retry, retry_count);
        vTaskDelay(2000 / portTICK_PERIOD_MS);
    }
  
}


/**
 * @brief Get the sntp time object
 * 
 * @param strftime_buf[] - full char array with current date / time   
 * @param now          - Time structure based ESP-IDF device 
 * @param timeinfo     - SNTP time data structure
 */
void get_sntp_time(char *strftime_buf, time_t *now, struct tm *timeinfo)
{
    time(now);

    // Set timezone to São Paulo Standard Time
    setenv("TZ", "BRST+3BRDT+2,M10.3.0,M2.3.0", 1);
    tzset();
 
    localtime_r(now, timeinfo);
    strftime(strftime_buf, strftime_buf_size, "%c", timeinfo);
    ESP_LOGI(TAG, "The current date/time in Sao Paulo(BRA) is: %s", strftime_buf);
}


/********************************************************************************************
*                                  Application section                                      *
*********************************************************************************************/
void app_main(void)
{
    ESP_ERROR_CHECK( nvs_flash_init() );
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK( esp_event_loop_create_default() );

    /* This helper function configures Wi-Fi or Ethernet, as selected in menuconfig.
     * Read "Establishing Wi-Fi or Ethernet Connection" section in
     * examples/protocols/README.md for more information about this function.
     */
    ESP_ERROR_CHECK(example_connect());

    initialize_sntp();

    while (pdTRUE)
    {
        get_sntp_time(&strftime_buf, &now, &timeinfo);
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
}