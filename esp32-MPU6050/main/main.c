/**
* Company: 
* Engineer:         Leandro Santos
* Create Date:      06/01/2021 
* Design Name:      ESP32-MPU6050

* Target Devices:   ESP32
* Tool versions:    ESP-IDF(v4.3.1) 
* Description:      ESP32 I2C communication according to ESP-IDF documentation 
*                   for MPU6050 sensor 
*               
* Dependencies: 
*
* Revision: 
* Revision 0.01 - File Created
* Additional Comments: 
 **/
#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "driver/i2c.h"
#include "esp_log.h"
#include "esp_system.h"
#include "nvs_flash.h"
#include "sdkconfig.h"

/********************************************************************************************
*                     Global defines - Handles and Function prototypes                      *
*********************************************************************************************/
#define I2C_MASTER_SCL_IO   22          /*!< gpio number for I2C master clock */
#define I2C_MASTER_SDA_IO   21          /*!< gpio number for I2C master data  */
#define I2C_MASTER_NUM      I2C_NUM_0   /*!< I2C port number for master dev */
#define I2C_MASTER_FREQ_HZ  100000      /*!< I2C master clock frequency */

#define MPU_I2C         0x68    // Endereco i2c do mpu6050

#define accel_fs_2G          0           /*!< Accelerometer full scale range is +/- 2g */
#define accel_fs_4G          1           /*!< Accelerometer full scale range is +/- 4g */
#define accel_fs_8G          2           /*!< Accelerometer full scale range is +/- 8g */
#define accel_fs_16G         3           /*!< Accelerometer full scale range is +/- 16g */

#define ACCEL_CONFIG_REG    0x1C
#define ACCEL_X_HIGH        0x3B
#define ACCEL_X_LOW         0x3C
#define MPU6050_PWR_MGMT_1  0x6B

#define TEMP_X_HIGH         0x41
#define TEMP_X_LOW          0x42

typedef struct _mpu6050_acceleration_t
{
    float accel_x;
    float accel_y;
    float accel_z;
} mpu6050_acceleration_t;

static const char *TAG = "esp32-i2c";

/**
 * Initialize I2C 
 */
void I2C_INIT(void)
{
    i2c_config_t conf;

    conf.mode = I2C_MODE_MASTER;
    conf.sda_io_num = (gpio_num_t)I2C_MASTER_SDA_IO;
    conf.scl_io_num = (gpio_num_t)I2C_MASTER_SCL_IO;
    conf.sda_pullup_en = GPIO_PULLUP_ENABLE;
    conf.scl_pullup_en = GPIO_PULLUP_ENABLE;
    conf.master.clk_speed = I2C_MASTER_FREQ_HZ;
    conf.clk_flags = I2C_SCLK_SRC_FLAG_FOR_NOMAL;

    ESP_ERROR_CHECK ( i2c_param_config (I2C_MASTER_NUM, &conf) );
    ESP_ERROR_CHECK ( i2c_driver_install (I2C_MASTER_NUM, I2C_MODE_MASTER, 0, 0, 0));
    
    vTaskDelay(200 / portTICK_PERIOD_MS);

    ESP_LOGI(TAG, "\t*** I2C INITIALIZE (CONCLUDED) ***\n");
}

/**
 * I2C start communication method 
 */
void i2c_start_register(uint8_t device_address, uint8_t register_address)
{
	i2c_cmd_handle_t cmd = i2c_cmd_link_create();
	i2c_master_start(cmd);
	i2c_master_write_byte(cmd, (device_address << 1) | I2C_MASTER_WRITE, 1);
	i2c_master_write_byte(cmd, register_address, 1);
	i2c_master_stop(cmd);
	i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, 1000 / portTICK_PERIOD_MS);
	i2c_cmd_link_delete(cmd);
}

/**
 * Initializing MPU6050 device with basic configuration 
 */
void MPU6050_INIT(void)
{
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    ESP_ERROR_CHECK( i2c_master_start(cmd));
    ESP_ERROR_CHECK( i2c_master_write_byte(cmd, (MPU_I2C << 1) | I2C_MASTER_WRITE, 1));
    ESP_ERROR_CHECK( i2c_master_write_byte(cmd, MPU6050_PWR_MGMT_1, 1));
    ESP_ERROR_CHECK( i2c_master_write_byte(cmd, 0, 1));
    ESP_ERROR_CHECK( i2c_master_stop(cmd));
    
    ESP_ERROR_CHECK( i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, 1000/portTICK_PERIOD_MS));
    i2c_cmd_link_delete(cmd);
}

/**
 * Reading MPU6050 temperature 
 */
float mpu6050_get_temperature(void)
{
    uint8_t data[2];

    i2c_start_register(MPU_I2C, TEMP_X_HIGH);

    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    ESP_ERROR_CHECK( i2c_master_start(cmd));
    ESP_ERROR_CHECK( i2c_master_write_byte(cmd, (MPU_I2C << 1) | I2C_MASTER_READ, true));
    ESP_ERROR_CHECK( i2c_master_read_byte(cmd, data+0, 0));    
    ESP_ERROR_CHECK( i2c_master_read_byte(cmd, data+1, 1));    

    ESP_ERROR_CHECK( i2c_master_stop(cmd));
    ESP_ERROR_CHECK( i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, 1000 / portTICK_RATE_MS));
    i2c_cmd_link_delete(cmd);
    
    return (((int16_t)((data[0] << 8) + (data[1]))/340) + 36.53);
}

/**
 * Reading MPU6050 accelerometer full-scale range sensitivity configuration 
 */
float mpu6050_get_accel_sensitivity(void)
{
    uint8_t accel_fs[0];
    float accel_sensitivity;
 
    // Reading Accelerometer sensitivity configuration on MPU6050 
    i2c_start_register(MPU_I2C, ACCEL_CONFIG_REG);
    
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    ESP_ERROR_CHECK( i2c_master_start(cmd));
    ESP_ERROR_CHECK( i2c_master_write_byte(cmd, (MPU_I2C << 1) | I2C_MASTER_READ, true));
    ESP_ERROR_CHECK( i2c_master_read_byte(cmd, accel_fs+0, 1));

    ESP_ERROR_CHECK( i2c_master_stop(cmd));
    ESP_ERROR_CHECK( i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, 1000 / portTICK_RATE_MS));
    i2c_cmd_link_delete(cmd);
    
    // Calculating full-scale range (by accelerometer sensitivity register configuration on MPU6050) 
    accel_fs[0] = (accel_fs[0] >> 3) & 0x03;
    
    switch (accel_fs[0]) {
        case accel_fs_2G:
            accel_sensitivity = 16384;
            break;

        case accel_fs_4G:
            accel_sensitivity = 8192;
            break;

        case accel_fs_8G:
            accel_sensitivity = 4096;
            break;

        default:
            accel_sensitivity = 2048;
            break;
    }
    // ESP_LOGI(TAG, "\t*** accel_sensitivity: %d\n", (uint16_t)accel_sensitivity);
    return accel_sensitivity;
}

/**
 * Reading MPU6050 accelerometer data 
 */
mpu6050_acceleration_t mpu6050_get_acceleration(void)
{
    mpu6050_acceleration_t accel_values;
    uint8_t data[7];

    float accel_sensitivity = mpu6050_get_accel_sensitivity();
    
    //Initializing MPU6050 Accelerometer read data sequence
    i2c_start_register(MPU_I2C, ACCEL_X_HIGH);
    
    //Reading MPU6050 accelerometer data
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    ESP_ERROR_CHECK( i2c_master_start(cmd) );
    ESP_ERROR_CHECK( i2c_master_write_byte(cmd, (MPU_I2C << 1) | I2C_MASTER_READ, 1));
    ESP_ERROR_CHECK( i2c_master_read_byte(cmd, data+0, 0));    
    ESP_ERROR_CHECK( i2c_master_read_byte(cmd, data+1, 0));    
    ESP_ERROR_CHECK( i2c_master_read_byte(cmd, data+2, 0));    
    ESP_ERROR_CHECK( i2c_master_read_byte(cmd, data+3, 0));    
    ESP_ERROR_CHECK( i2c_master_read_byte(cmd, data+4, 0));    
    ESP_ERROR_CHECK( i2c_master_read_byte(cmd, data+5, 1));    
    ESP_ERROR_CHECK (i2c_master_stop(cmd) );

    ESP_ERROR_CHECK (i2c_master_cmd_begin (I2C_MASTER_NUM, cmd, 1000/portTICK_RATE_MS));
    i2c_cmd_link_delete(cmd);

    // Applying full-scale range on each Accelerometer read data
    accel_values.accel_x = (int16_t)((data[0] << 8) + (data[1])) / accel_sensitivity;
    accel_values.accel_y = (int16_t)((data[2] << 8) + (data[3])) / accel_sensitivity;
    accel_values.accel_z = (int16_t)((data[4] << 8) + (data[5])) / accel_sensitivity;
    
    return accel_values;
}

/********************************************************************************************
*                   FreeRTOS section - Handles and Function prototypes                      *
*********************************************************************************************/
void task_mpu6050(void *params)
{
    mpu6050_acceleration_t accel_values;
    float temperature;

    while (1)
    {
        accel_values = mpu6050_get_acceleration();
        temperature =  mpu6050_get_temperature();

        ESP_LOGI(TAG, "\t       X: %f | Y: %f | Z: %f", accel_values.accel_x, accel_values.accel_y, accel_values.accel_z);
        ESP_LOGI(TAG, "\tTemp(°C): %.2f", temperature);

        vTaskDelay(500 / portTICK_PERIOD_MS);
    }
}

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

    I2C_INIT();
    
    MPU6050_INIT();

    xTaskCreate(&task_mpu6050, "Task MPU6050", 4096, NULL, 5, NULL);

    while(1) {
        vTaskDelay(1000/portTICK_PERIOD_MS);
    }
}