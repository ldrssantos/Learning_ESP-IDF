#ifndef INTERFACES_APP_N
    #define INTERFACES_APP_N
    
    #define ESP32_BLINK_GPIO GPIO_NUM_13
    #define RED_LED_GPIO GPIO_NUM_19
    #define TACTILE_SW_GPIO GPIO_NUM_4

    #define ADC_POT ADC1_CHANNEL_4

    #define ESP_INTR_FLAG_DEFAULT 0

    #define TIMER_DIVIDER (16)
    #define TIMER_SCALE (TIMER_BASE_CLK / TIMER_DIVIDER)

    // #define ECHO_TEST_TXD (CONFIG_EXAMPLE_UART_TXD)
    // #define ECHO_TEST_RXD (CONFIG_EXAMPLE_UART_RXD)
    // #define ECHO_TEST_RTS (UART_PIN_NO_CHANGE)
    // #define ECHO_TEST_CTS (UART_PIN_NO_CHANGE)

    // #define ECHO_UART_PORT_NUM      (CONFIG_EXAMPLE_UART_PORT_NUM)
    // #define ECHO_UART_BAUD_RATE     (CONFIG_EXAMPLE_UART_BAUD_RATE)
    // #define ECHO_TASK_STACK_SIZE    (CONFIG_EXAMPLE_TASK_STACK_SIZE)

    #define ECHO_TEST_TXD (1)
    #define ECHO_TEST_RXD (3)
    #define ECHO_TEST_RTS (-1)
    #define ECHO_TEST_CTS (-1)

    #define ECHO_UART_PORT_NUM      (0)
    #define ECHO_UART_BAUD_RATE     (115200)
    #define ECHO_TASK_STACK_SIZE    (2048)

    #define UART_DATA_MAX  (5)

    #define BUF_SIZE (1024)
    #define INTERFACE_SIZE (3)


    // TIMER Structure for our code
    typedef struct{
        int timer_group;
        int timer_idx;
        int alarm_interval;
        bool auto_reload;
    } esp_timer_info_t;

    // Timer event structure 
    typedef struct{
        esp_timer_info_t info;
        uint64_t timer_counter_value;
    }esp_timer_event_t;
       
    void USER_GPIO_INIT(void);
    void USER_TIMER_INIT(void);
    void USER_ADC_INIT(void);

    uint32_t user_adc_read(void);

    void uart_read_app_task(void *pvParameter);
    void uart_write_app_task(void *pvParameter);

    void LedCtrlTask(void *pvParameter);
#endif

