//Architecture:
#define CORE_TYPE = ESP32_S3
#include "driver/gpio.h"
#include "driver/uart.h"
#include "driver/i2c.h"
//Use config.h if present, otherwise defaults
#if __has_include ( "config.h")
  #include "config.h"
#else
  #include "config.example.h"
#endif
#include "Dynamo_HW.h"

//ADC Settings:
#define ADC_DMAX 4095 //4095 in single shot, 8191 in continuous
#define ADC_VMAX 3.1 //Max readable voltage is actually 3.1v using mode ADC_ATTEN_DB_11  

//TTY settings:
#define TTY_BAUD 115200 //must match other endpoint
#define TTY_UART 0
#define TTY_TXD_PIN GPIO_NUM_43 //GPIO43(U0TXD)
#define TTY_RXD_PIN GPIO_NUM_44 //GPIO44(U0RXD?
#define TTY_TX_BUFF 256
#define TTY_RX_BUFF 4096

//I2C settings: 
#define I2C_MASTER false
#define I2C_SLAVE_ADDR 0x43
#define I2C_CLOCK 40000
#define I2C_SDA_PIN GPIO_NUM_17 //GPIO17
#define I2C_SCL_PIN GPIO_NUM_18 //GPIO18
#define I2C_SLAVE_PORT 0
#define I2C_TX_BUFF 256
#define I2C_RX_BUFF 4096

//Booster Control IO:
#define DIR_MONITOR GPIO_NUM_38 //GPIO38
#define DIR_OVERRIDE GPIO_NUM_21 //GPIO21
#define MASTER_EN GPIO_NUM_15 //GPIO15

//Track Configurations
//TrackChannel(int enable_out_pin, int reverse_pin, int brake_pin, int adc_pin, float adcscale, int tripMilliamps )
//TrackChannel(9, 6, 13, 1, 0, 4090); //Track 1
//TrackChannel(10, 7, 14, 2, 0, 4090); //Track 2
//TrackChannel(11, 8, 47, 3, 0, 4090); //Track 3
//TrackChannel(12, 9, 48, 4, 0, 4090); //Track 4

//GPIO Initialization
void gpio_init(){
    gpio_reset_pin(MASTER_EN);
    gpio_set_direction(MASTER_EN, GPIO_MODE_INPUT);
    gpio_set_pull_mode(MASTER_EN, GPIO_FLOATING);  
    
    gpio_reset_pin(DIR_MONITOR);
    gpio_set_direction(DIR_MONITOR, GPIO_MODE_INPUT);
    gpio_set_pull_mode(DIR_MONITOR, GPIO_FLOATING);  

    gpio_reset_pin(DIR_OVERRIDE);
    gpio_set_direction(DIR_OVERRIDE, GPIO_MODE_OUTPUT);
    gpio_set_pull_mode(DIR_OVERRIDE, GPIO_PULLDOWN_ONLY);  
  return;
}

//TTY config
void tty_init(){ 

  const uart_port_t uart_num = TTY_UART;
  uart_config_t uart_config = {
    .baud_rate = TTY_BAUD,
    .data_bits = UART_DATA_8_BITS,
    .parity = UART_PARITY_DISABLE,
    .stop_bits = UART_STOP_BITS_1,
    .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
    //.rx_flow_ctrl_thresh = 122, //not used with flowctrl_disable
    //.source_clk = UART_SCLK_DEFAULT,  
  };
    ESP_ERROR_CHECK(uart_driver_install(TTY_UART, TTY_TX_BUFF, TTY_RX_BUFF, 0, NULL, 0));
    ESP_ERROR_CHECK(uart_param_config(TTY_UART, &uart_config));
    ESP_ERROR_CHECK(uart_set_pin(TTY_UART, TTY_TXD_PIN, TTY_RXD_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));
return;
}

void i2c_init(){
  /*
  int i2c_slave_port = I2C_SLAVE_PORT;
  i2c_config_t conf_slave = {
    .sda_io_num = I2C_SDA_PIN,            // select SDA GPIO specific to your project
    .sda_pullup_en = GPIO_PULLUP_ENABLE,
    .scl_io_num = I2C_SCL_PIN,            // select SCL GPIO specific to your project
    .scl_pullup_en = GPIO_PULLUP_ENABLE,
    .mode = I2C_MODE_SLAVE,
    .slave.addr_10bit_en = 0,
    .slave.slave_addr = I2C_SLAVE_ADDR,        // slave address of your project
    .slave.maximum_speed = I2C_CLOCK, // expected maximum clock speed
    .clk_flags = 0,                            // optional; you can use I2C_SCLK_SRC_FLAG_* flags to choose I2C source clock here
};
*/
return;
}

TrackChannel::TrackChannel(gpio_num_t enable_out_pin, gpio_num_t reverse_pin, gpio_num_t brake_pin, gpio_num_t adc_pin, float adcscale, int tripMilliamps ){
//Configure the necessary GPIOs and ADC input
    gpio_reset_pin(enable_out_pin);
    gpio_set_direction(enable_out_pin, GPIO_MODE_OUTPUT);
    gpio_set_pull_mode(enable_out_pin, GPIO_PULLDOWN_ONLY);
    gpio_reset_pin(reverse_pin);
    gpio_set_direction(reverse_pin, GPIO_MODE_OUTPUT);
    gpio_set_pull_mode(reverse_pin, GPIO_PULLDOWN_ONLY);
    gpio_reset_pin(brake_pin);
    gpio_set_direction(brake_pin, GPIO_MODE_OUTPUT);
    gpio_set_pull_mode(brake_pin, GPIO_PULLDOWN_ONLY);
    gpio_reset_pin(adc_pin);
}

int mACheck() {
  //todo: read ADC
  return 0;
}
