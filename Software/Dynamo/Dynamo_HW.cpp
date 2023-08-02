#include "Dynamo_HW.h"

//Architecture:
#define CORE_TYPE = ESP32_S3

//Use config.h if present, otherwise defaults
#if __has_include ( "config.h")
  #include "config.h"
#else
  #include "config.example.h"
#endif


//TTY settings:
#define TTY_BAUD 115200 //must match other endpoint
#define TTY_UART 0
#define TTY_TXD_PIN GPIO_NUM_43 //GPIO43(U0TXD)
#define TTY_RXD_PIN GPIO_NUM_44 //GPIO44(U0RXD?
#define TTY_TX_BUFF 256
#define TTY_RX_BUFF 4096

//I2C settings: 
#define I2C_SDA_PIN 17 //GPIO17
#define I2C_SCL_PIN 18 //GPIO18
#define I2C_MASTER false
#define I2C_SLAVE_ADDR 43
#define I2C_CLOCK 40000

#define I2C_SLAVE_PORT 0
#define I2C_TX_BUFF 256
#define I2C_RX_BUFF 4096

//Booster Control IO and RMT:
#define DIR_MONITOR 38 //GPIO38
#define DIR_OVERRIDE 21 //GPIO21
#define MASTER_EN 15 //GPIO15
//On ESP32-S3, RMT channels 0-3 are TX only and 4-7 are RX only
#define DIR_MONITOR_RMT 4
#define DIR_OVERRIDE_RMT 0

//Track Configurations
//TrackChannel(int enable_out_pin, int reverse_pin, int brake_pin, int adc_pin, float adcscale, int tripMilliamps )
//TrackChannel(9, 6, 13, 1, 0, 4090); //Track 1
//TrackChannel(10, 7, 14, 2, 0, 4090); //Track 2
//TrackChannel(11, 8, 47, 3, 0, 4090); //Track 3
//TrackChannel(12, 9, 48, 4, 0, 4090); //Track 4

//ADC Settings:
#define ADC_DMAX 4095 //4095 in single shot, 8191 in continuous
#define ADC_VMAX 3.1 //Max readable voltage is actually 3.1v using mode ADC_ATTEN_DB_11  

//Control GPIO and RMT initialization
void gpio_init(){
    gpio_reset_pin(gpio_num_t(MASTER_EN));
    gpio_set_direction(gpio_num_t(MASTER_EN), GPIO_MODE_INPUT);
    gpio_set_pull_mode(gpio_num_t(MASTER_EN), GPIO_FLOATING);  
    
    gpio_reset_pin(gpio_num_t(DIR_MONITOR));
    gpio_set_direction(gpio_num_t(DIR_MONITOR), GPIO_MODE_INPUT);
    gpio_set_pull_mode(gpio_num_t(DIR_MONITOR), GPIO_FLOATING);  
    rmt_rx_init(); //Initialize DIR_MONITOR for RMT monitoring

    gpio_reset_pin(gpio_num_t(DIR_OVERRIDE));
    gpio_set_direction(gpio_num_t(DIR_OVERRIDE), GPIO_MODE_OUTPUT);
    gpio_set_pull_mode(gpio_num_t(DIR_OVERRIDE), GPIO_PULLDOWN_ONLY);  


  return;
}

void adc_init(){

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
  i2c_config_t i2c_conf_slave;
    i2c_conf_slave.sda_io_num = gpio_num_t(I2C_SDA_PIN);
    i2c_conf_slave.sda_pullup_en = GPIO_PULLUP_ENABLE;
    i2c_conf_slave.scl_io_num = gpio_num_t(I2C_SCL_PIN);
    i2c_conf_slave.scl_pullup_en = GPIO_PULLUP_ENABLE;
    i2c_conf_slave.mode = I2C_MODE_SLAVE;
    i2c_conf_slave.slave.addr_10bit_en = 0;
    i2c_conf_slave.slave.slave_addr = I2C_SLAVE_ADDR;  // slave address of your project
    i2c_conf_slave.slave.maximum_speed = I2C_CLOCK; // expected maximum clock speed
    i2c_conf_slave.clk_flags = 0;   // optional; you can use I2C_SCLK_SRC_FLAG_* flags to choose I2C source clock here
  /*esp_err_t err = i2c_param_config(i2c_slave_number(I2C_SLAVE_PORT), &i2c_conf_slave);
  if (err != ESP_OK) {
    return err;
  }*/
return;
}

void rmt_rx_init(){
  //Initialize RMT for DCC auditing
  rmt_config_t rmt_rx_config;
  // Configure the RMT channel for RX
//  bzero(&config, sizeof(rmt_config_t));
  rmt_rx_config.rmt_mode = RMT_MODE_RX;
  rmt_rx_config.channel = rmt_channel_t(DIR_MONITOR_RMT);
  rmt_rx_config.clk_div = RMT_CLOCK_DIVIDER;
  rmt_rx_config.gpio_num = gpio_num_t(DIR_MONITOR);
  rmt_rx_config.mem_block_num = 2; // With longest DCC packet 11 inc checksum (future expansion)
                            // number of bits needed is 22preamble + start +
                            // 11*9 + extrazero + EOT = 124
                            // 2 mem block of 64 RMT items should be enough
  ESP_ERROR_CHECK(rmt_config(&rmt_rx_config));
  // NOTE: ESP_INTR_FLAG_IRAM is *NOT* included in this bitmask
  ESP_ERROR_CHECK(rmt_driver_install(rmt_rx_config.channel, 0, ESP_INTR_FLAG_LOWMED|ESP_INTR_FLAG_SHARED));   
}

void rmt_tx_init(){
  //Initialize RMT for DCC TX 
  rmt_config_t rmt_tx_config;
  // Configure the RMT channel for TX
  rmt_tx_config.rmt_mode = RMT_MODE_TX;
  rmt_tx_config.channel = rmt_channel_t(DIR_OVERRIDE_RMT);
  rmt_tx_config.clk_div = RMT_CLOCK_DIVIDER;
  rmt_tx_config.gpio_num = gpio_num_t(DIR_OVERRIDE);
  rmt_tx_config.mem_block_num = 2; // With longest DCC packet 11 inc checksum (future expansion)
                            // number of bits needed is 22preamble + start +
                            // 11*9 + extrazero + EOT = 124
                            // 2 mem block of 64 RMT items should be enough
  ESP_ERROR_CHECK(rmt_config(&rmt_tx_config));
  // NOTE: ESP_INTR_FLAG_IRAM is *NOT* included in this bitmask
  ESP_ERROR_CHECK(rmt_driver_install(rmt_tx_config.channel, 0, ESP_INTR_FLAG_LOWMED|ESP_INTR_FLAG_SHARED));    
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

//From DCC-EX ESP32 branch DCCRMT.cpp:

void setDCCBit1(rmt_item32_t* item) {
  item->level0    = 1;
  item->duration0 = DCC_1_HALFPERIOD;
  item->level1    = 0;
  item->duration1 = DCC_1_HALFPERIOD;
}

void setDCCBit0(rmt_item32_t* item) {
  item->level0    = 1;
  item->duration0 = DCC_0_HALFPERIOD;
  item->level1    = 0;
  item->duration1 = DCC_0_HALFPERIOD;
}

void setEOT(rmt_item32_t* item) {
  item->val = 0;
}
