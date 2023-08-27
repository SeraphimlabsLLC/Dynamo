#ifndef ESP32_TRACKS_HW_H
  #include "ESP32_Tracks_HW.h"
#endif
//Track struct setup. If more than 4 are possible, add more definitions here. 
const int MAX_TRACK = 3; //Track index counts from 0.
//TrackChannel(enable_out_pin, enable_in_pin, uint8_t reverse_pin, brake_pin, adc_pin, adcscale, adc_overload_trip)
TrackChannel DCCSigs[]{ //Define track channel objects
  TrackChannel(ENAO_1, 0, REV_1, BRK_1, AIN_1, ADC_TICKS_PER_AMP_1, ADC_TRIP_TICKS_1),
  TrackChannel(ENAO_2, 0, REV_2, BRK_2, AIN_2, ADC_TICKS_PER_AMP_2, ADC_TRIP_TICKS_2),
  TrackChannel(ENAO_3, 0, REV_3, BRK_3, AIN_3, ADC_TICKS_PER_AMP_3, ADC_TRIP_TICKS_3),
  TrackChannel(ENAO_4, 0, REV_4, BRK_4, AIN_4, ADC_TICKS_PER_AMP_4, ADC_TRIP_TICKS_4),
};

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
    //common settings:
    //adc1_config_width(ADC_WIDTH_BIT_12);
    //adc1_config_channel_atten(pinToADC1Channel(currentPin),ADC_ATTEN_DB_11); //ADC range 0-3.1v

    //per track settings:
    int i = 0;
    while (i < 3){ 
      //So apparently the Arduino IDE allows ESP reads using just analogRead(gpio)
//      int zeroval = analogRead(ChannelList[i].adc_pin);
      //ChannelList[i].adc_base_ticks = zeroval;
    }
/*  
    pinMode(AIN_1, ANALOG);
    senseOffset = adc1_get_raw(pinToADC1Channel(currentPin));
    */
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
/* NMRA allows up to 32 bytes per packet, the max length would be 301 bits transmitted and need 38 bytes (3 bits extra). 
 * DCC_EX is limited to only 11 bytes max. Much easier to account for and uses a smaller buffer. 
  */
  rmt_rx_config.mem_block_num = 3; 
  ESP_ERROR_CHECK(rmt_config(&rmt_rx_config));
  // NOTE: ESP_INTR_FLAG_IRAM is *NOT* included in this bitmask
  ESP_ERROR_CHECK(rmt_driver_install(rmt_rx_config.channel, 0, ESP_INTR_FLAG_LOWMED|ESP_INTR_FLAG_SHARED));   
  return;
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
  return;
}

TrackChannel::TrackChannel(uint8_t enable_out_pin, uint8_t enable_in_pin, uint8_t reverse_pin, uint8_t brake_pin, uint8_t adc_pin, uint16_t adcscale, uint16_t adc_overload_trip){
    TrackChannel::powerstate = 0; //Power is off by default
    TrackChannel::powermode = 0; //Mode is not configured by default
    //Copy config values to class values
    TrackChannel::enable_out_pin = enable_out_pin;
    TrackChannel::enable_in_pin = enable_in_pin;
    TrackChannel::reverse_pin = reverse_pin;
    TrackChannel::brake_pin = brake_pin;
    TrackChannel::adc_pin = adc_pin;
    TrackChannel::adcscale = adcscale;
    TrackChannel::adc_overload_trip = adc_overload_trip;
    //Configure GPIOs
    gpio_reset_pin(gpio_num_t(adc_pin));
    gpio_reset_pin(gpio_num_t(enable_out_pin));
    gpio_set_direction(gpio_num_t(enable_out_pin), GPIO_MODE_OUTPUT);
    gpio_set_pull_mode(gpio_num_t(enable_out_pin), GPIO_PULLDOWN_ONLY);
    gpio_reset_pin(gpio_num_t(reverse_pin));
    gpio_set_direction(gpio_num_t(reverse_pin), GPIO_MODE_OUTPUT);
    gpio_set_pull_mode(gpio_num_t(reverse_pin), GPIO_PULLDOWN_ONLY);
    gpio_reset_pin(gpio_num_t(brake_pin));
    gpio_set_direction(gpio_num_t(brake_pin), GPIO_MODE_OUTPUT);
    gpio_set_pull_mode(gpio_num_t(brake_pin), GPIO_PULLDOWN_ONLY);
}
/*
void TrackChannel::ConfigureTrack(uint8_t track){  //Updates GPIO modes when changing powermode
//powermode 0 = none, 1 = DCC_external, 2 = DCC_override, 3 = DC.  
  if (ChannelList[track].powermode == 1) { //DCC external. Configure enable_in if used and rev/brake
    gpio_reset_pin(gpio_num_t(reverse_pin));
    gpio_set_direction(gpio_num_t(reverse_pin), GPIO_MODE_OUTPUT);
    gpio_set_pull_mode(gpio_num_t(reverse_pin), GPIO_PULLDOWN_ONLY);
    gpio_reset_pin(gpio_num_t(brake_pin));
    gpio_set_direction(gpio_num_t(brake_pin), GPIO_MODE_OUTPUT);
    gpio_set_pull_mode(gpio_num_t(brake_pin), GPIO_PULLDOWN_ONLY);      
  }
}
*/

int adc_read() {
  //todo: read ADC
  return 0;
}

void EnableOut(bool pinstate){ //Write to enable
  //gpio_set_level(gpio_num_t(ChannelList[track].enable_out_pin), pinstate);
}
void ReverseOut(bool pinstate){ //Write to reverse
  //gpio_set_level(gpio_num_t(ChannelList[track].reverse_pin), pinstate);
}
void BrakeOut(bool pinstate){ //Write to brake
  //gpio_set_level(gpio_num_t(ChannelList[track].brake_pin), pinstate);
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
