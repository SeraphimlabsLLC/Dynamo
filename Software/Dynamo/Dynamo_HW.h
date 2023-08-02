//Contains some code snippets from DCC-EX ESP32 branch
#pragma once
#include "driver/gpio.h"
#include "driver/uart.h"
#include "driver/i2c.h"
#include "driver/rmt.h"
#include "soc/rmt_reg.h"
#include "soc/rmt_struct.h"
//Use config.h if present, otherwise defaults
#if __has_include ( "config.h")
  #include "config.h"
#else
  #include "config.example.h"
#endif

// make calculations easy and set up for microseconds. Taken from DCC-EX DCCRMT.h
#define RMT_CLOCK_DIVIDER 80
#define DCC_1_HALFPERIOD 58  //4640 // 1 / 80000000 * 4640 = 58us
#define DCC_0_HALFPERIOD 100 //8000

class TrackChannel {
  //Very similar to DCC-EX class MotorDriver, but no dual signal support. 
  public:
    enum powerstate {off, overload, on_forward, on_reversed};
    enum powermode {none, DCC_external, DCC_override, DC};
    int base_milliamps; //value of adc when output is off
    int last_milliamps; //value of adc on previous read
    int current_milliamps; //value of adc on current read
    //todo: see if the RMT channel can be attached in here when required
    TrackChannel(gpio_num_t enable_out_pin, gpio_num_t reverse_pin, gpio_num_t brake_pin, gpio_num_t adc_pin, float adcscale, int tripMilliamps );
    int mA_Check();
};

void gpio_init();
void adc_init();

void tty_init();
void i2c_init();

void rmt_rx_init(); //Initialize RMT RX
void rmt_tx_init(); //Initialize RMT TX
void setDCCBit1(rmt_item32_t* item);
void setDCCBit0(rmt_item32_t* item);
void setEOT(rmt_item32_t* item);
