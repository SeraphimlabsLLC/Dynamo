//Contains some code snippets from DCC-EX ESP32 branch
#pragma once
//Use config.h if present, otherwise defaults
#if __has_include ( "config.h")
  #include "config.h"
#else
  #include "config.example.h"
#endif
#include "driver/gpio.h"
#include "driver/uart.h"
#include "driver/i2c.h"
#include "driver/rmt.h"
#include "soc/rmt_reg.h"
#include "soc/rmt_struct.h"

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
#define DIR_MONITOR_RMT 4 //On ESP32-S3, RMT channels 0-3 are TX only and 4-7 are RX only
#define DIR_OVERRIDE_RMT 0  
#define RMT_CLOCK_DIVIDER 80  // make calculations easy and set up for microseconds. Taken from DCC-EX DCCRMT.h
#define DCC_1_HALFPERIOD 58  //4640 // 1 / 80000000 * 4640 = 58us
#define DCC_0_HALFPERIOD 100 //8000

//Track Configurations
#define ENAO_1 9
#define REV_1 6
#define BRK_1 13
#define AIN_1 1
#define ADC_TICKS_PER_AMP_1 819 //Ticks per Amp. On outputs capable of <1A, this will be larger than 4095.
#define ADC_TRIP_TICKS_1 4090 //ADC ticks at which reverse/cutofff is necessary.

#define ENAO_2 10
#define REV_2 7
#define BRK_2 14
#define AIN_2 2
#define ADC_TICKS_PER_AMP_2 819
#define ADC_TRIP_TICKS_2 4090

#define ENAO_3 11
#define REV_3 8
#define BRK_3 47
#define AIN_3 3
#define ADC_TICKS_PER_AMP_3 819
#define ADC_TRIP_TICKS_3 4090

#define ENAO_4 12
#define REV_4 9
#define BRK_4 48
#define AIN_4 4
#define ADC_TICKS_PER_AMP_4 819
#define ADC_TRIP_TICKS_4 4090

class TrackChannel {
  //Very similar to DCC-EX class MotorDriver, but no dual signal support. 
  public:
    uint8_t powerstate; //0 = off, 1 = overload, 2 = on_forward, 3 =on_reversed. 
    uint8_t powermode; //0 = none, 1 = DCC_external, 2 = DCC_override, 3 = DC.
    uint16_t adcscale; //ADC ticks per amp. This can be higher than the adc max value if the hardware is <1A max. 
    uint16_t adc_overload_trip; //Pre-calculate trip threshold in adc ticks
    uint16_t adc_base_ticks; //value read from ADC when output is off for calc reference.
    uint16_t adc_previous_ticks; //value read on prior scan
    uint16_t adc_current_ticks; //value read on most recent scan
    int adc_read();
    void EnableOut(bool pinstate); //Write to enable
    void ReverseOut(bool pinstate); //Write to reverse
    void BrakeOut(bool pinstate); //Write to brake
    uint8_t enable_out_pin;
    uint8_t enable_in_pin; //Not used in Dynamo, will be used in ArchBridge. 
    uint8_t reverse_pin;
    uint8_t brake_pin;
    uint8_t adc_pin;
    TrackChannel(uint8_t enable_out_pin, uint8_t enable_in_pin, uint8_t reverse_pin, uint8_t brake_pin, uint8_t adc_pin, uint16_t adcscale, uint16_t adc_overload_trip);
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
