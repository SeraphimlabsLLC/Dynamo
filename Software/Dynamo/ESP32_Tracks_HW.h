//Contains some code snippets from DCC-EX ESP32 branch
#pragma once
#define ESP32_TRACKS_HW_H

//#ifndef ESP32_RMTDCC_HW_H
//  #include "ESP32_rmtdcc.h"
//#endif

//Use config.h if present, otherwise defaults
#ifndef CONFIG_H
  #if __has_include ("config.h")
    #include "config.h"
  #endif
  #ifndef CONFIG_H
    #include "config.example.h"
  #endif
#endif  

#include "Arduino.h"
#include "driver/gpio.h"
#include "driver/i2c.h"
#include "driver/adc.h"

/*Track Configurations
 * Format: Enable Out pin, Enable In/DC mode pin, rev/sig pin, brake pin, adc pin, adc ticks per amp x1000, adc zero offset x 1000, adc trip ticks x1000
 * adc ticks per amp is calculated to match the hardware. 
 * adc trip ticks is calculated to where the hardware must shut off. Max is 4095 * 1000. 
 * 
 * Loconet hardware configuration
 * Format: uart#, mode, tx pin, rx pin, baud, txbuff, rxbuff, read size
 * 
 * RMT Hardware configuration
 * Format: 
 * On ESP32-S3, RMT channels 0-3 are TX only and 4-7 are RX only
 */
void Tracks_Init();
void Tracks_Loop();
bool MasterEnable();
 
#if BOARD_TYPE == DYNAMO
 // #pragma message "Building as Dynamo"
  #define MAX_TRACKS 4
  #define TRACK_1 DCCSigs[0].SetupHW(9, 0, 6, 13, 1, 81900, -60000, 3500000, 'C');
  #define TRACK_2 DCCSigs[1].SetupHW(10, 0, 7,  14, 2, 81900, -60000, 3500000, 'D');
  #define TRACK_3 DCCSigs[2].SetupHW(11, 0, 8, 48, 4, 81900, -60000, 3500000, 'E');
  #define TRACK_4 DCCSigs[3].SetupHW(12, 0, 9, 48, 5, 81900, -60000, 3500000, 'F');
  #define MASTER_EN 15 //GPIO15
  #define MASTER_EN_DEGLITCH 6 //uSec required between readings. Must have 2 of the same value to change state.

  #define ADC_MIN_OFFSET 60 //ADC is inaccurate at low values.
  void IRAM_ATTR Master_en_ISR(); //Pin change interrupt on MASTER_EN 
#endif

#if BOARD_TYPE == ARCH_BRIDGE
 // #pragma message "Building as Arch Bridge"
  #define MAX_TRACKS 2
  #define TRACK_1 DCCSigs[0].SetupHW(10, 13, 11, 12, 1, 16938409, -65000, 3650000, 'C');
  #define TRACK_2 DCCSigs[1].SetupHW(14, 48, 21,  47, 2, 1693480, -65000, 3650000, 'D');
  #define MASTER_EN 3 //Is an Output Enable instead of an input

  #define ADC_MIN_OFFSET 60 //ADC is inaccurate at low values.
  //Arch_Bridge boards do not generate a DIR_OVERRIDE, but the TX RMT can be attached to individual track REV pins.  
#endif

//ADC Settings:
#define OL_COOLDOWN 5000 //Time in mS that the power should be off when overload detected. 
#define ADC_DMAX 4095 //4095 in single shot, 8191 in continuous
#define ADC_VMAX 3.1 //Max readable voltage is actually 3.1v using mode ADC_ATTEN_DB_11  

class TrackChannel {
  //Very similar to DCC-EX class MotorDriver, but no dual signal support. 
  public:
    uint8_t index; //What track number is this? 
    char trackID; //Char to use as a handle for DCC-EX commmands
    int16_t cabaddr; //DCC addresss used for DC mode
    int8_t powerstate; //0 = off, 1 = on_forward, 2 = on_reverse - indicates overloaded 
    uint8_t powermode; //0 = none, 1 = EXT, 2 = DCC_Internal, 3 = DC, 4 = DCX. Future: Mode for generating PROG? 
    int32_t adc_base_ticks; //value read from ADC when output is off for calc reference.
    int32_t adc_previous_ticks; //value read on prior scan
    int32_t adc_current_ticks; //value read on most recent scan
    int32_t adc_smooth_ticks; // =(adc_smooth_ticks * 15 + adc_current_ticks) / 16
    int32_t adc_overload_trip; //Pre-calculate trip threshold in adc ticks
    int32_t adc_scale; //ADC ticks per amp * 1000. This can be higher than the adc max value if the hardware is <1A max. 
    int16_t adc_offset; //ADC offset in ticks * 1000. 

    gpio_num_t enable_out_pin;
    gpio_num_t enable_in_pin; //Enable_in on ArchBridge, DC mode select on Dynamo
    gpio_num_t reverse_pin;
    gpio_num_t brake_pin;
    adc1_channel_t adc_channel;

    void SetupHW(uint8_t en_out_pin, uint8_t en_in_pin, uint8_t rev_pin, uint8_t brk_pin, uint8_t adcpin, uint32_t adcscale, int32_t adcoffset, uint32_t adc_ol_trip, char track); 
    void ModeChange (int8_t newmode);
    void StateChange(int8_t newstate);
    uint8_t CheckEnable(); //Reads en_in, sets en_out the same, and returns on or off. 
    void adc_read();

    private: 
    uint64_t overload_cooldown; //time_us of overload last detected
};
