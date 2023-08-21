//Use config.h if present, otherwise defaults
#if __has_include ( "config.h")
  #include "config.h"
#else
  #include "config.example.h"
#endif
#include "Dynamo_HW.h"

bool Master_Enable = false; //true to enable outputs
uint8_t next_adc_read = 0; //Keep track of which track channel is next to have its adc read

//Track struct setup. If more than 4 are possible, add more definitions here. 
#define MAX_TRACK 3 //Track index counts from 0.
TrackChannel ChannelList[]{ //Define track channel objects
  //TrackChannel(enable_out_pin, enable_in_pin, uint8_t reverse_pin, brake_pin, adc_pin, adcscale, adc_overload_trip)
  TrackChannel(ENAO_1, 0, REV_1, BRK_1, AIN_1, ADC_TICKS_PER_AMP_1, ADC_TRIP_TICKS_1),
  TrackChannel(ENAO_2, 0, REV_2, BRK_2, AIN_2, ADC_TICKS_PER_AMP_2, ADC_TRIP_TICKS_2),
  TrackChannel(ENAO_3, 0, REV_3, BRK_3, AIN_3, ADC_TICKS_PER_AMP_3, ADC_TRIP_TICKS_3),
  TrackChannel(ENAO_4, 0, REV_4, BRK_4, AIN_4, ADC_TICKS_PER_AMP_4, ADC_TRIP_TICKS_4), 
 
};

void setup() {
//Am I a joke to you? All the setup is getting done without this. 
}

void loop() {

//todo: check dcc signal and master enable gpio, and update Master_Enable

uint8_t i = 0;
if (Master_Enable == true){  //OK to run. Enable tracks in suitable state. 
  while (i < MAX_TRACK){ //Fault 
    if (ChannelList[i].powerstate >= 2){ //State is set to on forward or on reverse, ok to enable. 
      gpio_set_level(gpio_num_t(ChannelList[i].enable_out_pin), 1); //Write 1 to enable out on each track
    }    
    i++; 
  }
} else { //Fault exists, disable all tracks. Leave track states unchanged for later re-enabling. 
  while (i < MAX_TRACK){
    gpio_set_level(gpio_num_t(ChannelList[i].enable_out_pin), 0); //Write 0 to enable out on each track
    i++;
  } 
}


  //read ADCs, try to clear those that are overcurrent, and disable them if they don't. 
  if ChannelList[next_adc_read].powerstate >= 2 { //if the track should be on, read its value.
  //read ADC
  }
  
  next_adc_read++;
  if (next_adc_read > MAX_TRACK){
    next_adc_read = 0;
  }

}
