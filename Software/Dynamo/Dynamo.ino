//Use config.h if present, otherwise defaults
#if __has_include ( "config.h")
  #include "config.h"
#else
  #include "config.example.h"
#endif
#ifndef ESP32_TRACKS_HW_H
  #include "ESP32_Tracks_HW.h"
#endif
extern TrackChannel DCCSigs[];
extern int MAX_TRACK;

bool Master_Enable = false; //true to enable outputs
uint8_t adc_working = 0; //Which track is being read now



void setup() {
//Am I a joke to you? All the setup is getting done without this. 
}

void loop() {
uint8_t i = 0;
if (Master_Enable == true){  //OK to run. Enable tracks in suitable state. 
  while (i < 4){ //Fault 
    if (DCCSigs[i].powerstate >= 2){ //State is set to on forward or on reverse, ok to enable. 
      gpio_set_level(gpio_num_t(DCCSigs[i].enable_out_pin), 1); //Write 1 to enable out on each track
      DCCSigs[i].adc_previous_ticks = DCCSigs[i].adc_current_ticks; //cache previous adc reading
      DCCSigs[i].adc_current_ticks = analogRead(DCCSigs[i].adc_pin); //update adc reading
      if (DCCSigs[i].adc_current_ticks > DCCSigs[i].adc_overload_trip) {
        gpio_set_level(gpio_num_t(DCCSigs[i].enable_out_pin), 0); //overload tripped, track off.
        DCCSigs[i].powerstate = 1; //set track to overloaded so it stays off until further processing. 
      }
    }    
    i++; 
  }
} else { //Fault exists, disable all tracks. Leave track states unchanged for later re-enabling. 
  i = 0; //reset i for the next loop
  while (i < 4){
    //gpio_set_level(gpio_num_t(DCCSigs[i].enable_out_pin), 0); //Write 0 to enable out on each track
    //digitalWrite(DCCSigs[i].enable_out_pin, 0);
    i++;
  } 
}

}
