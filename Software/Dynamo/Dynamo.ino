//Use config.h if present, otherwise defaults
#if __has_include ( "config.h")
  #include "config.h"
#else
  #include "config.example.h"
#endif

#ifndef ESP32_UART_H
  #include "ESP32_uart.h"
#endif

#ifndef ESP32_TRACKS_HW_H
  #include "ESP32_Tracks_HW.h"
#endif
extern ESP_Uart tty; //normal serial port

extern TrackChannel DCCSigs[];
extern uint8_t max_tracks;
uint64_t time_us = 0; 
uint64_t last_time_us = 0;

void setup() {
  uint8_t master_en;
  delay(50); //Slight delay in startup for console init. 
  ESP_uart_init(); //Initialize tty
  ESP32_Tracks_Setup();  
  master_en = MasterEnable();
  Serial.printf("Master Enable presently %u \n");
  DCCSigs[0].ModeChange(1); //set to DCC EXT
  DCCSigs[0].StateChange(2); //set to ON FWD
  DCCSigs[1].ModeChange(1); //set to DCC EXT
  DCCSigs[1].StateChange(2); //set to ON FWD
  DCCSigs[2].ModeChange(1); //set to DCC EXT
  DCCSigs[2].StateChange(2); //set to ON FWD
  DCCSigs[3].ModeChange(1); //set to DCC EXT
  DCCSigs[3].StateChange(2); //set to ON FWD
}

void loop() {
    uint8_t master_en;
ESP32_Tracks_Loop(); //Process and update tracks


#define HEARTBEAT_US 10000000 //10 seconds
  time_us = esp_timer_get_time();
  if ((time_us - last_time_us) > HEARTBEAT_US) {
    master_en = MasterEnable();
    Serial.printf("Master Enable presently %u \n");
    Serial.printf("%Heartbeat scan Jitter: %u uS \n", (time_us - last_time_us - HEARTBEAT_US)); 
    last_time_us = time_us;
  }

}
