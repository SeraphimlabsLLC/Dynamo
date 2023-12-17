//Use config.h if present, otherwise defaults
#ifndef CONFIG_H
  #if __has_include ("config.h")
    #include "config.h"
  #endif
  #ifndef CONFIG_H
    #include "config.example.h"
  #endif
#endif  
#ifndef ESP32_UART_H
  #include "ESP32_uart.h"
#endif
#ifndef ESP32_RMTDCC_H
  #include "ESP32_rmtdcc.h"
#endif
#ifndef ESP32_TRACKS_HW_H
  #include "ESP32_Tracks_HW.h"
#endif
#ifndef ESP32_DCCEX_H
  #include "ESP32_dccex.h"
#endif  

#ifndef ESP32_TIMER_H
  #include "ESP32_timer.h"
#endif


void setup() {
  ESP_uart_init(); //Initialize tty
  dccex_init(); //Initialize DCCEX parser
  Tracks_Init();  //Initialize GPIO and RMT hardware, calls the relevant rmt inits
}

void loop() {
dccex_loop(); //Process serial input for DCCEX commands
Tracks_Loop(); //Process and update tracks
rmt_loop(); //Process and update DCC packets

Heartbeat(HEARTBEAT_S);
}
