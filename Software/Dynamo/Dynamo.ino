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
/*#ifndef ESP32_RMTDCC_H
  #include "ESP32_rmtdcc.h"
#endif */
#ifndef DCC_READ_H
  #include "dcc_read.h"
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

volatile uint64_t edge_last;
volatile uint32_t edge_delta;
volatile uint8_t edge_stream; 
volatile uint8_t edge_count;
//volatile uint16_t edge_time[DCC_RX_BUF]; 
//volatile uint8_t edge_time_int = 0; 

void IRAM_ATTR dir_monitor_isr(){
  edge_delta = TIME_US - edge_last;
  edge_last = TIME_US;
  edge_count++;
  edge_stream = edge_stream << 1; //bit shift one left
  edge_stream = edge_stream + gpio_get_level(gpio_num_t(DIR_MONITOR));
  if ((edge_delta > DCC_1_MIN_HALFPERIOD) && (edge_delta < DCC_1_MAX_HALFPERIOD)) {
    edge_stream = edge_stream << 1; //bit shift one let
    edge_stream = edge_stream + 1; 
  }
  if ((edge_delta > DCC_0_MIN_HALFPERIOD) && (edge_delta < DCC_0_MAX_HALFPERIOD)) {
    edge_stream = edge_stream << 1; //bit shift one left
    edge_stream = edge_stream + 0; 
  } 
/*  // Debug output of edge timing
  edge_time[edge_time_int] = edge_delta; 
  edge_time_int++; 
  if (edge_time_int >= DCC_RX_BUF) {
    edge_time_int = 0; 
  }
*/
  return;
}

void setup() {
  ESP_uart_init(); //Initialize tty
  dccex_init(); //Initialize DCCEX parser
  dccrx_init(); //Initialize DCC signal audit
  Tracks_Init();  //Initialize GPIO
  attachInterrupt(DIR_MONITOR, dir_monitor_isr, CHANGE); 
}

void loop() {
dccex_loop(); //Process serial input for DCCEX commands
Tracks_Loop(); //Process and update tracks
dccrx_loop(); //Process and update DCC packets

Heartbeat(HEARTBEAT_S);
}
