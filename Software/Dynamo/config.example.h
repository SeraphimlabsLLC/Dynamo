//Config file for Arch Bridge or Dynamo boards using ESP32-S3 processors.
#define CONFIG_H

#define BOARD_ID "SeraphimLabs Dynamo v0.1 \n"
//Board Select. Define both, then pick which is active
#define DYNAMO 1
#define ARCH_BRIDGE 2
#define BOARD_TYPE DYNAMO
#define HEARTBEAT_S 60 //Seconds between console heartbeats

//System limits. DCC can address up to 16384 combined devices between turnouts, sensors, and signals. 
#define MAX_ACCESSORIES 16 

//Command bridging filters
#define RS_TO_DCCEX false //Only enable if DCC-EX is not the CS. 
#define RCN_TURNOUTS //DCC-EX turnouts use RCN

#define TIME_US micros() 
// Alternative: esp_timer_get_time();

/*TTY settings:
* uart_init(uint8_t uartnum, uint8_t txpin, uint8_t rxpin, uint32_t baudrate, uint16_t txbuff, uint16_t rxbuff);
* Parser selection. 0 = no parser, 1 = dccex, 2 = loconet
*/
#define TTY_CONFIG tty.uart_init(0, 43, 44, 115200, 4, 128); 
//#define DCCEX_UART dccex.dccex_port.uart_init(0, 43, 44, 115200, 4, 128); 

#define DCC_GENERATE false //Turn DCC test signal generator on or off
#warning Compiling Dynamo for ESP32-S3

//I2C settings: 
#define I2C_SDA_PIN 17 //GPI8 on Arch Bridge
#define I2C_SCL_PIN 18 //GPIO9 on Arch Bridge
#define I2C_MASTER false
#define I2C_SLAVE_ADDR 43
#define I2C_CLOCK 40000

#define I2C_SLAVE_PORT 0
#define I2C_TX_BUFF 256
#define I2C_RX_BUFF 4096

//DCC time Constants. Periods from NMRA S9.1 with some additional fudge factor
#if BOARD_TYPE == DYNAMO
  #define DIR_MONITOR 38 //GPIO38, Dir Monitor 
  #define DIR_OVERRIDE 2 //GPIO21, Dir Override. Used to inject a DCC waveform. 
#endif
#if BOARD_TYPE == ARCH_BRIDGE
  #define DIR_MONITOR 9 //GPIO9, railsync input. 
#endif
#define DCC_1_HALFPERIOD 58  //4640 // 1 / 80000000 * 4640 = 58us
#define DCC_1_MIN_HALFPERIOD 50 //NMRA S9.1 says 55uS Minimum half-1. 
#define DCC_1_MAX_HALFPERIOD 66 //NMRA S9.1 says 61uS Maximum half-1
#define DCC_0_HALFPERIOD 100 //8000
#define DCC_0_MIN_HALFPERIOD 90 //NMRA S9.1 says 95uS Minimum half-0
#define DCC_0_MAX_HALFPERIOD 12000 //NMRA S9.1 says 10000uS Maximum half-0, and 12000uS maximum full-0. 
#define DCC_RX_BUF 32; //Number of edge symbols possible
