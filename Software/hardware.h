//ADC Settings:
#DEFINE ADC_DMAX 4095 //4095 in single shot, 8191 in continuous
#DEFINE ADC_VMAX 3.1 //Max readable voltage is actually 3.1v using mode ADC_ATTEN_DB_11  

//TTY settings:
/* Pin Map
TXD = GPIO43(U0TXD)
RXD = GPIO44(U0RXD)
*/
#DEFINE TTY_TX_BUFF = 4096
#DEFINE TTY_RX_BUFF = 4096

//I2C settings: 
/* Pin Map
SDA = GPIO17
SCL = GPIO18
*/
#DEFINE I2C_TX_BUFF = 4096
#DEFINE I2C_RX_BUFF = 4096

//Loconet settings:
/* Pin Map
LN_TX = GPIO17(U1RXD)
LN_RX = GPIO18 (U1RXD)
LN_COLL = GPIO0
*/
#DEFINE LN_TX_BUFF = 4096
#DEFINE LN_RX_BUFF = 4096

//Booster Control IO
/* Pin Map
DIR_MONITOR = GPIO38
DIR_OVERRIDE = GPIO21
MASTER_EN = GPIO15
*/

//Track 1 output
/*  Pin Map
ENA_1 = GPIO9
REV_1 = GPIO6
BRK_1 = GPIO13
A1 = GPIO1(ADC1_CH0)
*/
#DEFINE OUT1_MAX_A 4090 //ADC_DMAX / ADC_VMAX * SENSE_V

//Track 2 output
/* Pin Map
ENA_2 = GPIO10
REV_2 = GPIO7
BRK_2 = GPIO14
A2 = GPIO2(ADC1_CH1)
*/
#DEFINE OUT2_MAX_A 4090 //ADC_DMAX / ADC_VMAX * SENSE_V

//Track 3 output
/* Pin Map
ENA_3 = GPIO11
REV_3 = GPIO8
BRK_3 = GPIO47
A3 = GPIO2(ADC1_CH2)
*/
#DEFINE OUT3_MAX_A 4090 //ADC_DMAX / ADC_VMAX * SENSE_V

//Track 3 output
/* Pin Map
ENA_4 = GPIO12
REV_4 = GPIO9
BRK_4 = GPIO48
A4 = GPIO2(ADC1_CH3)
*/
#DEFINE OUT2_MAX_A 4090 //ADC_DMAX / ADC_VMAX * SENSE_V
