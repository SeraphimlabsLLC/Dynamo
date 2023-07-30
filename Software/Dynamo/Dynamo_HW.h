#include "driver/uart.h"
#include "driver/gpio.h"
//Use config.h if present, otherwise defaults
#if __has_include ( "config.h")
  #include "config.h"
#else
  #include "config.example.h"
#endif
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

void tty_init();

void i2c_init();

void rmt_init();
