//Use config.h if present, otherwise defaults
#if __has_include ( "config.h")
  #include "config.h"
#else
  #include "config.example.h"
#endif
#include "Dynamo_HW.h"
void setup() {

}

void loop() {
//read ADCs, try to clear those that are overcurrent, and disable them if they don't. 


}
