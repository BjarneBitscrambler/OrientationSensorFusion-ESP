
/*! \file debug_print.c
    \brief defines function debug_print() that outputs to serial port
    Can disable these prints by compiling without defining ENABLE_DEBUG_LOG
*/

#include <HardwareSerial.h>
#include "debug_print.h"

void debug_log(const char* str) {
#if ENABLE_DEBUG_LOG == 1
   Serial.println(str);
#endif
}
