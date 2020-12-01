
/*! \file debug_print.c
    \brief defines function debug_print() that outputs to serial port
    Can disable these prints by compiling without defining ENABLE_DEBUG_LOG
*/

#include <HardwareSerial.h>
#include "build.h"
#include "debug_print.h"

#if (ENABLE_DEBUG_LOG == 1)
void debug_log(const char* str) {
   Serial.println(str);
}
#endif
