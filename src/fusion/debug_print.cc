
/** @file debug_print.cc
    @brief defines function debug_log() that outputs using ESP_LOGx()
    Usually this outputs to serial port - see system file esp_log.h
    Can change the definition here to send the messages via another method,
    e.g. if porting to a different platform.
    Can disable these prints by compiling without defining ENABLE_DEBUG_LOG
*/

#include <esp_log.h>
#include "build.h"
#include "debug_print.h"

#if (ENABLE_DEBUG_LOG == 1)
void debug_log(const char* str) 
{   ESP_LOGD("debug_print",str);
}
#endif
