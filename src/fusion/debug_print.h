
/** @file debug_print.h
    @brief Declares function debug_log() that outputs using ESP_LOGx()
    Usually this outputs to serial port - see system file esp_log.h
    Can change the definition in debug_print.cc to send messages via
    another method, e.g. if porting to a different platform.
*/

#ifndef DEBUG_PRINT_H
#define DEBUG_PRINT_H

#ifdef __cplusplus
extern "C" {
#endif

#if (ENABLE_DEBUG_LOG == 1)
void debug_log(const char* str);
#else
    #define debug_log(x) 
#endif

#ifdef __cplusplus
}
#endif

#endif // #ifndef DEBUG_PRINT_H
