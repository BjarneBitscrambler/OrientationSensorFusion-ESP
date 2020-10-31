
/*! \file debug_print.h
    \brief defines function debug_print() that outputs to serial port
    Can disable these prints by compiling without defining ENABLE_DEBUG_LOG
*/

#ifndef DEBUG_PRINT_H
#define DEBUG_PRINT_H

#ifdef __cplusplus
extern "C" {
#endif

#define ENABLE_DEBUG_LOG 1

void debug_log(const char* str);

#ifdef __cplusplus
}
#endif

#endif // #ifndef DEBUG_PRINT_H
