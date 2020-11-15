
/*! \file debug_print.h
    \brief defines function debug_print() that outputs to serial port
    Can disable these prints by compiling without defining ENABLE_DEBUG_LOG
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
