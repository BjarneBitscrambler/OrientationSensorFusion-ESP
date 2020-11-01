#include <Arduino.h>
#include <stdint.h>

#include "hal_issdk.h"


void ARM_systick_enable( void ) {
    //we don't need this function
    return;
}

void ARM_systick_start_ticks(int32_t *pstart) {
  //cast the unsigned value from micros() to signed, to avoid having to
  // change prototype in fusion files. Cast it back to unsigned when using.
  *pstart = (int32_t) micros();
}

int32_t ARM_systick_elapsed_ticks(int32_t start_ticks) {
    //cast start_ticks back to unsigned before using (which it was
    //when it was originally obtained from micros()). 
    //The return value is cast to signed int as that is what
    //the fusion routines expect.
    return (int32_t)((micros() - (uint32_t)start_ticks));
}
void ARM_systick_delay_ms(uint32_t iSystemCoreClock, uint32_t delay_ms) {
  delay(delay_ms);
}
