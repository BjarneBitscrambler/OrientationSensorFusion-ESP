#include "replacement_functions.h"


void ARM_systick_enable( void ) {
    //we don't need this function
    return;
}

void ARM_systick_start_ticks(int32_t *pstart) {
  *pstart = (int32_t) micros();
}

int32_t ARM_systick_elapsed_ticks(int32_t start_ticks) {
    return (int32_t)((micros() - (uint32_t)start_ticks));
}
void ARM_systick_delay_ms(uint32_t iSystemCoreClock, uint32_t delay_ms) {
  delay(delay_ms);
}
