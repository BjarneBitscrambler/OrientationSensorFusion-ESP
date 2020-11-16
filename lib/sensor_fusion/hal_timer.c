#include <Arduino.h>
#include <stdint.h>

#include "hal_timer.h"

#define CORE_SYSTICK_HZ  1000000     //use the 1us resolution timer available on ESP processors
#define MICROSECS_IN_SEC 1000000     

void SystickStartCount(int32_t *pstart) {
  // cast the unsigned value from micros() to signed, to avoid having to
  // change prototype in fusion files. Cast it back to unsigned when using.
  *pstart = (int32_t)micros();
}  // end SystickStartCount()

int32_t SystickElapsedMicros(int32_t start_ticks) {
  // Cast start_ticks back to unsigned before using (which it was
  // when it was originally obtained from micros()).
  // Return value is cast to signed int as expected by fusion routines.
  // Note that if changing CORE_SYSTICK_HZ, it may be good to check if
  // the conversion to microsecs should be using floats as intermediates.
  return (int32_t)(((micros() - (uint32_t)start_ticks)) /
                   (CORE_SYSTICK_HZ / MICROSECS_IN_SEC));
}  // end SystickElapsedMicros()

void SystickDelayMillis(uint32_t delay_ms) {
  delay(delay_ms);
}  // end SystickDelayMillis()
