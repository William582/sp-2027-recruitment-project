#include "cmsis_os.h"
#include "io/led/led.hpp"

sp::LED led(&htim5);

extern "C" void led_task(void const * argument)
{
  (void)argument;
  led.start();

  while (true) {
    for (uint8_t g = 0; g < 10; g++) {
      led.set(0, g * 0.01f, 0);
      osDelay(100);
    }

    for (uint8_t g = 10; g > 0; g--) {
      led.set(0, g * 0.01f, 0);
      osDelay(100);
    }
  }
}