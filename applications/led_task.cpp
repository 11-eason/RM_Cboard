#include "cmsis_os.h"
#include "io/led/led.hpp"

sp::LED led(&htim5);

extern "C" void led_task()
{
  led.start();

  while (true) {
    for (;;) {
      led.set(1.0f, 0, 0);  // ºì
      osDelay(200);
      led.set(0, 1.0f, 0);  // ÂÌ
      osDelay(200);
      led.set(0, 0, 1.0f);  // À¶
      osDelay(200);
      led.set(0, 0, 0);  // Ãð
      osDelay(200);
    }
  }
}