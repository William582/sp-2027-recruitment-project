#include "cmsis_os.h"
#include "io/dbus/dbus.hpp"

sp::DBus remote(&huart3);

extern "C" void dt7_task(void const * argument)
{
  (void)argument;
  remote.request();

  while (true) {
    osDelay(10);
  }
}

extern "C" void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef * huart, uint16_t size)
{
  if (huart == &huart3) {
    remote.update(size, osKernelSysTick());
    remote.request();
  }
}

extern "C" void HAL_UART_ErrorCallback(UART_HandleTypeDef * huart)
{
  if (huart == &huart3) {
    remote.request();
  }
}