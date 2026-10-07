#include "cmsis_os.h"
#include "io/plotter/plotter.hpp"
#include "tools/mahony/mahony.hpp"

extern sp::Mahony imu;

sp::Plotter plotter(&huart1, false);

extern "C" void plotter_task(void const * argument)
{
  (void)argument;

  while (true) {
    plotter.plot(imu.yaw, imu.pitch, imu.roll);
    osDelay(10);
  }
}