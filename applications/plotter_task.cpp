#include "cmsis_os.h"
#include "io/plotter/plotter.hpp"
#include "tools/mahony/mahony.hpp"

extern sp::Mahony imu;
extern volatile float plot_motor_a_target_angle;
extern volatile float plot_motor_a_actual_angle;
extern volatile float plot_motor_b_target_angle;
extern volatile float plot_motor_b_actual_angle;

sp::Plotter plotter(&huart1, false);

extern "C" void plotter_task(void const * argument)
{
  (void)argument;

  while (true) {
    //plotter.plot(
    // imu.yaw, imu.pitch, imu.roll）
    plotter.plot(
      plot_motor_a_target_angle, plot_motor_a_actual_angle, plot_motor_b_target_angle,
      plot_motor_b_actual_angle);
    osDelay(10);
  }
}