#include "cmsis_os.h"
#include "io/bmi088/bmi088.hpp"
#include "tools/mahony/mahony.hpp"

const float r_ab[3][3] = {
  {0.0f, -1.0f, 0.0f},
  {1.0f, 0.0f, 0.0f},
  {0.0f, 0.0f, 1.0f},
};

sp::BMI088 bmi088(&hspi1, GPIOA, GPIO_PIN_4, GPIOB, GPIO_PIN_0, r_ab);
sp::Mahony imu(1e-3f);
volatile bool imu_ready = false;

extern "C" void imu_task(void const * argument)
{
  (void)argument;
  bmi088.init();

  while (true) {
    bmi088.update();
    imu.update(bmi088.acc, bmi088.gyro);
    imu_ready = true;
    osDelay(1);
  }
}