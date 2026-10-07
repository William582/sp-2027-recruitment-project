#include "cmsis_os.h"
#include "io/bmi088/bmi088.hpp"
#include "tools/mahony/mahony.hpp"
#include "tools/pid/pid.hpp"
#include "tim.h"

namespace
{
constexpr float IMU_TEMP_TARGET = 50.0f;
constexpr float IMU_TEMP_PID_DT = 1e-3f;
constexpr float IMU_TEMP_PID_KP = 400.0f;
constexpr float IMU_TEMP_PID_KI = 2.0f;
constexpr float IMU_TEMP_PID_KD = 0.0f;
constexpr float IMU_TEMP_PID_MAX_OUTPUT = 2000.0f;
constexpr float IMU_TEMP_PID_MAX_INTEGRAL = 200.0f;
constexpr float IMU_TEMP_PID_INTEGRAL_PAUSE = 1500.0f;

sp::PID imu_temp_pid(
  IMU_TEMP_PID_DT, IMU_TEMP_PID_KP, IMU_TEMP_PID_KI, IMU_TEMP_PID_KD,
  IMU_TEMP_PID_MAX_OUTPUT, IMU_TEMP_PID_MAX_INTEGRAL, 0.1f, false, true);

void control_imu_heater(float temperature)
{
  imu_temp_pid.calc(IMU_TEMP_TARGET, temperature, IMU_TEMP_PID_INTEGRAL_PAUSE);

  float pwm = imu_temp_pid.out;
  if (pwm < 0.0f) pwm = 0.0f;
  if (pwm > IMU_TEMP_PID_MAX_OUTPUT) pwm = IMU_TEMP_PID_MAX_OUTPUT;

  __HAL_TIM_SET_COMPARE(&htim10, TIM_CHANNEL_1, static_cast<uint32_t>(pwm));
}
}  // namespace

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
  if (HAL_TIM_PWM_Start(&htim10, TIM_CHANNEL_1) != HAL_OK) {
    Error_Handler();
  }

  while (true) {
    bmi088.update();
    control_imu_heater(bmi088.temp);
    imu.update(bmi088.acc, bmi088.gyro);
    imu_ready = true;
    osDelay(1);
  }
}