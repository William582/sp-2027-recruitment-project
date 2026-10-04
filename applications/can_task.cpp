#include <cmath>

#include "cmsis_os.h"
#include "io/can/can.hpp"
#include "io/dbus/dbus.hpp"
#include "motor/rm_motor/rm_motor.hpp"
#include "tools/mahony/mahony.hpp"

extern sp::DBus remote;
extern sp::Mahony imu;
extern volatile bool imu_ready;

sp::CAN can1(&hcan1);
sp::RM_Motor motor_a(1, sp::RM_Motors::GM6020);
sp::RM_Motor motor_b(2, sp::RM_Motors::GM6020);

namespace
{
constexpr float PI = 3.14159265358979323846f;
constexpr float MOTOR_A_ENCODER_SIGN = 1.0f;
constexpr float MOTOR_B_ENCODER_SIGN = 1.0f;
constexpr bool MOTOR_DIRECTIONS_VERIFIED = false;

constexpr float MAX_OUTPUT_SPEED = 10.0f;
constexpr float MAX_TORQUE = 0.5f;
constexpr float POSITION_TO_SPEED_GAIN = 2.0f;
constexpr float SPEED_TO_TORQUE_GAIN = 0.05f;

constexpr uint32_t MANUAL_DETECT_WINDOW_MS = 50;
constexpr uint32_t FOLLOW_SETTLE_TIME_MS = 100;
constexpr float MANUAL_YAW_STABLE_DELTA = 0.01f;
constexpr float MANUAL_MOTOR_DELTA = 0.05f;
constexpr float OTHER_MOTOR_STABLE_DELTA = 0.015f;
constexpr float FOLLOW_POSITION_TOLERANCE = 0.03f;
constexpr float FOLLOW_SPEED_TOLERANCE = 0.1f;

float clamp(float value, float limit)
{
  if (value > limit) return limit;
  if (value < -limit) return -limit;
  return value;
}

float ratio_from_switch(sp::DBusSwitchMode mode)
{
  switch (mode) {
    case sp::DBusSwitchMode::DOWN:
      return 0.5f;
    case sp::DBusSwitchMode::MID:
      return -1.0f;
    case sp::DBusSwitchMode::UP:
      return 3.0f;
  }
  return 0.5f;
}

float position_control(float target, float position, float speed)
{
  const float desired_speed = clamp((target - position) * POSITION_TO_SPEED_GAIN, MAX_OUTPUT_SPEED);
  return clamp((desired_speed - speed) * SPEED_TO_TORQUE_GAIN, MAX_TORQUE);
}

void send_motor_commands(float torque_a, float torque_b)
{
  for (uint8_t i = 0; i < sp::CAN_DATA_LEN; i++) can1.tx_data[i] = 0;

  motor_a.cmd(MOTOR_A_ENCODER_SIGN * torque_a);
  motor_b.cmd(MOTOR_B_ENCODER_SIGN * torque_b);
  motor_a.write(can1.tx_data);
  motor_b.write(can1.tx_data);
  can1.send(motor_a.tx_id);
}

void read_motor_feedback(uint32_t stamp_ms)
{
  while (HAL_CAN_GetRxFifoFillLevel(&hcan1, CAN_RX_FIFO0) > 0) {
    can1.recv(CAN_RX_FIFO0);
    if (can1.frame_type) continue;

    if (can1.rx_id == motor_a.rx_id) {
      motor_a.read(can1.rx_data, stamp_ms);
    }
    else if (can1.rx_id == motor_b.rx_id) {
      motor_b.read(can1.rx_data, stamp_ms);
    }
  }
}

float unwrap_delta(float delta)
{
  if (delta > PI) return delta - 2.0f * PI;
  if (delta < -PI) return delta + 2.0f * PI;
  return delta;
}
}  // namespace

extern "C" void can_task(void const * argument)
{
  (void)argument;
  can1.config();
  can1.start();

  bool have_yaw = false;
  bool baseline_ready = false;
  bool have_window = false;
  bool follow_pending = false;
  bool have_right_switch = false;
  float last_yaw_raw = 0.0f;
  float yaw_unwrapped = 0.0f;
  float startup_yaw = 0.0f;
  float startup_a = 0.0f;
  float startup_b = 0.0f;
  float yaw_bias = 0.0f;
  float a_link_offset = 0.0f;
  float b_link_offset = 0.0f;
  float link_ratio = 0.5f;
  float window_yaw = 0.0f;
  float window_a = 0.0f;
  float window_b = 0.0f;
  uint32_t window_start_ms = 0;
  uint32_t settled_ms = 0;
  sp::DBusSwitchMode last_right_switch = sp::DBusSwitchMode::DOWN;

  while (true) {
    const uint32_t now_ms = HAL_GetTick();
    read_motor_feedback(now_ms);

    const bool remote_alive = remote.is_alive(now_ms);
    const bool motors_alive = motor_a.is_alive(now_ms) && motor_b.is_alive(now_ms);

    if (imu_ready) {
      const float yaw_raw = imu.yaw;
      if (!have_yaw) {
        last_yaw_raw = yaw_raw;
        yaw_unwrapped = yaw_raw;
        have_yaw = true;
      }
      else {
        yaw_unwrapped += unwrap_delta(yaw_raw - last_yaw_raw);
        last_yaw_raw = yaw_raw;
      }
    }

    const float angle_a = MOTOR_A_ENCODER_SIGN * motor_a.angle;
    const float angle_b = MOTOR_B_ENCODER_SIGN * motor_b.angle;
    const float speed_a = MOTOR_A_ENCODER_SIGN * motor_a.speed;
    const float speed_b = MOTOR_B_ENCODER_SIGN * motor_b.speed;

    if (!baseline_ready && remote_alive && motors_alive && have_yaw) {
      const float initial_ratio = ratio_from_switch(remote.sw_l);
      startup_yaw = yaw_unwrapped;
      startup_a = angle_a;
      startup_b = angle_b;
      link_ratio = initial_ratio;
      a_link_offset = angle_a - yaw_unwrapped;
      b_link_offset = angle_b - initial_ratio * yaw_unwrapped;
      baseline_ready = true;
    }

    float torque_a = 0.0f;
    float torque_b = 0.0f;
    const bool control_ready = baseline_ready && remote_alive && motors_alive && have_yaw &&
                               MOTOR_DIRECTIONS_VERIFIED;

    if (control_ready) {
      const sp::DBusSwitchMode right_switch = remote.sw_r;
      const float ratio = ratio_from_switch(remote.sw_l);

      if (!have_right_switch || right_switch != last_right_switch) {
        if (right_switch == sp::DBusSwitchMode::MID) {
          yaw_bias = 0.0f;
          a_link_offset = angle_a - yaw_unwrapped;
          b_link_offset = angle_b - ratio * yaw_unwrapped;
          follow_pending = false;
          settled_ms = 0;
          link_ratio = ratio;
        }
        else if (right_switch == sp::DBusSwitchMode::UP) {
          follow_pending = false;
          settled_ms = 0;
        }
        else {
          follow_pending = false;
          settled_ms = 0;
        }

        last_right_switch = right_switch;
        have_right_switch = true;
        have_window = false;
      }

      if (right_switch == sp::DBusSwitchMode::MID) {
        if (ratio != link_ratio) {
          b_link_offset = angle_b - ratio * (yaw_unwrapped + yaw_bias);
          link_ratio = ratio;
        }

        if (!have_window) {
          window_yaw = yaw_unwrapped;
          window_a = angle_a;
          window_b = angle_b;
          window_start_ms = now_ms;
          have_window = true;
        }
        else if (now_ms - window_start_ms >= MANUAL_DETECT_WINDOW_MS) {
          const float yaw_delta = yaw_unwrapped - window_yaw;
          const float delta_a = angle_a - window_a;
          const float delta_b = angle_b - window_b;

          if (fabsf(yaw_delta) >= MANUAL_YAW_STABLE_DELTA) {
            follow_pending = true;
            settled_ms = 0;
          }
          else if (!follow_pending) {
            if (fabsf(delta_a) >= MANUAL_MOTOR_DELTA &&
                fabsf(delta_b) <= OTHER_MOTOR_STABLE_DELTA) {
              yaw_bias += delta_a;
              follow_pending = true;
              settled_ms = 0;
            }
            else if (fabsf(delta_b) >= MANUAL_MOTOR_DELTA &&
                     fabsf(delta_a) <= OTHER_MOTOR_STABLE_DELTA) {
              yaw_bias += delta_b / ratio;
              follow_pending = true;
              settled_ms = 0;
            }
          }

          window_yaw = yaw_unwrapped;
          window_a = angle_a;
          window_b = angle_b;
          window_start_ms = now_ms;
        }

        const float target_a = a_link_offset + yaw_unwrapped + yaw_bias;
        const float target_b = b_link_offset + ratio * (yaw_unwrapped + yaw_bias);

        if (follow_pending) {
          const bool settled = fabsf(target_a - angle_a) < FOLLOW_POSITION_TOLERANCE &&
                               fabsf(target_b - angle_b) < FOLLOW_POSITION_TOLERANCE &&
                               fabsf(speed_a) < FOLLOW_SPEED_TOLERANCE &&
                               fabsf(speed_b) < FOLLOW_SPEED_TOLERANCE;
          settled_ms = settled ? settled_ms + 1 : 0;
          if (settled_ms >= FOLLOW_SETTLE_TIME_MS) {
            follow_pending = false;
            settled_ms = 0;
          }
        }

        torque_a = position_control(target_a, angle_a, speed_a);
        torque_b = position_control(target_b, angle_b, speed_b);
      }
      else if (right_switch == sp::DBusSwitchMode::UP) {
        const float yaw_delta = yaw_unwrapped - startup_yaw;
        const float target_a = startup_a + yaw_delta;
        const float target_b = startup_b + ratio * yaw_delta;
        torque_a = position_control(target_a, angle_a, speed_a);
        torque_b = position_control(target_b, angle_b, speed_b);
      }
    }
    send_motor_commands(torque_a, torque_b);
    send_motor_commands(torque_a, torque_b);
    osDelay(1);
  }
}