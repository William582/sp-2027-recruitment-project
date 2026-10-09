#include <cmath>

#include "cmsis_os.h"
#include "io/can/can.hpp"
#include "io/dbus/dbus.hpp"
#include "motor/rm_motor/rm_motor.hpp"
#include "tools/mahony/mahony.hpp"
#include "tools/pid/pid.hpp"

extern sp::DBus remote;
extern sp::Mahony imu;
extern volatile bool imu_ready;

sp::CAN can1(&hcan1);
sp::RM_Motor motor_a(1, sp::RM_Motors::GM6020);
sp::RM_Motor motor_b(2, sp::RM_Motors::GM6020);

volatile float plot_motor_a_target_angle = 0.0f;
volatile float plot_motor_a_actual_angle = 0.0f;
volatile float plot_motor_b_target_angle = 0.0f;
volatile float plot_motor_b_actual_angle = 0.0f;

namespace
{
constexpr float PI = 3.14159265358979323846f;
constexpr float MOTOR_A_ENCODER_SIGN = 1.0f;
constexpr float MOTOR_B_ENCODER_SIGN = 1.0f;
constexpr bool MOTOR_DIRECTIONS_VERIFIED = true;

constexpr float RATED_TORQUE_SPEED_RPM = 132.0f;
constexpr float MAX_OUTPUT_SPEED = RATED_TORQUE_SPEED_RPM * (2.0f * PI / 60.0f);
constexpr float MAX_TORQUE = 1.2f;  //最大力矩
//Kp
constexpr float POSITION_TO_SPEED_GAIN = 0.0f;
constexpr float SPEED_TO_TORQUE_GAIN = 0.0f;
//time
constexpr float PID_DT = 0.001f;
//Ki
constexpr float POSITION_PID_KI = 0.00f;
constexpr float SPEED_PID_KI = 0.002f;
//积分项最大输出
constexpr float POSITION_PID_MAX_IOUT = 0.5f;
constexpr float SPEED_PID_MAX_IOUT = 0.05f;

constexpr float POSITION_PID_INTEGRAL_PAUSE = 2.0f;  // 2.0s内位置误差大于2.0rad时暂停积分
constexpr float SPEED_PID_INTEGRAL_PAUSE = 0.15f;    // 0.15s内速度误差大于0.15rad/s时暂停积分
constexpr float PID_D_FILTER_ALPHA = 0.1f;           //微分项滤波系数，alpha=1时不滤波

//电机A和电机B的角度环和速度环的PID控制器
sp::PID position_pid_a(
  PID_DT, POSITION_TO_SPEED_GAIN, POSITION_PID_KI, 0.0f, MAX_OUTPUT_SPEED, POSITION_PID_MAX_IOUT,
  PID_D_FILTER_ALPHA, false, true);
sp::PID speed_pid_a(
  PID_DT, SPEED_TO_TORQUE_GAIN, SPEED_PID_KI, 0.0f, MAX_TORQUE, SPEED_PID_MAX_IOUT,
  PID_D_FILTER_ALPHA, false, true);
sp::PID position_pid_b(
  PID_DT, POSITION_TO_SPEED_GAIN, POSITION_PID_KI, 0.0f, MAX_OUTPUT_SPEED, POSITION_PID_MAX_IOUT,
  PID_D_FILTER_ALPHA, false, true);
sp::PID speed_pid_b(
  PID_DT, SPEED_TO_TORQUE_GAIN, SPEED_PID_KI, 0.0f, MAX_TORQUE, SPEED_PID_MAX_IOUT,
  PID_D_FILTER_ALPHA, false, true);

constexpr uint32_t MANUAL_DETECT_WINDOW_MS = 50;
constexpr uint32_t FOLLOW_SETTLE_TIME_MS = 100;

//在检测时间窗内，若 yaw 变化量达到或超过 0.1rad，就认为 yaw 有动作并进入待跟随状态
constexpr float MANUAL_YAW_STABLE_DELTA = 0.1f;

//若某个电机角度变化量达到或超过 0.05，且另一个电机基本没动，则认为检测到该电机的手动转动。
constexpr float MANUAL_MOTOR_DELTA = 0.05f;
constexpr float OTHER_MOTOR_STABLE_DELTA = 0.015f;
//稳定条件：在待跟随状态下，若电机角度与目标角度的差值小于 0.03 rad，且电机速度小于 0.1 rad/s，则认为电机已稳定到位。
constexpr float FOLLOW_POSITION_TOLERANCE = 0.03f;
constexpr float FOLLOW_SPEED_TOLERANCE = 0.1f;
// 清零位置环和速度环的PID状态
void clear_control_pids()
{
  position_pid_a.clear();
  speed_pid_a.clear();
  position_pid_b.clear();
  speed_pid_b.clear();
}
// 将遥控器左拨杆的档位转换为联动比例
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

// 将电机反馈数据从CAN总线读取到电机对象中
void send_motor_commands(float torque_a, float torque_b)
{
  for (uint8_t i = 0; i < sp::CAN_DATA_LEN; i++) {
    can1.tx_data[i] = 0;
  }

  motor_a.cmd(MOTOR_A_ENCODER_SIGN * torque_a);
  motor_b.cmd(MOTOR_B_ENCODER_SIGN * torque_b);
  motor_a.write(can1.tx_data);
  motor_b.write(can1.tx_data);
  can1.send(motor_a.tx_id);
}

// 读取电机反馈数据
void read_motor_feedback(uint32_t stamp_ms)
{
  while (HAL_CAN_GetRxFifoFillLevel(&hcan1, CAN_RX_FIFO0) > 0) {
    can1.recv(CAN_RX_FIFO0);
    if (can1.rx_id == motor_a.rx_id) {
      motor_a.read(can1.rx_data, stamp_ms);
    }
    else if (can1.rx_id == motor_b.rx_id) {
      motor_b.read(can1.rx_data, stamp_ms);
    }
  }
}

// 将角度变化量限制在 [-pi, pi] 范围内, 以便处理角度环绕问题
float unwrap_delta(float delta)
{
  if (delta > PI) return delta - 2.0f * PI;
  if (delta < -PI) return delta + 2.0f * PI;
  return delta;
}
}  // namespace
// CAN通信任务函数
extern "C" void can_task(void const * argument)
{
  (void)argument;
  osDelay(500);
  can1.config();
  can1.start();

  bool have_yaw = false;        // 只有当IMU正常工作时，才认为有有效的yaw数据
  bool baseline_ready = false;  // 只有当遥控器、两台电机和IMU都正常工作时，才认为基线已准备好
  bool have_window = false;
  bool follow_pending = false;
  bool have_right_switch = false;
  float last_yaw_raw = 0.0f;   // 记录上一次的原始yaw值，用于计算角度变化量
  float yaw_unwrapped = 0.0f;  // 记录解包后的yaw值
  float startup_yaw = 0.0f;    // 记录启动时的yaw值
  float startup_a = 0.0f;      // 记录启动时的电机A角度
  float startup_b = 0.0f;      // 记录启动时的电机B角度
  float yaw_bias = 0.0f;       // 记录yaw偏置
  float a_link_offset = 0.0f;  // 记录电机A连杆偏置
  float b_link_offset = 0.0f;  // 记录电机B连杆偏置
  float link_ratio = 0.5f;     // 记录联动比例
  float window_yaw = 0.0f;     // 记录窗口内的yaw值
  float window_a = 0.0f;       // 记录窗口内的电机A角度
  float window_b = 0.0f;       // 记录窗口内的电机B角度
  uint32_t window_start_ms = 0;
  uint32_t settled_ms = 0;
  sp::DBusSwitchMode last_right_switch = sp::DBusSwitchMode::DOWN;

  while (true) {
    //1.首先判断遥控器、IMU和电机是否正常工作，随后读取yaw值
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

    plot_motor_a_target_angle = angle_a;
    plot_motor_a_actual_angle = angle_a;
    plot_motor_b_target_angle = angle_b;
    plot_motor_b_actual_angle = angle_b;

    if (!baseline_ready && remote_alive && motors_alive && have_yaw) {
      const float initial_ratio = ratio_from_switch(remote.sw_l);
      startup_yaw = yaw_unwrapped;
      startup_a = angle_a;
      startup_b = angle_b;
      link_ratio = initial_ratio;
      a_link_offset = angle_a - yaw_unwrapped;  //初始C板和A电机的角度差
      b_link_offset = angle_b - initial_ratio * yaw_unwrapped;
      baseline_ready = true;
    }

    float torque_a = 0.0f;
    float torque_b = 0.0f;  //设定力矩变量

    const bool control_ready =
      baseline_ready && remote_alive && motors_alive && have_yaw && MOTOR_DIRECTIONS_VERIFIED;

    if (control_ready) {
      const sp::DBusSwitchMode right_switch = remote.sw_r;
      const float ratio = ratio_from_switch(remote.sw_l);
      //当第一次检测到右拨杆的档位或右拨杆档位发生变化时，清零PID状态，并根据档位设置初始偏置和联动比例
      if (!have_right_switch || right_switch != last_right_switch) {
        clear_control_pids();
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
      }  //完成初始化配置

      //当右拨杆处于中档时，进行联动控制
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
          //进入AB电机手动跟随模式
          if (fabsf(yaw_delta) >= MANUAL_YAW_STABLE_DELTA) {
            follow_pending = true;
            settled_ms = 0;
          }
          else if (!follow_pending) {
            if (
              fabsf(delta_a) >= MANUAL_MOTOR_DELTA && fabsf(delta_b) <= OTHER_MOTOR_STABLE_DELTA) {
              yaw_bias += delta_a;
              follow_pending = true;
              settled_ms = 0;
            }
            else if (
              fabsf(delta_b) >= MANUAL_MOTOR_DELTA && fabsf(delta_a) <= OTHER_MOTOR_STABLE_DELTA) {
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
        plot_motor_a_target_angle = target_a;
        plot_motor_b_target_angle = target_b;

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

        position_pid_a.calc(target_a, angle_a, POSITION_PID_INTEGRAL_PAUSE);
        speed_pid_a.calc(position_pid_a.out, speed_a, SPEED_PID_INTEGRAL_PAUSE);
        position_pid_b.calc(target_b, angle_b, POSITION_PID_INTEGRAL_PAUSE);
        speed_pid_b.calc(position_pid_b.out, speed_b, SPEED_PID_INTEGRAL_PAUSE);
        torque_a = speed_pid_a.out;
        torque_b = speed_pid_b.out;
      }
      else if (right_switch == sp::DBusSwitchMode::UP) {
        const float yaw_delta = yaw_unwrapped - startup_yaw;
        const float target_a = startup_a + yaw_delta;
        const float target_b = startup_b + ratio * yaw_delta;
        plot_motor_a_target_angle = target_a;
        plot_motor_b_target_angle = target_b;
        position_pid_a.calc(target_a, angle_a, POSITION_PID_INTEGRAL_PAUSE);
        speed_pid_a.calc(position_pid_a.out, speed_a, SPEED_PID_INTEGRAL_PAUSE);
        position_pid_b.calc(target_b, angle_b, POSITION_PID_INTEGRAL_PAUSE);
        speed_pid_b.calc(position_pid_b.out, speed_b, SPEED_PID_INTEGRAL_PAUSE);
        torque_a = speed_pid_a.out;
        torque_b = speed_pid_b.out;
      }
    }
    else {
      clear_control_pids();
    }
    send_motor_commands(torque_a, torque_b);
    osDelay(1);
  }
}