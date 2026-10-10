/*4.4.1.1. C板基本功能
基本功能考核作为晋级的必要条件；未完成本项要求者，不予晋级。
选手须自行完成硬件接线，并使用无线烧录器实现蜂鸣器控制、LED控制、串口打印、遥控器控制等相关功能。
1. 蜂鸣器控制：当成功烧录或c板上电后，需要有蜂鸣器提示音，音乐可自定义。
2. LED控制：c板led灯需亮起流水灯灯效，并能够以此判断程序是否堵塞。
3. 串口打印：需在上位机软件中打印imu三轴数据。
4. 遥控器控制：使c板与dt7遥控器能够正常通讯。

4.4.1.2. 姿态与电机联动
1. 当右拨杆处于下档时，系统进入失能模式，即down档，所有电机应处于无力状态。
2. 右拨杆处于中档时，系统进入姿态联动模式
  通过C板imu获取姿态信息，控制两台6020电机完成不同的运动。
  C板及两台电机构成三个旋转输入端（输入的值分别为：C板imu的yaw角度，两个电机编码器数值）。将与C板保持1∶1联动比例的电机记为A电机，另一台记为B电机。B电机的联动比例由左拨杆档位决定：
  其中，负号表示反向转动。
  联动控制应满足以下要求：
  1. 将C板绕yaw轴转动时，两台电机按照设定比例跟随运动。
  2. 手动转动任一电机时，另一台电机按照对应比例跟随运动；C板的实际偏航角（Yaw）保持不变。手动转动电机后，系统应同步调整C板的联动参考零点，该参考零点随A电机的位置变化。
  3. 再次转动C板时，两台电机应基于调整后的参考零点继续联动，不应自动返回原零位。(即不会像方向盘一样回正）
  4. c板和其中一块6020保持1:1联动比（减速比）,  另一块6020和c板的联动比（减速比）和左拨杆有关:下档1:0.5   中档1:-1   上挡1:3
    （例如在左拨杆上档1:3时，c板绕yaw轴逆时针转60度，A电机逆时针转60度，B电机逆时针转180度）
3. 右拨杆切换至上档时，进入复位模式
触发复位功能，两台电机上的R标方向对齐C板上的R标方向。*/
#include <cmath>
#include <cstdint>

#include "cmsis_os.h"
#include "io/can/can.hpp"
#include "io/dbus/dbus.hpp"
#include "motor/rm_motor/rm_motor.hpp"
#include "tools/mahony/mahony.hpp"
#include "tools/pid/pid.hpp"

extern sp::DBus remote;
extern sp::Mahony imu;
extern volatile bool imu_ready;

// ============================================================
// 1. CAN 与电机对象
// ============================================================

sp::CAN can1(&hcan1);

sp::RM_Motor motor_a(1, sp::RM_Motors::GM6020);
sp::RM_Motor motor_b(2, sp::RM_Motors::GM6020);

// 上位机绘图变量
volatile float plot_motor_a_target_angle = 0.0f;
volatile float plot_motor_a_actual_angle = 0.0f;
volatile float plot_motor_b_target_angle = 0.0f;
volatile float plot_motor_b_actual_angle = 0.0f;

namespace
{

// ============================================================
// 2. 参数配置
// ============================================================

constexpr float PI = 3.14159265358979323846f;
constexpr float TWO_PI = 2.0f * PI;

// 电机逻辑方向；若实际方向相反，修改为 -1.0f
constexpr float MOTOR_A_ENCODER_SIGN = 1.0f;
constexpr float MOTOR_B_ENCODER_SIGN = 1.0f;

// 控制周期
constexpr float PID_DT = 0.001f;

// 速度环输出限幅，即最终电机控制量限幅
constexpr float MAX_TORQUE = 1.2f;

// 位置环输出限幅，即目标速度限幅，单位 rad/s
constexpr float MAX_TARGET_SPEED = 3.0f;

// 位置外环 PID 参数
constexpr float POSITION_PID_KP = 1.2f;
constexpr float POSITION_PID_KI = 0.0f;
constexpr float POSITION_PID_KD = 0.0f;

// 速度内环 PID 参数

constexpr float SPEED_PID_KP = 0.2f;
constexpr float SPEED_PID_KI = 0.0f;
constexpr float SPEED_PID_KD = 0.0f;

// 手动转动检测参数
constexpr float YAW_STATIONARY_THRESHOLD = 0.0002f;
constexpr float MANUAL_MOTOR_DELTA = 0.05f;
constexpr float OTHER_MOTOR_STABLE_DELTA = 0.015f;

// ============================================================
// 3. PID 对象
// ============================================================

// 位置外环：位置误差 -> 目标速度
sp::PID position_pid_a(
  PID_DT, POSITION_PID_KP, POSITION_PID_KI, POSITION_PID_KD, MAX_TARGET_SPEED, 0.0f, 0.1f);

sp::PID position_pid_b(
  PID_DT, POSITION_PID_KP, POSITION_PID_KI, POSITION_PID_KD, MAX_TARGET_SPEED, 0.0f, 0.1f);

// 速度内环：速度误差 -> 最终控制量
sp::PID speed_pid_a(PID_DT, SPEED_PID_KP, SPEED_PID_KI, SPEED_PID_KD, MAX_TORQUE, 0.0f, 0.1f);

sp::PID speed_pid_b(PID_DT, SPEED_PID_KP, SPEED_PID_KI, SPEED_PID_KD, MAX_TORQUE, 0.0f, 0.1f);

// ============================================================
// 4. 基础函数
// ============================================================

// 清除全部 PID 状态
void clear_control_pids()
{
  position_pid_a.clear();
  position_pid_b.clear();

  speed_pid_a.clear();
  speed_pid_b.clear();
}

// 统一 PID 计算接口
float position_control(sp::PID & pid, float target, float actual)
{
  pid.calc(target, actual);
  return pid.out;
}

// 单圈角度差解包至 [-PI, PI]
float unwrap_delta(float delta)
{
  while (delta > PI) {
    delta -= TWO_PI;
  }

  while (delta < -PI) {
    delta += TWO_PI;
  }

  return delta;
}

// 将单圈角度转为连续角度
// 注意：相邻两次有效反馈之间的实际转角必须小于 PI
float update_unwrapped_angle(
  float raw_angle, float & last_raw_angle, float & continuous_angle, bool & initialized)
{
  if (!initialized) {
    last_raw_angle = raw_angle;
    continuous_angle = raw_angle;
    initialized = true;
    return 0.0f;
  }

  const float delta = unwrap_delta(raw_angle - last_raw_angle);

  continuous_angle += delta;
  last_raw_angle = raw_angle;

  return delta;
}

// 左拨杆对应 B 电机的联动比例
float ratio_from_switch(sp::DBusSwitchMode mode)
{
  switch (mode) {
    case sp::DBusSwitchMode::DOWN:
      return 0.5f;

    case sp::DBusSwitchMode::MID:
      return -1.0f;

    case sp::DBusSwitchMode::UP:
      return 3.0f;

    default:
      return 0.5f;
  }
}

// ============================================================
// 5. 双环控制函数
// ============================================================

// 位置外环输出目标速度；速度内环输出最终控制量
float dual_loop_control(
  sp::PID & position_pid, sp::PID & speed_pid, float target_angle, float actual_angle,
  float actual_speed)
{
  const float target_speed = position_control(position_pid, target_angle, actual_angle);

  return position_control(speed_pid, target_speed, actual_speed);
}

// ============================================================
// 6. CAN 电机通信
// ============================================================

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

}  // namespace

// ============================================================
// 7. CAN 控制任务
// ============================================================

extern "C" void can_task(void const * argument)
{
  (void)argument;

  osDelay(500);

  can1.config();
  can1.start();

  // --------------------------------------------------------
  // A. IMU 连续角度
  // --------------------------------------------------------

  bool have_yaw = false;

  float last_yaw_raw = 0.0f;
  float yaw_unwrapped = 0.0f;

  // --------------------------------------------------------
  // B. A 电机连续角度
  // --------------------------------------------------------

  bool motor_a_angle_ready = false;

  float motor_a_last_raw = 0.0f;
  float motor_a_unwrapped_raw = 0.0f;

  // --------------------------------------------------------
  // C. B 电机连续角度
  // --------------------------------------------------------

  bool motor_b_angle_ready = false;

  float motor_b_last_raw = 0.0f;
  float motor_b_unwrapped_raw = 0.0f;

  // --------------------------------------------------------
  // D. 联动状态变量
  // --------------------------------------------------------

  bool baseline_ready = false;
  bool have_right_switch = false;

  sp::DBusSwitchMode last_right_switch = sp::DBusSwitchMode::DOWN;

  // 首次初始化时记录的位置，作为复位目标
  float startup_a = 0.0f;
  float startup_b = 0.0f;

  // 统一虚拟联动角度
  float link_angle = 0.0f;

  // 电机位置偏置
  float a_link_offset = 0.0f;
  float b_link_offset = 0.0f;

  // B 电机联动比例
  float link_ratio = 0.5f;

  // ========================================================
  // 主循环
  // ========================================================

  while (true) {
    // ----------------------------------------------------
    // 1. 读取电机反馈及设备状态
    // ----------------------------------------------------

    const uint32_t now_ms = HAL_GetTick();

    read_motor_feedback(now_ms);

    const bool remote_alive = remote.is_alive(now_ms);
    const bool motor_a_alive = motor_a.is_alive(now_ms);
    const bool motor_b_alive = motor_b.is_alive(now_ms);
    const bool motors_alive = motor_a_alive && motor_b_alive;

    // ----------------------------------------------------
    // 2. 更新 IMU 连续 Yaw
    // ----------------------------------------------------

    float yaw_step = 0.0f;

    if (imu_ready) {
      yaw_step = update_unwrapped_angle(imu.yaw, last_yaw_raw, yaw_unwrapped, have_yaw);
    }

    // ----------------------------------------------------
    // 3. 更新两台电机连续角度
    // ----------------------------------------------------

    if (motor_a_alive) {
      update_unwrapped_angle(
        motor_a.angle, motor_a_last_raw, motor_a_unwrapped_raw, motor_a_angle_ready);
    }

    if (motor_b_alive) {
      update_unwrapped_angle(
        motor_b.angle, motor_b_last_raw, motor_b_unwrapped_raw, motor_b_angle_ready);
    }

    const float angle_a = motor_a_angle_ready ? MOTOR_A_ENCODER_SIGN * motor_a_unwrapped_raw : 0.0f;
    const float angle_b = motor_b_angle_ready ? MOTOR_B_ENCODER_SIGN * motor_b_unwrapped_raw : 0.0f;

    // ----------------------------------------------------
    // 4. 读取速度反馈
    // ----------------------------------------------------
    // 速度反馈方向必须和对应的逻辑角度方向一致。

    const float actual_speed_a = MOTOR_A_ENCODER_SIGN * motor_a.speed;

    const float actual_speed_b = MOTOR_B_ENCODER_SIGN * motor_b.speed;

    // 上位机绘图
    plot_motor_a_actual_angle = angle_a;
    plot_motor_b_actual_angle = angle_b;

    plot_motor_a_target_angle = angle_a;
    plot_motor_b_target_angle = angle_b;

    // ----------------------------------------------------
    // 5. 初始化联动基准
    // ----------------------------------------------------

    if (
      !baseline_ready && remote_alive && motors_alive && have_yaw && motor_a_angle_ready &&
      motor_b_angle_ready) {
      // 假设初始化时已经将机械 R 标对齐
      startup_a = angle_a;
      startup_b = angle_b;

      link_angle = yaw_unwrapped;

      link_ratio = ratio_from_switch(remote.sw_l);

      a_link_offset = angle_a - link_angle;

      b_link_offset = angle_b - link_ratio * link_angle;

      baseline_ready = true;
    }

    // ----------------------------------------------------
    // 6. 设备状态检查
    // ----------------------------------------------------

    const bool control_ready = baseline_ready && remote_alive && motors_alive && have_yaw &&
                               imu_ready && motor_a_angle_ready && motor_b_angle_ready;

    float torque_a = 0.0f;
    float torque_b = 0.0f;

    if (!control_ready) {
      clear_control_pids();

      have_right_switch = false;

      send_motor_commands(0.0f, 0.0f);

      osDelay(1);
      continue;
    }

    // ----------------------------------------------------
    // 7. 获取遥控器档位
    // ----------------------------------------------------

    const sp::DBusSwitchMode right_switch = remote.sw_r;

    const float ratio = ratio_from_switch(remote.sw_l);

    bool entered_mid = false;

    // ----------------------------------------------------
    // 8. 处理右拨杆切换
    // ----------------------------------------------------

    if (!have_right_switch || right_switch != last_right_switch) {
      clear_control_pids();

      if (right_switch == sp::DBusSwitchMode::MID) {
        // 切入联动时，当前位置作为新的参考
        link_angle = yaw_unwrapped;

        link_ratio = ratio;

        a_link_offset = angle_a - link_angle;

        b_link_offset = angle_b - link_ratio * link_angle;

        entered_mid = true;
      }

      last_right_switch = right_switch;
      have_right_switch = true;
    }

    // ====================================================
    // 9. 三种控制模式
    // ====================================================

    switch (right_switch) {
        // ----------------------------------------------------
        // DOWN：失能模式
        // ----------------------------------------------------

      case sp::DBusSwitchMode::DOWN:

        torque_a = 0.0f;
        torque_b = 0.0f;

        clear_control_pids();

        break;

        // ----------------------------------------------------
        // MID：姿态联动模式
        // ----------------------------------------------------

      case sp::DBusSwitchMode::MID: {
        // 更新虚拟联动角度
        if (!entered_mid) {
          link_angle += yaw_step;
        }

        // -----------------------------------------------
        // 9.1 左拨杆比例切换
        // -----------------------------------------------

        if (std::fabs(ratio - link_ratio) > 1e-6f) {
          // 切换比例时保持 B 电机目标位置连续
          b_link_offset = angle_b - ratio * link_angle;

          link_ratio = ratio;

          clear_control_pids();
        }

        // -----------------------------------------------
        // 9.2 计算初始目标
        // -----------------------------------------------

        float target_a = a_link_offset + link_angle;

        float target_b = b_link_offset + ratio * link_angle;

        // -----------------------------------------------
        // 9.3 手动转动检测
        // -----------------------------------------------

        if (std::fabs(yaw_step) < YAW_STATIONARY_THRESHOLD) {
          const float error_a = target_a - angle_a;

          const float error_b = target_b - angle_b;

          // A 电机被手动转动，B 电机基本稳定
          if (
            std::fabs(error_a) >= MANUAL_MOTOR_DELTA &&
            std::fabs(error_b) <= OTHER_MOTOR_STABLE_DELTA) {
            link_angle -= error_a;
          }

          // B 电机被手动转动，A 电机基本稳定
          else if (
            std::fabs(error_b) >= MANUAL_MOTOR_DELTA &&
            std::fabs(error_a) <= OTHER_MOTOR_STABLE_DELTA) {
            link_angle -= error_b / ratio;
          }
        }

        // -----------------------------------------------
        // 9.4 重新计算两台电机的目标位置
        // -----------------------------------------------

        target_a = a_link_offset + link_angle;

        target_b = b_link_offset + ratio * link_angle;

        plot_motor_a_target_angle = target_a;
        plot_motor_b_target_angle = target_b;

        // -----------------------------------------------
        // 9.5 双环控制
        // -----------------------------------------------

        torque_a =
          dual_loop_control(position_pid_a, speed_pid_a, target_a, angle_a, actual_speed_a);

        torque_b =
          dual_loop_control(position_pid_b, speed_pid_b, target_b, angle_b, actual_speed_b);

        break;
      }

        // ----------------------------------------------------
        // UP：复位模式
        // ----------------------------------------------------

      case sp::DBusSwitchMode::UP: {
        // 回到首次初始化时记录的位置。
        // 初始化位置只有在机械 R 标已经对齐时
        // 才能作为机械复位位置。

        const float target_a = startup_a;
        const float target_b = startup_b;

        plot_motor_a_target_angle = target_a;
        plot_motor_b_target_angle = target_b;

        // 位置外环 + 速度内环
        torque_a =
          dual_loop_control(position_pid_a, speed_pid_a, target_a, angle_a, actual_speed_a);

        torque_b =
          dual_loop_control(position_pid_b, speed_pid_b, target_b, angle_b, actual_speed_b);

        break;
      }

      default:

        torque_a = 0.0f;
        torque_b = 0.0f;

        clear_control_pids();

        break;
    }

    // ----------------------------------------------------
    // 10. 发送电机控制指令
    // ----------------------------------------------------

    send_motor_commands(torque_a, torque_b);

    osDelay(1);
  }
}