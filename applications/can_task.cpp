#include <cmath>

#include "cmsis_os.h"
#include "io/can/can.hpp"
#include "motor/rm_motor/rm_motor.hpp"
#include "shared_data.hpp"
#include "tools/pid/pid.hpp"

sp::CAN can1(&hcan1);
sp::RM_Motor motor_a(1, sp::RM_Motors::GM6020);
sp::RM_Motor motor_b(2, sp::RM_Motors::GM6020);

sp::PID pid_a(1e-3f, 3.0f, 0.0f, 0.0f, 0.8f, 0.0f, 1.0f, false, false);
sp::PID pid_b(1e-3f, 3.0f, 0.0f, 0.0f, 0.8f, 0.0f, 1.0f, false, false);
namespace
{
float yaw_accum = 0.0f;  // 累积的 yaw（处理回绕）
float yaw_ref = 0.0f;    // 联动参考点
float motor_a_ref = 0.0f;
float motor_b_ref = 0.0f;

float last_yaw = 0.0f;
float last_motor_a = 0.0f;
float last_motor_b = 0.0f;
uint8_t last_sw_r = 0xFF;
uint8_t last_sw_l = 0xFF;
uint32_t yaw_move_ms = 0;

constexpr float MANUAL_THRESHOLD = 0.01f;      // 手动转动阈值（约 573°/s）
constexpr float YAW_MOVE_THRESHOLD = 0.0005f;  // C板转动判定（约 28°/s）
constexpr uint32_t YAW_SETTLE_MS = 100;
constexpr float PI_F = 3.14159265f;

void sync_update()
{
  float yaw = g_data.yaw;
  uint8_t sw_r = g_data.sw_r;
  uint8_t sw_l = g_data.sw_l;

  bool mode_changed = (sw_r != last_sw_r);
  last_sw_r = sw_r;

  // 下档：失能
  if (sw_r == 0) {
    motor_a.cmd(0.0f);
    motor_b.cmd(0.0f);
    return;
  }

  uint32_t now_ms = osKernelSysTick();

  // === yaw 回绕处理：累积成连续角度 ===
  float delta_yaw_raw = yaw - last_yaw;
  if (delta_yaw_raw > PI_F) delta_yaw_raw -= 2 * PI_F;
  if (delta_yaw_raw < -PI_F) delta_yaw_raw += 2 * PI_F;
  yaw_accum += delta_yaw_raw;
  last_yaw = yaw;

  // === k_b 计算 ===
  // sw_l: 0=下档 → 0.5, 1=中档 → -1, 2=上档 → 3
  float k_b = (sw_l == 0) ? 0.5f : (sw_l == 1) ? -1.0f : 3.0f;

  // === 左拨杆切换：重算 motor_b_ref，避免目标突变 ===
  if (sw_l != last_sw_l) {
    last_sw_l = sw_l;
    // 保持 B 电机当前角度不变：motor_b_ref = 当前角度 - (yaw_accum - yaw_ref) * 新k_b
    motor_b_ref = motor_b.angle - (yaw_accum - yaw_ref) * k_b;
  }

  // === 上档：复位 ===
  if (sw_r == 2) {
    yaw_ref = yaw_accum;
    motor_a_ref = yaw_accum;
    motor_b_ref = yaw_accum;
    yaw_move_ms = now_ms;

    pid_a.calc(yaw_accum, motor_a.angle);
    pid_b.calc(yaw_accum, motor_b.angle);
    motor_a.cmd(pid_a.out);
    motor_b.cmd(pid_b.out);
    return;
  }

  // === 中档：姿态联动 ===
  if (mode_changed) {
    yaw_ref = yaw_accum;
    motor_a_ref = motor_a.angle;
    motor_b_ref = motor_b.angle;
    last_motor_a = motor_a.angle;
    last_motor_b = motor_b.angle;
    yaw_move_ms = now_ms;
  }

  float delta_a = motor_a.angle - last_motor_a;
  float delta_b = motor_b.angle - last_motor_b;

  if (fabsf(delta_yaw_raw) > YAW_MOVE_THRESHOLD) {
    yaw_move_ms = now_ms;
  }

  // === 手动转动检测：C板停下超过 100ms ===
  bool yaw_settled = (now_ms - yaw_move_ms) > YAW_SETTLE_MS;

  if (yaw_settled) {
    if (fabsf(delta_a) > MANUAL_THRESHOLD && fabsf(delta_b) < MANUAL_THRESHOLD) {
      motor_a_ref += delta_a;
      motor_b_ref += delta_a * k_b;
    }
    else if (fabsf(delta_b) > MANUAL_THRESHOLD && fabsf(delta_a) < MANUAL_THRESHOLD) {
      motor_b_ref += delta_b;
      motor_a_ref += delta_b / k_b;
    }
  }

  last_motor_a = motor_a.angle;
  last_motor_b = motor_b.angle;

  // === 基于绝对参考计算目标（用 yaw_accum，无回绕）===
  float target_a = motor_a_ref + (yaw_accum - yaw_ref);
  float target_b = motor_b_ref + (yaw_accum - yaw_ref) * k_b;

  pid_a.calc(target_a, motor_a.angle);
  pid_b.calc(target_b, motor_b.angle);

  motor_a.cmd(pid_a.out);
  motor_b.cmd(pid_b.out);
}
}  // namespace

extern "C" void can_task()
{
  can1.config();
  can1.start();

  for (;;) {
    g_data.angle_a = motor_a.angle;
    g_data.angle_b = motor_b.angle;

    sync_update();

    motor_a.write(can1.tx_data);
    motor_b.write(can1.tx_data);
    can1.send(motor_a.tx_id);

    osDelay(1);
  }
}

extern "C" void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef * hcan)
{
  auto stamp_ms = osKernelSysTick();

  while (HAL_CAN_GetRxFifoFillLevel(hcan, CAN_RX_FIFO0) > 0) {
    if (hcan == &hcan1) {
      can1.recv();

      if (can1.rx_id == motor_a.rx_id) {
        motor_a.read(can1.rx_data, stamp_ms);
      }
      else if (can1.rx_id == motor_b.rx_id) {
        motor_b.read(can1.rx_data, stamp_ms);
      }
    }
  }
}

extern "C" void HAL_CAN_RxFifo1MsgPendingCallback(CAN_HandleTypeDef * hcan) { (void)hcan; }