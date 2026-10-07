#include <cmath>

#include "cmsis_os.h"
#include "io/can/can.hpp"
#include "motor/rm_motor/rm_motor.hpp"
#include "shared_data.hpp"
#include "tools/pid/pid.hpp"

sp::CAN can1(&hcan1);
sp::RM_Motor motor_a(1, sp::RM_Motors::GM6020); // A电机 ID=1
sp::RM_Motor motor_b(2, sp::RM_Motors::GM6020); // B电机 ID=2

// angular=false（多圈角度），dynamic=false
sp::PID pid_a(1e-3f, 20.0f, 0.0f, 0.5f, 2.0f, 0.0f, 1.0f, false, false);
sp::PID pid_b(1e-3f, 20.0f, 0.0f, 0.5f, 2.0f, 0.0f, 1.0f, false, false);

namespace
{
    float last_yaw = 0.0f;
    float last_motor_a = 0.0f;
    float last_motor_b = 0.0f;
    float target_a = 0.0f;
    float target_b = 0.0f;
    uint8_t last_sw_r = 0xFF;
    uint32_t last_manual_ms = 0;

    constexpr float MANUAL_THRESHOLD = 0.001f;     // 手动转动检测阈值 (rad/ms ≈ 57°/s)
    constexpr float YAW_STILL_THRESHOLD = 0.0001f; // C板静止判定阈值 (rad/ms ≈ 5.7°/s)
    constexpr uint32_t MANUAL_COOLDOWN_MS = 50;    // 手动检测冷却时间

    void sync_update()
    {
        float yaw = g_data.yaw;
        uint8_t sw_r = g_data.sw_r;
        uint8_t sw_l = g_data.sw_l;

        bool mode_changed = (sw_r != last_sw_r);
        last_sw_r = sw_r;

        // 下档：失能
        if (sw_r == 0)
        {
            motor_a.cmd(0.0f);
            motor_b.cmd(0.0f);
            return;
        }

        // 上档：复位，两电机R标对齐C板R标
        if (sw_r == 2)
        {
            target_a = yaw;
            target_b = yaw;
            last_yaw = yaw;
            last_motor_a = motor_a.angle;
            last_motor_b = motor_b.angle;
        }

        // 中档：姿态联动
        if (sw_r == 1)
        {
            // 从其他模式切进来时，从当前位置开始追踪，并进入冷却
            if (mode_changed)
            {
                last_yaw = yaw;
                last_motor_a = motor_a.angle;
                last_motor_b = motor_b.angle;
                target_a = motor_a.angle;
                target_b = motor_b.angle;
                last_manual_ms = osKernelSysTick();
            }

            float k_b = (sw_l == 0) ? 0.5f : (sw_l == 1) ? -1.0f
                                                         : 3.0f;

            float delta_yaw = yaw - last_yaw;
            float actual_delta_a = motor_a.angle - last_motor_a;
            float actual_delta_b = motor_b.angle - last_motor_b;

            // C板联动：C板转多少，电机跟多少
            target_a += delta_yaw;
            target_b += delta_yaw * k_b;

            // 手动转动检测：仅C板静止时
            if (fabsf(delta_yaw) < YAW_STILL_THRESHOLD)
            {
                bool manual_a = fabsf(actual_delta_a) > MANUAL_THRESHOLD;
                bool manual_b = fabsf(actual_delta_b) > MANUAL_THRESHOLD;

                uint32_t now_ms = osKernelSysTick();
                bool in_cooldown = (now_ms - last_manual_ms) < MANUAL_COOLDOWN_MS;

                // 本轴目标始终跟随实际位置（手感顺滑）
                // 但只在非冷却期才把增量传播到另一轴（防止反馈回路）
                if (manual_a && !manual_b)
                {
                    target_a = motor_a.angle;
                    if (!in_cooldown)
                    {
                        target_b += actual_delta_a * k_b;
                        last_manual_ms = now_ms;
                    }
                }
                else if (manual_b && !manual_a)
                {
                    target_b = motor_b.angle;
                    if (!in_cooldown)
                    {
                        target_a += actual_delta_b / k_b;
                        last_manual_ms = now_ms;
                    }
                }
                // 两个同时触发 → 说明是PID响应另一轴，忽略
            }

            last_yaw = yaw;
            last_motor_a = motor_a.angle;
            last_motor_b = motor_b.angle;
        }

        pid_a.calc(target_a, motor_a.angle);
        pid_b.calc(target_b, motor_b.angle);

        motor_a.cmd(pid_a.out);
        motor_b.cmd(pid_b.out);
    }
} // namespace

extern "C" void can_task()
{
    can1.config();
    can1.start();

    for (;;)
    {
        g_data.angle_a = motor_a.angle;
        g_data.angle_b = motor_b.angle;

        sync_update();

        motor_a.write(can1.tx_data);
        motor_b.write(can1.tx_data);
        can1.send(motor_a.tx_id);

        osDelay(1);
    }
}

extern "C" void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    auto stamp_ms = osKernelSysTick();

    while (HAL_CAN_GetRxFifoFillLevel(hcan, CAN_RX_FIFO0) > 0)
    {
        if (hcan == &hcan1)
        {
            can1.recv();

            if (can1.rx_id == motor_a.rx_id)
            {
                motor_a.read(can1.rx_data, stamp_ms);
            }
            else if (can1.rx_id == motor_b.rx_id)
            {
                motor_b.read(can1.rx_data, stamp_ms);
            }
        }
    }
}

extern "C" void HAL_CAN_RxFifo1MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    // FIFO1 未使用，空实现防止未处理中断
    (void)hcan;
}