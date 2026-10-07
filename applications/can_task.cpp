#include <cmath>

#include "cmsis_os.h"
#include "io/can/can.hpp"
#include "motor/rm_motor/rm_motor.hpp"
#include "shared_data.hpp"
#include "tools/pid/pid.hpp"

sp::CAN can1(&hcan1);
sp::RM_Motor motor_a(1, sp::RM_Motors::GM6020); // A电机 ID=1
sp::RM_Motor motor_b(2, sp::RM_Motors::GM6020); // B电机 ID=2

// 位置环 PID：dt=1ms, kp=20, ki=0, kd=0.5, max_out=2.0, max_iout=0, alpha=1, angular=true, dynamic=false
sp::PID pid_a(1e-3f, 20.0f, 0.0f, 0.5f, 2.0f, 0.0f, 1.0f, true, false);
sp::PID pid_b(1e-3f, 20.0f, 0.0f, 0.5f, 2.0f, 0.0f, 1.0f, true, false);

namespace
{
    float last_yaw = 0.0f;
    float last_motor_a = 0.0f;
    float last_motor_b = 0.0f;
    float target_a = 0.0f;
    float target_b = 0.0f;
    uint8_t last_sw_r = 0xFF;

    constexpr float MANUAL_THRESHOLD = 0.001f; // 单位 rad/ms，约 57°/s

    void sync_update()
    {
        float yaw = g_data.yaw;
        uint8_t sw_r = g_data.sw_r;
        uint8_t sw_l = g_data.sw_l;

        bool mode_changed = (sw_r != last_sw_r);
        last_sw_r = sw_r;

        // 失能模式
        if (sw_r == 0)
        {
            motor_a.cmd(0.0f);
            motor_b.cmd(0.0f);
            return;
        }

        // 复位模式：两电机R标对齐C板R标
        if (sw_r == 2)
        {
            target_a = yaw;
            target_b = yaw;
            last_yaw = yaw;
            last_motor_a = motor_a.angle;
            last_motor_b = motor_b.angle;
        }

        // 中档：姿态联动模式
        if (sw_r == 1)
        {
            // 从其他模式切进来时，从当前位置开始追踪
            if (mode_changed)
            {
                last_yaw = yaw;
                last_motor_a = motor_a.angle;
                last_motor_b = motor_b.angle;
                target_a = motor_a.angle;
                target_b = motor_b.angle;
            }

            float k_b = (sw_l == 0) ? 0.5f : (sw_l == 1) ? -1.0f
                                                         : 3.0f;

            float delta_yaw = yaw - last_yaw;
            float delta_motor_a = motor_a.angle - last_motor_a;
            float delta_motor_b = motor_b.angle - last_motor_b;

            // 手动转动量 = 电机实际变化 - C板联动预期
            float manual_a = delta_motor_a - delta_yaw;
            float manual_b = delta_motor_b - delta_yaw * k_b;

            // C板联动
            target_a += delta_yaw;
            target_b += delta_yaw * k_b;

            // 手动转A：A跟随手，B按比例跟
            if (fabsf(manual_a) > MANUAL_THRESHOLD)
            {
                target_a += manual_a;
                target_b += manual_a * k_b;
            }

            // 手动转B：B跟随手，A按比例跟
            if (fabsf(manual_b) > MANUAL_THRESHOLD)
            {
                target_b += manual_b;
                target_a += manual_b / k_b;
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

        // 两个电机的指令都写入同一组 8 字节数据，共用 tx_id=0x1FE
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