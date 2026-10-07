#include "cmsis_os.h"
#include "io/can/can.hpp"
#include "motor/rm_motor/rm_motor.hpp"
#include "shared_data.hpp"

sp::CAN can1(&hcan1);
sp::RM_Motor motor_a(1, sp::RM_Motors::GM6020); // A电机 ID=1
sp::RM_Motor motor_b(2, sp::RM_Motors::GM6020); // B电机 ID=2

extern "C" void can_task()
{
    can1.config();
    can1.start();

    for (;;)
    {
        // 写角度到共享数据
        g_data.angle_a = motor_a.angle;
        g_data.angle_b = motor_b.angle;

        // 暂时发0力矩，验证双电机通信
        motor_a.cmd(0.0f);
        motor_b.cmd(0.0f);

        motor_a.write(can1.tx_data);
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