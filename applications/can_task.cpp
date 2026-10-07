#include "cmsis_os.h"
#include "io/can/can.hpp"
#include "motor/rm_motor/rm_motor.hpp"

sp::CAN can1(&hcan1);
sp::RM_Motor motor6020(1, sp::RM_Motors::GM6020); // 电机ID=1, 电流控制模式

extern "C" void can_task()
{
    can1.config();
    can1.start();

    for (;;)
    {
        motor6020.cmd(0.0f); // 先发0电流，验证通信正常

        motor6020.write(can1.tx_data);
        can1.send(motor6020.tx_id);

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

            if (can1.rx_id == motor6020.rx_id)
            {
                motor6020.read(can1.rx_data, stamp_ms);
            }
        }
    }
}