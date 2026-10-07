#include "cmsis_os.h"
#include "io/dbus/dbus.hpp"
#include "shared_data.hpp"

// C板：USART3 接 DBUS
sp::DBus remote(&huart3);

extern "C" void uart_task()
{
    remote.request();

    for (;;)
    {
        // 使用调试查看 remote 内部变量的变化
        osDelay(10);
    }
}

extern "C" void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
    auto stamp_ms = osKernelSysTick();

    if (huart == &huart3)
    {
        remote.update(Size, stamp_ms);

        g_data.sw_r = static_cast<uint8_t>(remote.sw_r);
        g_data.sw_l = static_cast<uint8_t>(remote.sw_l);

        remote.request();
    }
}

extern "C" void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    if (huart == &huart3)
    {
        remote.request();
    }
}