#include <cstdio>

#include "cmsis_os.h"
#include "io/bmi088/bmi088.hpp"
#include "tools/mahony/mahony.hpp"
#include "shared_data.hpp"

const float r_ab[3][3] = {{0.0f, -1.0f, 0.0f}, {1.0f, 0.0f, 0.0f}, {0.0f, 0.0f, 1.0f}};

sp::BMI088 bmi088(&hspi1, GPIOA, GPIO_PIN_4, GPIOB, GPIO_PIN_0, r_ab);
sp::Mahony imu(1e-3f);

extern "C" void imu_task()
{
    bmi088.init();
    uint32_t print_div = 0;

    for (;;)
    {
        bmi088.update();
        imu.update(bmi088.acc, bmi088.gyro);

        g_data.yaw = imu.yaw;

        if (++print_div >= 10)
        {
            print_div = 0;
            float yaw_deg = imu.yaw * 57.29578f;
            float pitch_deg = imu.pitch * 57.29578f;
            float roll_deg = imu.roll * 57.29578f;
            printf("%.2f %.2f %.2f\r\n", yaw_deg, pitch_deg, roll_deg);

            osDelay(1);
        }
    }