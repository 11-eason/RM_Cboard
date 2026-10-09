#include "cmsis_os.h"
#include "io/bmi088/bmi088.hpp"
#include "io/plotter/plotter.hpp"
#include "shared_data.hpp"
#include "tools/mahony/mahony.hpp"
#include "usart.h"

const float r_ab[3][3] = {{0.0f, -1.0f, 0.0f}, {1.0f, 0.0f, 0.0f}, {0.0f, 0.0f, 1.0f}};

sp::BMI088 bmi088(&hspi1, GPIOA, GPIO_PIN_4, GPIOB, GPIO_PIN_0, r_ab);
sp::Mahony imu(1e-3f);
sp::Plotter plotter(&huart1, false);

extern "C" void imu_task()
{
  bmi088.init();

  uint32_t print_div = 0;
  uint32_t last_wake = osKernelGetTickCount();

  for (;;) {
    last_wake += 1;
    osDelayUntil(last_wake);

    bmi088.update();
    imu.update(bmi088.acc, bmi088.gyro);
    g_data.yaw = imu.yaw;

    if (++print_div >= 20) {
      print_div = 0;
      float yaw_deg = imu.yaw * 57.29578f;
      float pitch_deg = imu.pitch * 57.29578f;
      float roll_deg = imu.roll * 57.29578f;
      plotter.plot(yaw_deg, pitch_deg, roll_deg);
    }
  }
}