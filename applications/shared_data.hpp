#ifndef SHARED_DATA_HPP
#define SHARED_DATA_HPP

#include <cstdint>

struct SharedData
{
  float yaw;
  uint8_t sw_r;
  uint8_t sw_l;
  float angle_a;
  float angle_b;
};

extern SharedData g_data;

#endif  // SHARED_DATA_HPP