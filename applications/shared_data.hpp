#ifndef SHARED_DATA_HPP
#define SHARED_DATA_HPP

#include <cstdint>

struct SharedData
{
    // IMU 数据
    float yaw; // 单位：rad

    // 遥控器拨杆
    uint8_t sw_r; // 右拨杆：0=DOWN, 1=MID, 2=UP
    uint8_t sw_l; // 左拨杆：0=DOWN, 1=MID, 2=UP

    // 电机反馈
    float angle_a; // A电机角度，单位：rad
    float angle_b; // B电机角度，单位：rad
};

extern SharedData g_data;

#endif // SHARED_DATA_HPP