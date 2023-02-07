/**
 * @file motor.hpp
 * @author your name (you@domain.com)
 * @brief
 * @version 0.1
 * @date 2023-02-06
 *
 * @copyright Copyright (c) 2023
 *
 */

#pragma once

#include <Arduino.h>

class Motor
{
public:
    Motor()
    {
        memset(this, 0, sizeof(Motor));
    }
    bool system_enable;
    long int pos;
    long int pos_d;
    double vel; // 速度
    double vel_f;
    int pwm_cmd;
    double T;

    void estimate_velocity_backward()
    {
        vel = (double)(pos - pos_pre) / T;
        pos_pre = pos;
    }

protected:
    long int pos_pre; // 位置跟前一筆的位置
};