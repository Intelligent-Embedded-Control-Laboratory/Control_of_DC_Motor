/**
 * @file motor_position_pid.hpp
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
#include "motor.hpp"

class MotorPositionPID
{
public:
    MotorPositionPID()
    {
        memset(this, 0, sizeof(MotorPositionPID));
    }

    void init(const double& _kp, const double& _ki, const double& _kd, const double& _T)
    {
        T = _T;
        ki = _ki;
        kp = _kp;
        kd = _kd;
    }
    void update(const Motor& motor)
    {
        e_pos = (double)(motor.pos - motor.pos_d);
        e_vel = motor.vel;

        u = -kp * e_pos - kd * e_vel + i_term;

        if (abs((int)(u)) > motor_pwm_max)
        {
            u = sign(u) * motor_pwm_max;
            integrator = false;
        }
        else
        {
            integrator = true;
        }

        if (integrator)
        {
            i_func = -ki * e_pos;
            i_term += T / 2.0 * (i_func + i_func_pre);
            i_func_pre = i_func;
        }
    }
    void reset()
    {
        i_func = 0.0;
        i_func_pre = 0.0;
        i_term = 0.0;
    }
    void enable_integrator()
    {
        integrator = true;
    }
    void disable_integrator()
    {
        integrator = false;
    }
    double& get_control() { return u; };
    void set_motor_pwm_max(int pwm_res)
    {
        motor_pwm_max = (int)(pow(2.0, (double)(pwm_res))) - 1;
    }
protected:
    int motor_pwm_max;

    bool integrator;
    double kp, kd, ki;
    double pos_d;
    double e_pos, e_vel;
    double u;
    double i_term, i_func, i_func_pre;
    double T;
    double sign(const double& u_in)
    {
        if (u_in > 0.0)
            return 1.0;
        else if (u_in < 0.0)
            return -1.0;
        return 0.0;
    }
};