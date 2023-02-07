/**
 * @file main.cpp
 * @author your name (you@domain.com)
 * @brief Sample Project for Control of DC Motor
 * @version 0.1
 * @date 2021-11-21
 *
 * @copyright Copyright (c) 2023
 *
 */

#include <Arduino.h>
#include <LS7366R.h> // 讀取 QEI 解編碼器位置
#include <Encoder.h> // 解轉盤的編碼器位置
#include <motor.hpp>
#include <motor_position_pid.hpp>
#include <moving_average_filter.h>
#include <first_order_iir_filters.h>

#define PIN_QEI_LEFT 10       // 馬達的 Encoder 腳位
#define PIN_MOTOR_PWM_CW 22   // 馬達正轉的 PWM 腳位
#define PIN_MOTOR_PWM_CCW 23  // 馬達負轉的 PWM 腳位
#define PIN_VR A7 // 輸入
 // #define PIN_SWITCH_BUTTON 9   // 轉盤按鈕的接角
 // #define PIN_SWITCH_PHASE_A 14 // 飛梭旋鈕的 A B Phase
 // #define PIN_SWITCH_PHASE_B 15

#define CONFIG_ANALOG_READ_RES 10 // 解析度是 10，代表值的範圍是 0 ~ 1023 (2^10)，0 對應到 0 V，1023 對應 3.3 V。
#define CONFIG_MOTOR_PWM_WRITE_FREQ 17000 // 馬達 PWM 命令的頻率 (Frequency) (不是命令的更新率!)
#define CONFIG_MOTOR_PWM_WRITE_RES 11     // 馬達 PWM 命令的解析度 (Resolution)
#define CONFIG_ISR_TIME_MICROS 1000       // 內部中斷的時間間隔

#define PARAM_POS_PID_KP 1322.0 / 60000.0 * 2.0 * PI * 2.5
#define PARAM_POS_PID_KI 9240.0 / 60000.0 * 2.0 * PI * 2.5
#define PARAM_POS_PID_KD 63.0 / 60000.0 * 2.0 * PI * 2.5
#define PARAM_FILTER_WINDOW_SIZE_VEL 100
#define PARAM_FILTER_CUTOFF_HZ_CMD 1

 // Variable Declaration
LS7366R qei(PIN_QEI_LEFT, PIN_QEI_LEFT, MDR0_CONF, MDR1_CONF); // 它可以同時解兩個 Encoder，只需要一個腳位
// Encoder wheel(PIN_SWITCH_PHASE_A, PIN_SWITCH_PHASE_B);

IntervalTimer isr_timer;
double T = (double)(CONFIG_ISR_TIME_MICROS) / 1000000.0;

Motor motor;
MotorPositionPID pos_pid;
MovingAverageFilter  filter_vel;
FirstOrderIIR filter_cmd;

unsigned long t_begin, t_duration;

// 函數宣告
void isr_control();     // 內部中斷執行函數
void isr_hwit_button(); // 硬體中斷執行函數
void motor_reset();     // 馬達重置

void setup()
{
    // 開啟序列埠傳輸，設定 baud rate 為 115200
    Serial.begin(115200);
    delay(1000);

    // 設定外部中斷: PIN_SWITCH_BUTTON 這個接角在電壓 FALLING 的時候，執行 'isr_hwit_button' 這個副程式
    // attachInterrupt(digitalPinToInterrupt(PIN_SWITCH_BUTTON), isr_hwit_button, FALLING);

    // 定義腳位的功能
    pinMode(PIN_MOTOR_PWM_CW, OUTPUT);                                    // 設定 PIN_MOTOR_PWM_CW 腳位為輸出
    pinMode(PIN_MOTOR_PWM_CCW, OUTPUT);                                   // 設定 PIN_MOTOR_PWM_CCW 腳位為輸出
    analogWriteFrequency(PIN_MOTOR_PWM_CW, CONFIG_MOTOR_PWM_WRITE_FREQ);  // 設定 PIN_MOTOR_PWM_CW 腳位 PWM 的輸出頻率
    analogWriteFrequency(PIN_MOTOR_PWM_CCW, CONFIG_MOTOR_PWM_WRITE_FREQ); // 設定 PIN_MOTOR_PWM_CCW 腳位 PWM 的輸出頻率
    analogWriteResolution(CONFIG_MOTOR_PWM_WRITE_RES);                    // 設定所有 PWM 的解析度

    pinMode(PIN_VR, INPUT);
    analogReadResolution(CONFIG_ANALOG_READ_RES);

    // 將馬達正轉跟負轉的 PWM 腳位設定成0，避免暴衝。
    motor_reset();

    // 清空 QEI 的 Encoder 內存值
    qei.reset();

    // Initialization
    motor.T = T;
    pos_pid.init(PARAM_POS_PID_KP, PARAM_POS_PID_KI, PARAM_POS_PID_KD, T);
    pos_pid.set_motor_pwm_max(CONFIG_MOTOR_PWM_WRITE_RES);

    filter_cmd.init(FirstOrderIIR::TYPE_LPF, PARAM_FILTER_CUTOFF_HZ_CMD, T);
    filter_vel.init(PARAM_FILTER_WINDOW_SIZE_VEL);

    // 初始化時間中斷函式
    isr_timer.begin(isr_control, CONFIG_ISR_TIME_MICROS);
    isr_timer.priority(255);
}

void loop()
{
}

void isr_control()
{
    t_begin = micros();

    // 參考命令
    double command = map((double)(analogRead(PIN_VR)), 0, 1023, -60000, 60000);
    double command_f = 0.0;
    filter_cmd.update(&command, &command_f);
    
    motor.pos_d = (double)(command_f);

    // 讀入馬達的 pulse 數
    qei.sync();
    motor.pos = qei.left();

    // 利用後項差分 (Backward Difference) 進行速度估測
    motor.estimate_velocity_backward();

    // Velocity Filtering
    double vel = (double)(motor.vel);
    motor.vel_f = filter_vel.update(vel);

    // Update PID
    pos_pid.update(motor);

    // 馬達命令
    motor.system_enable = true;
    if (motor.system_enable) // 只有在 system_enable 為 true 的時候才會做命令的更新
    {
        pos_pid.enable_integrator();

        motor.pwm_cmd = (int)(pos_pid.get_control());
    }
    else
    {
        pos_pid.disable_integrator();
        motor.pwm_cmd = 0;
    }
    motor.pwm_cmd = 1023;

    // 馬達 PWM 命令更新
    if (motor.pwm_cmd >= 0) // 代表現在要馬達正轉
    {
        analogWrite(PIN_MOTOR_PWM_CW, motor.pwm_cmd);
        analogWrite(PIN_MOTOR_PWM_CCW, 0);
    }
    else // 代表現在要馬達負轉
    {
        analogWrite(PIN_MOTOR_PWM_CW, 0);
        analogWrite(PIN_MOTOR_PWM_CCW, -motor.pwm_cmd);
    }

    // Print data
    Serial.print(command);
    Serial.print(" ");
    Serial.print(motor.pos_d);
    Serial.print(" ");
    Serial.print(vel);
    Serial.print(" ");
    Serial.print(motor.vel_f);
    Serial.print(" ");
    Serial.println();
    Serial.flush();

    t_duration = micros() - t_begin;
}

void isr_hwit_button()
{
    motor.system_enable = !motor.system_enable;
    motor_reset();
}

void motor_reset()
{
    pos_pid.reset();

    analogWrite(PIN_MOTOR_PWM_CW, 0);
    analogWrite(PIN_MOTOR_PWM_CCW, 0);
}