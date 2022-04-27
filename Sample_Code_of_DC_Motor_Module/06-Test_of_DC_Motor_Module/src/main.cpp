/*
    Test of DC Motor Module

    Yang-Rui Li
    2021-11-21
*/

#include <Arduino.h>
#include <LS7366R.h> // 讀取 QEI 解編碼器位置
#include <Encoder.h> // 解轉盤的編碼器位置

#define ENC_LEFT 10         // 馬達的 Encoder 腳位
#define ENC_RIGHT 10        // 馬達的 Encoder 腳位
#define MOTOR_PWM_CW 22     // 馬達正轉的 PWM 腳位
#define MOTOR_PWM_CCW 23    // 馬達負轉的 PWM 腳位
#define MOTOR_PWM_FREQ 2150 // 馬達 PWM 命令的頻率 (Frequency) (不是命令的更新率!)
#define PWM_WRITE_RES 14    // 馬達 PWM 命令的解析度 (Resolution)
#define SWITCH_BUTTON 9     // 轉盤按鈕的接角
#define WHEEL_A 14          // 飛梭旋鈕的 A B Phase
#define WHEEL_B 15
#define ISR_TIME 10000      // 內部中斷的時間間隔

// Variable Declaration
LS7366R enc(ENC_LEFT, ENC_LEFT, MDR0_CONF, MDR1_CONF); // 它可以同時解兩個 Encoder，只需要一個腳位
Encoder Wheel(WHEEL_A, WHEEL_B);
IntervalTimer ISR_Timer;
double dt = (double)(ISR_TIME) / 1000000.0;
volatile bool ISR_Enable = 0;
volatile bool System_Enable = 0;
long int ref_cmd;              // 參考命令
long int pos = 0, pos_pre = 0; // 位置跟前一筆的位置
double vel = 0;                // 速度
int Motor_PWM_Cmd = 0;
int Motor_PWM_Max = (int)(pow(2.0, (double)(PWM_WRITE_RES))) - 1;

// 函數宣告
void ISR_Control();     // 內部中斷執行函數
void ISR_HWIT_Button(); // 硬體中斷執行函數
void Motor_Reset();     // 馬達重置

template <class InputType>
InputType sign(InputType u_in);

void setup()
{
    // 設定外部中斷: SWITCH_BUTTON 這個接角在電壓 FALLING 的時候，執行 'ISR_HWIT_Button' 這個副程式
    attachInterrupt(digitalPinToInterrupt(SWITCH_BUTTON), ISR_HWIT_Button, FALLING);

    // 開啟序列埠傳輸，設定 baud rate 為 115200 
    Serial.begin(115200); 
    delay(1000);

    // 定義腳位的功能
    pinMode(MOTOR_PWM_CW, OUTPUT);                       // 設定 MOTOR_PWM_CW 腳位為輸出
    pinMode(MOTOR_PWM_CCW, OUTPUT);                      // 設定 MOTOR_PWM_CCW 腳位為輸出
    analogWriteFrequency(MOTOR_PWM_CW, MOTOR_PWM_FREQ);  // 設定 MOTOR_PWM_CW 腳位 PWM 的輸出頻率
    analogWriteFrequency(MOTOR_PWM_CCW, MOTOR_PWM_FREQ); // 設定 MOTOR_PWM_CCW 腳位 PWM 的輸出頻率
    analogWriteResolution(PWM_WRITE_RES);                // 設定所有 PWM 的解析度

    // 將馬達正轉跟負轉的 PWM 腳位設定成0，避免暴衝。
    Motor_Reset();

    // 清空 QEI 的 Encoder 內存值
    enc.reset();

    delay(1000);

    ISR_Timer.begin(ISR_Control, ISR_TIME);
    ISR_Timer.priority(255);
}

void loop()
{
    if (ISR_Enable)
    {
        // 序列埠傳輸
        Serial.flush();
        Serial.print("System Enable Flag: " + String(System_Enable) + "\t");
        Serial.print("Motor PWM Command: " + String(Motor_PWM_Cmd) + "\t");
        Serial.print("Position: " + String(pos) + " (pulse)\t");
        Serial.print("Velocity: " + String(vel) + " (pulse/sec)\n");
    }
}

void ISR_Control()
{
    ISR_Enable = 1;

    // 參考命令
    ref_cmd = Wheel.read();

    // 讀入馬達的 pulse 數
    enc.sync();
    pos = enc.left();

    // 利用後項差分 (Backward Difference) 進行速度估測
    vel = (double)(pos - pos_pre) / dt;
    pos_pre = pos;

    // 馬達命令
    if (System_Enable) // 只有在 System_Enable 為 1 的時候才會做命令的更新
    {
        Motor_PWM_Cmd = Motor_PWM_Max * 0.5;
    }
    else
    {
        Motor_PWM_Cmd = 0;
    }

    // 馬達 PWM 命令更新
    if (Motor_PWM_Cmd >= 0) // 代表現在要馬達正轉
    {
        analogWrite(MOTOR_PWM_CW, Motor_PWM_Cmd);
        analogWrite(MOTOR_PWM_CCW, 0);
    }
    else // 代表現在要馬達負轉
    {
        analogWrite(MOTOR_PWM_CW, 0);
        analogWrite(MOTOR_PWM_CCW, -Motor_PWM_Cmd);
    }
}

void ISR_HWIT_Button()
{
    System_Enable = !System_Enable;

    Motor_Reset();
}

void Motor_Reset()
{
    Motor_PWM_Cmd = 0;
    analogWrite(MOTOR_PWM_CW, 0);
    analogWrite(MOTOR_PWM_CCW, 0);
}

template <class InputType>
InputType sign(InputType u_in)
{
    if (u_in > (InputType)(0.0))
        return (InputType)(1.0);
    else if (u_in < (InputType)(0.0))
        return -(InputType)(1.0);
    return (InputType)(0.0);
}