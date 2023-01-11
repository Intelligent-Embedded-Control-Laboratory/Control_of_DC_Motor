/*
    Sample Code of Motor Control
        - Read Encoder Data from QEI
        - Drive Motor by PWM Command

    Yang-Rui Li
    2021-11-21
*/

#include <Arduino.h>
#include <LS7366R.h> // 讀取 QEI 解出來的編碼器位置

#define ENC_LEFT 10                                    // 第一個 Encoder 的腳位
#define ENC_RIGHT 10                                   // 第二個 Encoder 的腳位
LS7366R enc(ENC_LEFT, ENC_LEFT, MDR0_CONF, MDR1_CONF); // 它可以同時解兩個 Encoder，只需要一個腳位

#define PIN_MOTOR_PWM_CW 22     // 馬達正轉的 PWM 腳位
#define PIN_MOTOR_PWM_CCW 23    // 馬達負轉的 PWM 腳位
#define PWM_WRITE_FREQ 2150 // 馬達PWM命令的更新頻率 (Frequency)
#define PWM_WRITE_RES 14    // 馬達PWM寫入的解析度 (Resolution)
// Ref: https://www.pjrc.com/teensy/td_pulse.html

// Variable Declaration
long int pos_1 = 0;
long int pos_2 = 0;

int Motor_PWM_Cmd = 0;
int Motor_PWM_Max = (int)(pow(2.0, (double)(PWM_WRITE_RES))) - 1;


void setup()
{
    Serial.begin(115200); // 開啟序列埠傳輸，設定baud rate為115200

    // 定義腳位的功能
    pinMode(PIN_MOTOR_PWM_CW, OUTPUT);                       // 設定 MOTOR_PWM_CW 腳位為輸出
    pinMode(PIN_MOTOR_PWM_CCW, OUTPUT);                      // 設定 MOTOR_PWM_CCW 腳位為輸出
    analogWriteFrequency(PIN_MOTOR_PWM_CW, PWM_WRITE_FREQ);  // 設定 PWM 的輸出頻率
    analogWriteFrequency(PIN_MOTOR_PWM_CCW, PWM_WRITE_FREQ); // 設定 PWM 的輸出頻率
    analogWriteResolution(PWM_WRITE_RES);                // 設定所有 PWM 的解析度

    // 將馬達正轉跟負轉的 PWM 腳位設定成0，避免暴衝。
    analogWrite(PIN_MOTOR_PWM_CW, 0);
    analogWrite(PIN_MOTOR_PWM_CCW, 0);

    // QEI 初始化
    enc.reset();
}

void loop()
{
    // 讀入馬達的 pulse 數
    enc.sync();
    pos_1 = enc.left();
    pos_2 = enc.right();

    // 馬達命令
    Motor_PWM_Cmd = -Motor_PWM_Max * 0.5;

    // 馬達 PWM 命令更新
    if (Motor_PWM_Cmd >= 0) // 代表現在要馬達正轉
    {
        analogWrite(PIN_MOTOR_PWM_CW, Motor_PWM_Cmd);
        analogWrite(PIN_MOTOR_PWM_CCW, 0);
    }
    else // 代表現在要馬達負轉
    {
        analogWrite(PIN_MOTOR_PWM_CW, 0);
        analogWrite(PIN_MOTOR_PWM_CCW, -Motor_PWM_Cmd);
    }

    // 序列埠傳輸
    Serial.print("Motor PWM Command: " + String(Motor_PWM_Cmd) + "\t");
    Serial.print("Position 1: " + String(pos_1) + " (pulse)\t");
    Serial.print("Position 2: " + String(pos_2) + " (pulse) \n");

    delay(10); // Delay 10 ms for 100 Hz refresh rate of serial port printer
}