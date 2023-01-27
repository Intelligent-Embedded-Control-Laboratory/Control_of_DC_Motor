/*
    Test of DC Motor Module

    Yang-Rui Li
    2021-11-21
*/

#include <Arduino.h>
#include <LS7366R.h> // 讀取 QEI 解編碼器位置
#include <Encoder.h> // 解轉盤的編碼器位置

// 馬達腳位定義
#define PIN_QEI_LEFT 10      // 馬達的 Encoder 腳位
#define PIN_MOTOR_PWM_CW 22  // 馬達正轉的 PWM 腳位
#define PIN_MOTOR_PWM_CCW 23 // 馬達負轉的 PWM 腳位
#define PIN_SWITCH_BUTTON 9  // 轉盤按鈕的接角
#define PIN_WHEEL_ENC_A 14   // 飛梭旋鈕的 A B Phase
#define PIN_WHEEL_ENC_B 15

// 硬體 PWM 設定
#define PWM_WRITE_FREQ 17000 // 馬達 PWM 命令的頻率 (Frequency) (不是命令的更新率!)
#define PWM_WRITE_RES 11     // 馬達 PWM 命令的解析度 (Resolution)

// 內部中斷的時間間隔
#define ISR_TIME 10000

// 變數宣告
LS7366R QEI(PIN_QEI_LEFT, PIN_QEI_LEFT, MDR0_CONF, MDR1_CONF); // 它可以同時解兩個 Encoder，只需要一個腳位
Encoder Wheel(PIN_WHEEL_ENC_A, PIN_WHEEL_ENC_B);
IntervalTimer ISR_Timer;

double T = (double)(ISR_TIME) / 1000000.0;
bool isr_is_trigger = false;
bool system_enable = false;
long int ref_cmd;              // 參考命令
long int pos = 0, pos_pre = 0; // 位置跟前一筆的位置
double vel = 0;                // 速度
int motor_pwm_cmd = 0;
int motor_pwm_cmd_max = (int)(pow(2.0, (double)(PWM_WRITE_RES))) - 1;

// 函數宣告
void isr_main_loop();   // 內部中斷執行函數
void isr_hwit_button(); // 硬體中斷執行函數
void motor_reset();     // 馬達重置
void motor_pwm_update(const int &motor_pwm_command);
template <class T>
T sign(T u_in);

void setup()
{
    // 開啟序列埠傳輸，設定 baud rate 為 115200
    Serial.begin(115200);

    // 設定外部中斷: PIN_SWITCH_BUTTON 這個接角在電壓 FALLING 的時候，執行 'isr_hwit_button()' 這個副程式
    attachInterrupt(digitalPinToInterrupt(PIN_SWITCH_BUTTON), isr_hwit_button, FALLING);

    // 定義腳位的功能
    pinMode(PIN_MOTOR_PWM_CW, OUTPUT);                       // 設定 PIN_MOTOR_PWM_CW 腳位為輸出
    pinMode(PIN_MOTOR_PWM_CCW, OUTPUT);                      // 設定 PIN_MOTOR_PWM_CCW 腳位為輸出
    analogWriteFrequency(PIN_MOTOR_PWM_CW, PWM_WRITE_FREQ);  // 設定 PIN_MOTOR_PWM_CW 腳位 PWM 的輸出頻率
    analogWriteFrequency(PIN_MOTOR_PWM_CCW, PWM_WRITE_FREQ); // 設定 PIN_MOTOR_PWM_CCW 腳位 PWM 的輸出頻率
    analogWriteResolution(PWM_WRITE_RES);                    // 設定所有 PWM 的解析度

    // 將馬達正轉跟負轉的 PWM 腳位設定成0，避免暴衝。
    motor_reset();

    // 清空 QEI 的 Encoder 內存值
    QEI.reset();

    Serial.println("Hello! Here is a Example Project of DC Motor! 2021-11-21");
    delay(1000);

    // 初始化內部中斷函式
    ISR_Timer.begin(isr_main_loop, ISR_TIME);
    ISR_Timer.priority(255);
}

void loop()
{
    if (isr_is_trigger)
    {
        isr_is_trigger = 0;

        // 序列埠傳輸
        Serial.flush();
        Serial.printf("System Flag: %d, ", system_enable);
        Serial.printf("PWM: %d, ", motor_pwm_cmd);
        Serial.printf("Pos: %d (pulse), ", pos);
        Serial.printf("Vel: %.4f (pulse/sec) ", vel);
    }
}

void isr_main_loop()
{
    isr_is_trigger = 1;

    // 參考命令
    ref_cmd = Wheel.read();

    // 讀入馬達的 pulse 數
    QEI.sync();
    pos = QEI.left();

    // 利用後項差分 (backward difference) 進行速度估測
    vel = (double)(pos - pos_pre) / T;
    pos_pre = pos;

    motor_pwm_cmd = motor_pwm_cmd_max * 0.5;
    if (abs(motor_pwm_cmd) >= motor_pwm_cmd_max)
    {
        motor_pwm_cmd = motor_pwm_cmd_max * sign(motor_pwm_cmd);
    }

    // 馬達命令
    if (system_enable) // 只有在 system_enable 為 true 的時候才會做命令的更新
    {
        motor_pwm_update(motor_pwm_cmd);
    }
    else
    {
        motor_pwm_cmd = 0;
        motor_reset();
    }
}

void isr_hwit_button()
{
    system_enable = !system_enable;

    motor_reset();
}

void motor_reset()
{
    motor_pwm_cmd = 0;
    analogWrite(PIN_MOTOR_PWM_CW, 0);
    analogWrite(PIN_MOTOR_PWM_CCW, 0);
}

void motor_pwm_update(const int &motor_pwm_cmd)
{
    // 馬達 PWM 命令更新
    if (motor_pwm_cmd >= 0) // 代表現在要馬達正轉
    {
        analogWrite(PIN_MOTOR_PWM_CW, motor_pwm_cmd);
        analogWrite(PIN_MOTOR_PWM_CCW, 0);
    }
    else // 代表現在要馬達負轉
    {
        analogWrite(PIN_MOTOR_PWM_CW, 0);
        analogWrite(PIN_MOTOR_PWM_CCW, -motor_pwm_cmd);
    }
}

template <class T>
T sign(T u_in)
{
    if (u_in > (T)(0.0))
        return (T)(1.0);
    else if (u_in < (T)(0.0))
        return -(T)(1.0);
    return (T)(0.0);
}