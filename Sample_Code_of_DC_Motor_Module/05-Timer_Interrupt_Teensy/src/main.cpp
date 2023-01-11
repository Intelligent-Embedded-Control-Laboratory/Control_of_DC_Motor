/*
    Sample Code for declaring ISR of Teensy

    Yang-Rui, Li
    2021-11-21
*/

#include <Arduino.h>

#define ISR_TIME 1000000  // 定義多少時間執行一次內部中斷的函數，單位是 micro second.
IntervalTimer ISR_Timer; // 宣告內部中斷的計數器
// 註：ISR = Interrupt Service Routine 中斷服務常式
// 一般來說，只要是中斷觸發 (內部 or 外部) 所執行的事情，都可以叫作 ISR；但是在我們實驗室，ISR 一般是指「內部中斷」觸發時所執行的函數。

volatile bool ISR_is_trigger = 0;
int idx = 0; // 內部中斷的計數器

double t = 0.0;                             // 內部中斷開始後的絕對時間，第一次執行內部中斷函數的時間為 0.0 秒
double dt = (double)(ISR_TIME) / 1000000.0; // 內部中斷的時間間隔

unsigned long int time_diff = 0; // // 用於計算兩次執行 ISR_Control() 所經過的時間 (即內部中斷時間) << 僅僅是為了確認 MCU 是否按照預期行為運作
unsigned long int time_diff_pre = 0;
unsigned long t_spend;

// 函數宣告：
void ISR_Control(); // 宣告內部中斷時，所執行的函數。 (Interrupt Service Routine, ISR)

// 主程式：
void setup()
{
    Serial.begin(115200); // 開啟序列埠傳輸，設定baud rate為115200

    delay(1000);

    ISR_Timer.begin(ISR_Control, ISR_TIME); // 內部中斷設定：經過 ISR_TIME 的時間，執行 ISR_Control 這個程式
    ISR_Timer.priority(255);                // 設定內部中斷的優先權，由高到低分別是 0~255，預設是 128。
}

void loop()
{
    if (ISR_is_trigger) // 當執行內部中斷之後，需要做的事情 (可能比較不重要)
    {
        ISR_is_trigger = 0;

        t_spend = micros();
        Serial.print("Time Difference of ISR: " + String(time_diff) + "\t");
        Serial.print("Time: " + String(t, 3) + "\n");
        t_spend = micros() - t_spend; // 夾程式執行時間
        
        Serial.println("Time Spend: " + String(t_spend));
        Serial.flush(); // 哨兵
    }
}

void ISR_Control()
{
    time_diff = micros() - time_diff_pre; // 用於計算兩次執行 ISR_Control() 所經過的時間 (即內部中斷時間)
    time_diff_pre = micros();

    ISR_is_trigger = 1;

    t = (double)(idx)*dt; // 從內部中斷開始後的絕對時間

    idx++; // 內部中斷的計數器
}