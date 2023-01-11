/*
    Sample Code of Motor Control
        - Read Encoder Data from QEI

    Yang-Rui Li
    2021-11-21
*/

#include <Arduino.h>
#include <LS7366R.h> // 讀取 QEI 解出來的編碼器位置

#define ENC_LEFT 10                                    // 第一個 Encoder 的腳位
#define ENC_RIGHT 10                                   // 第二個 Encoder 的腳位
LS7366R enc(ENC_LEFT, ENC_RIGHT, MDR0_CONF, MDR1_CONF); // 它可以同時解兩個 Encoder，只需要一個腳位

// Variable Declaration
long int pos_1 = 0;
long int pos_2 = 0;

void setup()
{
    Serial.begin(115200); // 開啟序列埠傳輸，設定 baud rate 為 115200

    // QEI 初始化
    enc.reset();
}

void loop()
{
    // 讀入馬達的 pulse 數
    enc.sync();
    pos_1 = enc.left();
    pos_2 = enc.right();

    // 序列埠傳輸
    Serial.print("Position 1: " + String(pos_1) + " (pulse)\t");
    Serial.print("Position 2: " + String(pos_2) + " (pulse) \n");

    delay(10); // Delay 10 ms for 100 Hz refresh rate of serial port printer
}