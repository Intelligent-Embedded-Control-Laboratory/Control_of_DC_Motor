/*
    Sample Code of Motor Control
        - Read Encoder Data from Encoder.h

    Yang-Rui Li
    2021-11-21
*/

#include <Arduino.h>
#include <Encoder.h> // 解轉盤的編碼器位置

#define ENC_PHASE_A 14 // Encoder 的 A B Phase
#define ENC_PHASE_B 15
Encoder Motor_Enc(ENC_PHASE_A, ENC_PHASE_B);

// Variable Declaration
long int pos = 0;

void setup()
{
    Serial.begin(115200); // 開啟序列埠傳輸，設定 baud rate 為 115200
}

void loop()
{
    // 讀入馬達的 pulse 數
    pos = Motor_Enc.read();

    // 序列埠傳輸
    Serial.print("Position: " + String(pos) + " (pulse) \n");

    delay(10); // Delay 10 ms for 100 Hz refresh rate of serial port printer
}