/*
    Sample Code of Motor Control
        - Read Encoder Data from QEI

    Yang-Rui Li
    2021-11-21
*/

#include <Arduino.h>
#include <LS7366R.h> // 讀取 QEI 解出來的編碼器位置

#define PIN_QEI 10    // QEI 的腳位
LS7366R_Dev enc(PIN_QEI); // LS7366R_Dev 是基於原作者 LS7366R 所複製的一個類別，專門處理 QEI 只調用一個腳位的情況

// Variable Declaration
long pos = 0;

void setup()
{
    Serial.begin(115200); // 開啟序列埠傳輸，設定 baud rate 為 115200
   
    // SPI.beginTransaction(SPISettings(40000000, MSBFIRST, SPI_MODE0));
    
    // QEI 初始化
    enc.reset();
}

void loop()
{
    // 讀入馬達的 pulse 數
    enc.sync();
    pos = enc.get_pulse();

    // 序列埠傳輸
    Serial.println("Position: " + String(pos) + " (pulse)");

    delay(10); // Delay 10 ms for 100 Hz refresh rate of serial port printer
}