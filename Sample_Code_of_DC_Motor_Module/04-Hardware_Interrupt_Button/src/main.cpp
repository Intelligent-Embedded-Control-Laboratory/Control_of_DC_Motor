/*
    Sample Code for using Hardward Interrupts

    Yang-Rui Li
    2021-11-21

    Ref.: http://yhhuang1966.blogspot.com/2016/09/arduino-interrupt.html
*/

#include <Arduino.h>

#define BUTTON_1 9            // 定義外部中斷觸發的腳位
volatile bool LED_Status = 0; // 用於紀錄 LED 狀態的變數

void ISR_Hardware_Button_1();

void setup()
{
    Serial.begin(115200); // 開啟序列埠傳輸，設定 baud rate 為115200

    // 設定外部中斷: BUTTON_1 這個接角在電壓 FALLING 的時候，執行 'ISR_Hardware_Button_1' 這個副程式
    // Teensy and ESP32
    // 中斷腳位就是 GPIO 腳位，所以有沒有加 digitalPinToInterrupt() 都沒關係
    //      attachInterrupt(BUTTON_1, ISR_Hardware_Button_1, FALLING); << 合法
    // 但有些板子只有特定腳位有中斷，並且調用中斷時會有另外一個編號，這個時候就需要用到 digitalPinToInterrupt()
    attachInterrupt(digitalPinToInterrupt(BUTTON_1), ISR_Hardware_Button_1, FALLING);
    // note: detachInterrupt(BUTTON_1) ---> 終止外部中斷的功能

    pinMode(LED_BUILTIN, OUTPUT);
}

void loop()
{
    // Serial.print("LED Status: " + String(LED_Status) + "\n");
    // delay(10);
}

void ISR_Hardware_Button_1()
{
    // 這個副程式，主要用於切換 LED 的狀態，也就是說利用這個函數來達成 LED 的閃爍控制
    LED_Status = !LED_Status;

    digitalWrite(LED_BUILTIN, LED_Status);

    Serial.print("Hello " + String(LED_Status));
}