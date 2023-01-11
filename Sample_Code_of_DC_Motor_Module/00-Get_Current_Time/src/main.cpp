#include <Arduino.h>

#define PERIOD_MICROS 1000000

unsigned long t_enter;

void setup()
{
    pinMode(LED_BUILTIN, OUTPUT);
    digitalWrite(LED_BUILTIN, LOW);

    t_enter = micros();
}

void loop()
{
    if (micros() - t_enter >= PERIOD_MICROS)
    {
        t_enter = micros();

        // Do something
        Serial.println(t_enter);
        digitalToggle(LED_BUILTIN);
    }
}