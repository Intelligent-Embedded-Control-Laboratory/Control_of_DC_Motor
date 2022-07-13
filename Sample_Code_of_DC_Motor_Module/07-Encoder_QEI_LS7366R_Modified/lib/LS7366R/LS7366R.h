// Copyright 2014 Werdroid
// Author Vladimir Kosmala

#include <Arduino.h>
#include <SPI.h>

#ifndef LS7366R_h
#define LS7366R_h

#define CLR B00000000
#define RD B01000000
#define WR B10000000
#define LOAD B11000000

#define MDR0 B00001000
#define MDR1 B00010000
#define DTR B00011000
#define CNTR B00100000
#define OTR B00101000
#define STR B00110000

// filter factor 1
// async index
// no index
// free-running
// 4x quadrature
#define MDR0_CONF B00000011

// no flag
// enabled
// 32 bits
#define MDR1_CONF B00000000

class LS7366R
{
public:
    unsigned char leftSelect;
    unsigned char rightSelect;
    long leftValue;
    long rightValue;
    LS7366R(unsigned char _leftSelect, unsigned char _rightSelect, unsigned char mdr0_conf, unsigned char mdr1_conf = MDR1_CONF);
    void reset();
    void sync();
    long left();
    long right();

private:
    long count;
};

class LS7366R_Dev
{
public:
    LS7366R_Dev(unsigned char _pin_slave, unsigned char mdr0_conf = MDR0_CONF, unsigned char mdr1_conf = MDR1_CONF)
    {
        leftSelect = _pin_slave;

        pinMode(leftSelect, OUTPUT);
        SPI.begin();

        digitalWrite(leftSelect, LOW);
        SPI.transfer(WR | MDR0);
        SPI.transfer(mdr0_conf);
        digitalWrite(leftSelect, HIGH);

        digitalWrite(leftSelect, LOW);
        SPI.transfer(WR | MDR1);
        SPI.transfer(mdr1_conf);
        digitalWrite(leftSelect, HIGH);

        reset();
    }

    void reset()
    {
        digitalWrite(leftSelect, LOW);
        SPI.transfer(CLR | CNTR);
        digitalWrite(leftSelect, HIGH);
    }

    void sync()
    {
        digitalWrite(leftSelect, LOW);
        SPI.transfer(LOAD | OTR);
        digitalWrite(leftSelect, HIGH);

        digitalWrite(leftSelect, LOW);
        SPI.transfer(RD | OTR);
        count = SPI.transfer(0x00);
        count <<= 8;
        count |= SPI.transfer(0x00);
        count <<= 8;
        count |= SPI.transfer(0x00);
        count <<= 8;
        count |= SPI.transfer(0x00);
        digitalWrite(leftSelect, HIGH);
        leftValue = count;
    }
    
    long get_pulse()
    {
        return leftValue;
    }

private:
    unsigned char leftSelect;
    long leftValue;
    long count;
};
#endif
