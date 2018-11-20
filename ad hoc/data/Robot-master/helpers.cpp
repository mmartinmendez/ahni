#include <stdint.h>
#include <Arduino.h>
#include "prototype.h"
#include "variables.h"

uint8_t Mode = 0;
long RSSIValuesWithDistance[7];
uint8_t temp = 0x03;
int initialized;
uint16_t tempInitTime = 0;
uint16_t initTime = 1400;

void reinitialize()
{
    Mode = MODE1;

    noInterrupts();  
    TCCR3B &= ~(0 << CS12);
    TIMSK3 &= ~(0 << TOIE1);
    interrupts();
}

void initializeAdhocMode() 
{
    //initialize timer 3
    // noInterrupts();  
    // TCCR3A = 0;
    // TCCR3B = 0;

    // TCNT3 = TIMER3COUNT;
    // TCCR3B |= (1 << CS12);
    // TIMSK3 |= (1 << TOIE1);
    // interrupts();

    // if(!initialized) {
        getArray();
    // }
}

void getArray() 
{
    for(int i=0;i<4;i++)
    {
        moveForward();
        tempInitTime = (uint16_t)millis();
        Command = INITIALIZE;
        getRSSI();
        RSSIValuesWithDistance[i] = RSSI_Value;
        Serial.println(RSSI_Value);
    }
    initialized = 1;
}

char *get_mode(uint8_t command, uint8_t *tempData) 
{
    uint8_t botStatus = Mode;
    tempData[0] = command;
    tempData[1] = botStatus;
    return tempData;
}

void set_mode(uint8_t newMode) 
{
    Mode = newMode;
}

// RSSI values sent to the slave
void sendRSSI(uint8_t src, uint8_t dst, uint8_t connection) 
{
    getRSSI();
    uint8_t value = RSSI_Value;
    CreatePacket(src, dst, connection, sizeof(value), &value);
}

ISR(TIMER3_OVF_vect) 
{
    long time = millis();
    sendRSSI(nodeID, 1, ADHOC);
    TCNT3 = TIMER3COUNT;
}

// Get the current data from the bot 
char *get_data(uint8_t src, uint8_t dst, uint8_t *tempData) 
{
    uint8_t botStatus = Mode;
    uint8_t value = (RSSIValuesWithDistance[0] << 24) | (RSSIValuesWithDistance[1] << 16) | (RSSIValuesWithDistance[2] << 8) | RSSIValuesWithDistance[3];
    tempData[0] = Command;
    tempData[1] = RSSIValuesWithDistance[0];
    tempData[9] = RSSIValuesWithDistance[1];
    tempData[18] = RSSIValuesWithDistance[2];
    tempData[27] = RSSIValuesWithDistance[3];
    Serial.println(tempData[1]);
    Serial.println(tempData[2]);
    Serial.println(tempData[3]);
    Serial.println(tempData[4]);

    CreatePacket(src, dst, TCP, sizeof(value), value);
    return tempData;
}