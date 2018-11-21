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
uint16_t rssiTime = 500;
uint16_t tempRssiTime = 0;

ISR(TIMER3_OVF_vect) 
{
    // long time = millis();
    // sendRSSI(nodeID, 1, ADHOC);
    // TCNT3 = TIMER3COUNT;

}

void reinitialize()
{
    Mode = MODE1;

    noInterrupts();  
    // TCCR3B &= ~(0 << CS12);
    TCCR3B &= ~(1<< CS12);  // turn off the clock altogether
    TCCR3B &= ~(1<< CS11);
    TCCR3B &= ~(1<< CS10);
    // TIMSK3 &= ~(0 << TOIE1);
    TIMSK3 &= ~(1 << OCIE1A);
    interrupts();
}

void initialize()
{
    moveForward();
    Command = INITIALIZE;
    tempRssiTime = millis();
    rssiTime = 1400;

    tempSendTime = millis();
    sendTime = 500;
}

void initializeAdhocMode() 
{
    //initialize timer 3
    noInterrupts();  
    TCCR3A = 0;
    TCCR3B = 0;

    TCNT3 = TIMER3COUNT;
    TCCR3B |= (1 << CS12);
    TIMSK3 |= (1 << TOIE1);
    interrupts();
}

char *get_mode(uint8_t command, uint8_t *tempData) 
{
    tempData[0] = command;
    tempData[1] = Mode;
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
    uint8_t value[] = {0,0};
    value[0] = 0x31;
    value[1] = RSSI_Value;
    CreatePacket(src, dst, connection, sizeof(value), value);
}

void sendToSlave(uint8_t src, uint8_t dst, uint8_t command)
{
    uint8_t value[2];
    value[0] = 0x12;
    value[1] = 0x26;
    CreatePacket(src, dst, ADHOC, sizeof(value), value);
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