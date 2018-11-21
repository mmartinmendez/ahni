#include <stdint.h>
#include <Arduino.h>
#include "helpers.h"

long distance[7] = {0,50,100,150,200,250,300};
int RSSIValuesWithDistance[7];
int RSSISlaveWithDistance[7];
long finalSlaveRSSI = RSSI_Value;
long finalMasterRSSI = 0;
uint8_t Mode = 0;

void initialize()
{
    rssiTime = 200;
    getRSSI();
    tempRssiTime = millis();

    if(Mode == MODE3)
    {
        
    }
}

void setMode(uint8_t mode)
{
    Mode = mode;
}

void calculateDistance()
{
    for(int i=0;i<6;i++)
    {
        if(finalMasterRSSI <= RSSIValuesWithDistance[i] && finalMasterRSSI > RSSIValuesWithDistance[i+1])
        {
            long locationMaster = getLocation(finalMasterRSSI);
            long locationSlave = getLocation(RSSI_Value);
            if((locationSlave - locationMaster) > sensorDistance + 15 && (locationSlave - locationMaster) < sensorDistance - 15)
            {
                getNewRSSI(locationSlave, locationMaster);
                if(finalSlaveRSSI > RSSI_Value)
                {
                    moveForward();
                }
                else 
                {
                    moveBack();
                }
            }
        }
        else 
        {
            finalSlaveRSSI = RSSI_Value;
        }
    }
}

long getLocation(long rssi) 
{
    long location = 0;
    double ratio = 0;
    int index = 0;
    for(int i=0;i<6;i++)
    {
        if(rssi <= RSSIValuesWithDistance[i] && rssi > RSSIValuesWithDistance[i+1])
        {
            // ratio = (double)(RSSI_Value-RSSIValuesWithDistance[i+1])/(double)(RSSIValuesWithDistance[i]-RSSIValuesWithDistance[i+1]);
            index = i;
        }
    }
    // location = (double)(distance[index+1]-distance[index])/ratio + distance[index];
    location = distance[index+1];
    return location;
}

void getNewRSSI(long slave, long master)
{
    long deltaDistance = slave - master - sensorDistance;
    long finalDistance = slave + deltaDistance;
    int index = 0;
    double ratio = 0;
    
    for(int i=0;i<6;i++)
    {
        if(finalDistance >= distance[i] && finalDistance < distance[i+1])
        {
            // ratio = (double)(finalDistance-distance[i+1])/(double)(distance[i+1]-distance[i]);
            index = i;
        }
    }
    // finalSlaveRSSI = (double)(RSSIValuesWithDistance[index]-RSSIValuesWithDistance[index+1])/ratio + RSSIValuesWithDistance[index+1];
    finalSlaveRSSI = RSSIValuesWithDistance[index];
}