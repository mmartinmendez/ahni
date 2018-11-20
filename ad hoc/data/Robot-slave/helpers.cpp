#include <stdint.h>
#include "helpers.h"
#include<Arduino.h>

long distance[6] = {0,50,100,150,200,250};
long RSSIValues[6] = {-54, -56, -60, -58, -63, -66};
long RSSIMaster[6] = {-52, -58, -60, -63, -65, -72};
long RSSIValuesWithDistance[6];
long finalSlaveRSSI = RSSI_Value;
long finalMasterRSSI = 0;
int initialized=0;
uint8_t Mode = 0;

void initialize()
{
    if(Mode == MODE2)
    {
    sensorDistance = getDistanceFront();
    if(sensorDistance > 254)
    {
        sensorDistance = 254;
    }
    }
    if(Mode == MODE3 && initialized==0)
    {
        for(int i=0;i<6;i++)
        {
            moveForward();
            tempInitTime = (uint16_t)millis();
            Command = INITIALIZE;
            getRSSI();
            RSSIValuesWithDistance[i] = RSSI_Value;
            Serial.println(RSSI_Value);
        }
        initialized=1;
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
        if(finalMasterRSSI <= RSSIValues[i] && finalMasterRSSI > RSSIValues[i+1])
        {
            long locationMaster = getLocation(finalMasterRSSI);
            getRSSI();
            long locationSlave = getLocation(RSSI_Value);
            if((locationSlave - locationMaster) > sensorDistance + 5 && (locationSlave - locationMaster) < sensorDistance - 5)
            {
                getNewRSSI(locationSlave, locationMaster);
            }
        }
    }
    if(Mode==MODE3)
    {
        finalMasterRSSI= -70;
        getLocation_mode3(finalMasterRSSI);
        getRSSI();
        finalSlaveRSSI= RSSI_Value;
        long slaveLocation=getLocation(getRSSI);
        long reqSlavelocation = getLocation(reqSlaveRSSI);
        if(slaveLocation<reqSlavelocation)
        {
            RSSIchange=0;
            Command = REQRSSI;
            moveForward();
        }
        else
        {
            RSSIchange=1;
            Command = REQRSSI;
            moveBack();
        }


        //RSSIchange = initMasterRSSI - finalMasterRSSI;
        //reqSlaveRSSI = finalSlaveRSSI - RSSIchange;
        
        
        
        //initMasterRSSI=finalMasterRSSI;
        // for(int i=0;i<6;i++)
        // {
        //     if(reqSlaveRSSI <= RSSIValues[i] && reqSlaveRSSI > RSSIValues[i+1])
        //     {
        //         if(RSSIchange<0)
        //         {
        //           reqSlaveRSSI=RSSIValues[i];
        //           break;
        //         }
        //         else
        //         {
        //           reqSlaveRSSI=RSSIValues[i+1];
        //           break;
        //         }
        //     }
        // }
        // Command = REQRSSI;
        // if(RSSIchange>0)
        // {
        //     moveBack();
        // }
        // else
        // {
        //     moveForward();
        // }


    }
}

long getLocation(long rssi) 
{
    long location = 0;
    double ratio = 0;
    int index = 0;
    for(int i=0;i<6;i++)
    {
        if(rssi <= RSSIValues[i] && rssi > RSSIValues[i+1])
        {
            ratio = (RSSI_Value-RSSIValues[i+1])/(RSSIValues[i]-RSSIValues[i+1]);
            index = i;
        }
    }
    location = (distance[index+1]-distance[index])/ratio + distance[index];
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
            ratio = (finalDistance-distance[i+1])/(distance[i+1]-distance[i]);
            index = i;
        }
    }
    finalSlaveRSSI = (RSSIValues[index]-RSSIValues[index+1])/ratio + RSSIValues[index+1];
}

void getLocation_mode3(long rssi)
{
    double ratio;
    long location=0;
    int index=0;
    for(int i=0;i<6;i++)
    {
        if(rssi <= RSSIMaster[i] && rssi > RSSIMaster[i+1])
        {
            ratio = (finalMasterRSSI-RSSIMaster[i+1])/(RSSIMaster[i]-RSSIMaster[i+1]);
            index = i;
        }
    }
    reqSlaveRSSI = (RSSIValues[index] - RSSIValues[index+1])/ratio + RSSIValues[index+1];
}
