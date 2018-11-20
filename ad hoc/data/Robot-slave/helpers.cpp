#include <stdint.h>
#include "helpers.h"

long distance[7] = {0,50,100,150,200,250,300};
long RSSIValues[7] = {-54, -56, -60, -58, -63, -66, -70};
long finalSlaveRSSI = RSSI_Value;
long finalMasterRSSI = 0;
uint8_t Mode = 0;

void initialize()
{
    sensorDistance = getDistanceFront();
    if(sensorDistance > 254)
    {
        sensorDistance = 254;
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