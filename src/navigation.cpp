/******************************************************************
 * navigation code
 ******************************************************************/

#include "rover.h"

extern nav_scan navBuffer[];
extern StatusDisplay *statusDisplay;

// return the bin index of the nearest obstruction, or -1 if no obstructions are visible
// inside the NAV_AVOID_CONE and within the specified distance.
// the sector parameter is the index of the bin straight ahead of the rover (defaults to NAV_BUFFER_SIZE/2)
// the distance parameter is the maximum distance for a point to be considered (defaults to NAV_AVOID_DISTANCE)
int navFindObstruction(int16_t sector, uint16_t distance)
{
    //debug("navFindObstruction()");
    int16_t bin = -1;
    uint16_t range = distance + 1;
    for (int16_t i = sector - NAV_AVOID_CONE; i <= sector + NAV_AVOID_CONE; i++)
    {
        int16_t realIndex = (i + NAV_BUFFER_SIZE) % NAV_BUFFER_SIZE;  // handle indexes that span buffer boundaries
        if (navBuffer[realIndex].pointCount == 0)
        {
            //debug("I have no data for sector " + String(i) + ", declaring that sector obstructed.");
            //bin = realIndex;
            //range = 0;
        } else if ((navBuffer[realIndex].distanceNear < distance) &&
                   (navBuffer[realIndex].distanceNear < range)) {
            bin = realIndex;
            range = navBuffer[realIndex].distanceNear;
        }
    }
    // if (bin == -1)
    //   debug("No obstruction detected");
    // else
    //   debug("Nearest obstruction detected in sector " + String(bin) + ", range=" + String(range) + "mm");
    statusDisplay->setObstruction(bin);
    return bin;
}

// turn the vehicle the specified number of degrees (positive right, negative left)
// by integrating the yaw rate read from the IMU's gyroscope.
// turns of less than 10 degrees are ignored for now - we're not that precise.
// this function does not return until the turn is complete.
void navTurn(int16_t degrees) {
    driveStop();
    // IMU gyro reads in radians, so convert to radians
    float radians = (abs((float)degrees) / 360.0) * (2.0 * PI);
    float radiansTurned = 0.0;
    uint16_t integrationTime;
    // TODO: be REALLY clever, and start slow then increase motor power until
    // it reaches a set yaw rate, so it will work accurately on any surface
    if (degrees < -10) {
        driveLeft(NAV_TURN_SPEED);
    } else if (degrees > 10) {
        driveRight(NAV_TURN_SPEED);
    } else return;
    // TODO: is there a nanosecond clock available? that would probably help greatly here
    integrationTime = millis();
    while (radiansTurned < radians) {
        delay(1); // not sure this delay is even necessary?
        uint16_t now = millis();
        radiansTurned += abs(getYawRate()) * (float)(now - integrationTime) / 1000.0;
        integrationTime = now;
    }
    driveStop();
}
