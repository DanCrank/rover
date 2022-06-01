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
