/******************************************************************
 * LIDAR management
 ******************************************************************/

#include "rover.h"
#include "RPLidar.h"

// Singleton instance of the RPLidar driver
RPLidar lidar;

// keep a global buffer representing the most recent scan of the surroundings
// each bin is a ten-degree segment (0-9.99, 10-19.99, etc.)
// if you want to use a different bin / buffer size, keep in mind that the
// code is currently optimized for an even number of bins and an even
// number of degrees in each bin. so good values of NAV_BUFFER_SIZE would
// be 18, 20, 30, 36, 60, 90, 180
nav_scan navBuffer[NAV_BUFFER_SIZE];

void setupLidar() {
    // initialize LIDAR
    debug("initializing LIDAR");
    lidarInit();
    if (!lidarConnect())
        debug("LIDAR connect failed");
    analogWrite(RPLIDAR_MOTOR, LIDAR_MOTOR_SPEED);
}

void initNavBuffer()
{
  //debug("InitNavBuffer()");
  for (unsigned int i = 0; i < NAV_BUFFER_SIZE; i++)
  {
    navBuffer[i].distanceNear = 0;
    navBuffer[i].pointCount = 0;
  }
}

// note: lidarInit only sets up the serial port and motor control.
// it does not actually attempt any communication with the device.
void lidarInit()
{
  debug("lidarInit()");
  lidar.begin(Serial1);
  pinMode(RPLIDAR_MOTOR, OUTPUT);
  analogWrite(RPLIDAR_MOTOR, 0);
  debug("lidarInit() complete");
  //setStatus(STATUS_INITIALIZING);
}

// handshake with the lidar device and start scanning.
// return TRUE if connection was successful, FALSE otherwise.
bool lidarConnect()
{
  debug("lidarConnect()");
  //setStatus(STATUS_LIDAR_CONNECTING);
  // try to detect RPLIDAR
  rplidar_response_device_info_t info;
  if (IS_OK(lidar.getDeviceInfo(info)))
  {
    debug("lidar.getDeviceInfo OK model=" + String(info.model) + " firmware_version=" + String(info.firmware_version) + " hardware_version=" + String(info.hardware_version));
    if (IS_OK(lidar.startScan()))
    {
      debug("lidar.startScan OK, starting motor");
      // start motor rotating
      analogWrite(RPLIDAR_MOTOR, LIDAR_MOTOR_SPEED);
      delay(1000);
      return true;
    }
  }
  debug("lidarConnect() FAILED!");
  return false;
}

// take one (or more) complete scans of the surroundings and update the buffer
// return TRUE if the buffer is updated, FALSE if there was a problem
// and we have to stop to re-connect to the LIDAR.
bool lidarScan()
{
  //unsigned long startTime = millis();
  //debug("lidarScan()");
  //derive a couple of unchanging values, for a bit of efficiency
  static unsigned int binSize = 360 / NAV_BUFFER_SIZE;
  static unsigned int halfABin = binSize / 2;
  initNavBuffer();
  unsigned int pointsCollected = 0;
  unsigned int pointsThrownOut = 0;
  unsigned int startBits = 0;
  //setStatus(STATUS_SCANNING);
  while (true)
  {
    // wait for a data point
    if (IS_OK(lidar.waitPoint()))
    {
      RPLidarMeasurement point = lidar.getCurrentPoint();
      // we want this function to be fast - therefore round off the floating point values first thing
      unsigned int theAngle = round(point.angle);
      unsigned int theDistance = round(point.distance);
      if (point.startBit)
      {
        startBits++;
        if (startBits > OVERSCAN)
        {
          //finish the scan
          //debug("Scan complete; " + String(pointsCollected) + " points collected, " + String(pointsThrownOut) + " thrown out");
          //debug("Scan took " + String(millis() - startTime) + " milliseconds at overscan=" + OVERSCAN);
          return true;
        }
      }
      if ((point.quality >= LIDAR_MINIMUM_QUALITY) && (theDistance > LIDAR_MINIMUM_DISTANCE)) // filter out bad data points
      {
        pointsCollected++;
        // normalize the heading
        // with the current mounting, "forward" is the direction reported by the lidar as
        // 90 degrees. we need that to be stored as 180 degrees (putting that in the middle
        // of the range makes the buffer indexing simpler).
        theAngle += 90;
        if (theAngle >= 360) theAngle -= 360;
        // consolidating the awkwardness here: to make navigation somewhat better, we want
        // "straight ahead" to be in the center of a bin, not on the dividing line between
        // two bins. So navBuffer[0] is going to be directly aft and
        // navBuffer[NAV_BUFFER_SIZE/2] will be directly forward.
        unsigned int bin = (unsigned int)((theAngle + halfABin) / binSize);
        if (bin == NAV_BUFFER_SIZE) bin = 0;  // awkward wrap around conversion
        // save the nearest blip in each bin for obstacle detection
        if ((navBuffer[bin].pointCount == 0) || (point.distance < navBuffer[bin].distanceNear))
          navBuffer[bin].distanceNear = point.distance;
        navBuffer[bin].pointCount++;
      } else pointsThrownOut++;
    } else {
      // here if there's an error; return FALSE so the control loop
      // can stop the vehicle and recover
      //debug("lidarScan() timed out while waiting for data point");
      return false;
    }
  }
}
