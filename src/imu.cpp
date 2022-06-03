// IMU management

#include "rover.h"
#include <Adafruit_ISM330DHCX.h>
#include <Adafruit_LIS3MDL.h>

Adafruit_ISM330DHCX imu;
Adafruit_LIS3MDL compass;

void setupIMU() {
    if (!imu.begin_I2C()){
        debug("***Failed to connect to IMU***");
    }
    if (!compass.begin_I2C()){
        debug("***Failed to connect to compass***");
    }
    imu.setAccelRange(LSM6DS_ACCEL_RANGE_2_G);
    imu.setAccelDataRate(LSM6DS_RATE_12_5_HZ);
    imu.setGyroRange(LSM6DS_GYRO_RANGE_125_DPS);
    imu.setGyroDataRate(LSM6DS_RATE_12_5_HZ);
    compass.setDataRate(LIS3MDL_DATARATE_300_HZ);
    compass.setRange(LIS3MDL_RANGE_4_GAUSS);
    compass.setPerformanceMode(LIS3MDL_HIGHMODE);
    compass.setOperationMode(LIS3MDL_CONTINUOUSMODE); // try SINGLEMODE?
    // TODO: enable compass' temperature sensor (off by default)
    compass.setIntThreshold(500);
    compass.configInterrupt(false, false, false, // disable interrupts
                            true,  // polarity
                            false, // don't latch
                            true); // enabled!
}

// adding getters for specific measurements that are needed - more can be added

uint16_t getMagneticHeading() {
    sensors_event_t event;
    compass.getEvent(&event);
    //char debugstr[100];
    // convert x/y to angle
    // current device mounting is +x is forward (heading 0) and +y is vehicle left (270 degrees)
    // so for trig purposes, x and y are purposefully swapped below
    // +2.0 x offset determined experimentally
    // https://electronics.stackexchange.com/questions/314957/how-to-use-lis3mdl
    float x = (event.magnetic.y * -1.0) + COMPASS_X_OFFSET;
    float y = event.magnetic.x + COMPASS_Y_OFFSET;
    // calculate the arctangent
    float radians = atan(x / y);
    // normalize quadrant
    if (y < 0) {
        radians = (radians + PI);
    }
    // THAT number is the direction of the magnetic field relative to the vehicle...we want the
    // vehicle's heading relative to magnetic north
    radians = (2 * PI) - radians;
    // convert to degrees and normalize
    //sprintf(debugstr, "x = %.2f  y = %.2f  radians = %.2f", x, y, radians);
    //debug(debugstr);
    return ((((int16_t)(radians * 57.29578)) + 360) % 360);
}

float getYawRate() {
    sensors_event_t accel, gyro, temp;
    imu.getEvent(&accel, &gyro, &temp);
    return gyro.gyro.heading;
}