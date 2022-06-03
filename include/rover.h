/******************************************************************
 * Prototype rover code
 *
 * Pin assignments:
 * LIDAR: will use Serial1 (0 / 1), motor control on A0 (14)
 * Radio: uses SPI - MOSI (24), MISO (23), SCK (25), jumper CS to 11 (labeled A),
 *          IRQ to 19 (labeled F), RST to 5 (labeled E) (so display pushbutton
 *          can reset the radio?)
 * Display: uses I2C (21 / 22), addr 0x3C
 * Motor driver: uses I2C (addr 0x60)
 * IMU: uses I2C, addr 0x1C for the compass, 0x6A for the gyro/accelerometer
 * GPS: uses Serial2 defined on SERCOM3 using pins 12 / 13
 * Adalogger: RTC uses I2C (addr 0x68), SD uses SPI with CS on 10
 * Current sensor: uses I2C (addr 0x40)
 *
 ******************************************************************/

#ifndef ROVER_h
#define ROVER_h

#include <string>
#include <vector>
#include <atomic>
#include <Arduino.h>

/******************************************************************
 * Cron
 ******************************************************************/
struct CronJob {
    void     (*job)();
    uint32_t interval;
    uint32_t lastRun;
};
void registerCronJob(CronJob *cronJob);
void checkCronJobs();

/******************************************************************
 * Motor driver
 ******************************************************************/
void setupDrive();
void driveForward(uint8_t speed);
void driveReverse(uint8_t speed);
void driveStop();
void driveLeft(uint8_t speed);
void driveRight(uint8_t speed);

/******************************************************************
 * IMU
 ******************************************************************/
void setupIMU();
uint16_t getMagneticHeading();
float getYawRate();

#define COMPASS_X_OFFSET -7.5
#define COMPASS_Y_OFFSET 0

/******************************************************************
 * GPS
 ******************************************************************/
#define READGPS_INTERVAL_MS 1

void setupGPS();
void checkForNewGPSData();
void readGPSData(void);

/******************************************************************
 * Display
 ******************************************************************/
void setupDisplay();

#define DISPLAY_ROWS 8
#define DISPLAY_COLS 9 // each line is null terminated

class StatusDisplay {
private:
    std::atomic<bool> lidarOK;
    std::atomic<uint16_t> gpsSats;
    std::atomic<int16_t> rssi;
    std::atomic<int16_t> obstruction;
    std::atomic<int16_t> magHeading;
    std::atomic<bool> defaultDisplay;
    char displayBuf[DISPLAY_ROWS][DISPLAY_COLS];
    void printToDisplay(const char * str, uint16_t x, uint16_t y);

public:
    StatusDisplay();
    void setLidarOK(bool newLidarOK);
    void setGpsSats(uint16_t newGpsSats);
    void setRssi(int16_t newRssi);
    void setObstruction(int16_t newObstruction);
    void setMagHeading(int16_t newMagHeading);
    void display(const std::string& str);
    void returnToDefaultDisplay();
    void updateDisplay();
};

/******************************************************************
 * Navigation
 ******************************************************************/
// NAV_BUFFER_SIZE is the number of bins that the LIDAR data will
// be sorted into. The 360 degree space around the rover is divided
// into NAV_BUFFER_SIZE "sectors" with sector (NAV_BUFFER_SIZE / 2)
// being *centered* on the heading directly ahead of the rover.
#define NAV_BUFFER_SIZE 60
// NAV_AVOID_DISTANCE is the maximum distance from the rover (in
// millimeters) that a LIDAR point can be to be considered an
// obstruction.
#define NAV_AVOID_DISTANCE 500
// NAV_AVOID_CONE is the maximum angular distance from the front
// of the rover (in number of sectors) that a LIDAR point can be considered
// an obstruction.
#define NAV_AVOID_CONE 15
// NAV_TURN_SPEED is the speed to use for turning. this will eventually
// go away in favor of a smarter turning function - see note in navTurn().
// 96 seems like the sweet spot - too slow and it gets bogged down in carpet,
// but too fast and it starts to get really inaccurate.
#define NAV_TURN_SPEED 96

struct nav_scan
{
  uint16_t distanceNear;
  uint16_t pointCount;
};

int navFindObstruction(int16_t sector = (NAV_BUFFER_SIZE / 2), uint16_t distance = NAV_AVOID_DISTANCE);
void navTurn(int16_t degrees);

/******************************************************************
 * LIDAR
 ******************************************************************/
#define RPLIDAR_MOTOR 16

// setting to collect more than one complete LIDAR rotation in a "scan".
// this must be at least 1. values higher than 1 may help the vehicle make
// better maneuvering decisions, but interfere with the actual maneuvering
// because the time spent scanning extends the length of the decision-making
// cycle.
#define OVERSCAN 1

// LIDAR motor speed (0-255)
#define LIDAR_MOTOR_SPEED 255

// minimum distance, inside which LIDAR data points are thrown out. this is
// used to filter out spurious data points at unreasonably close distances
// (inside the perimeter of the vehicle or thereabouts). value in mm.
// the datasheet lists minimum measurable distance as 150mm, so that's a
// good initial value.
#define LIDAR_MINIMUM_DISTANCE 100

// minimum quality, beneath which LIDAR data points are thrown out. this is
// a score assigned by the device, and its meaning is not documented (it is
// an unsigned byte coming from the device). in practice, most data points
// are observed to be quality = 15.
#define LIDAR_MINIMUM_QUALITY 4

void setupLidar();
void lidarInit();
bool lidarConnect();
bool lidarScan();

/******************************************************************
 * RFM69 (radio)
 ******************************************************************/
void setupRfm69();
void updateSignalStrength();

/******************************************************************
 * Messaging
 ******************************************************************/
// message timing parameters
#define ACK_TIMEOUT 1000 // millis to wait for an ack msg
#define MSG_DELAY 100 // millis to wait between Rx and Tx, to give the other side time to switch from Tx to Rx
#define LISTEN_DELAY 50 // millis to wait between checks of the receive buffer when receiving
#define TELEMETRY_INTERVAL 5000 // millis to wait after a successful telemetry exchange before sending another
#define DISPLAY_UPDATE_INTERVAL 250

// message IDs
#define MESSAGE_TELEMETRY 0
#define MESSAGE_TELEMETRY_ACK 1
#define MESSAGE_COMMAND_READY 2
#define MESSAGE_COMMAND 3
#define MESSAGE_COMMAND_ACK 4

void dump_buffer(std::vector<uint8_t> *v);

void sendTelemetry();

// RadioHead enforces the first 4 bytes of the payload of every message as TO, FROM, ID, FLAGS
// that means the max payload of a message is 60 bytes
class RoverTimestamp {
public:
    uint8_t year; // two-digit year. sue me on 1/1/3000
    uint8_t month;
    uint8_t day;
    uint8_t hour;
    uint8_t minute;
    uint8_t second;

    RoverTimestamp();
    // deserializing constructor
    RoverTimestamp(std::vector<uint8_t> *v, size_t i);
    // serialize a timestamp onto the given vector
    void serialize(std::vector<uint8_t> *v) const;
};

class RoverLocData {
public:
    float gps_lat;
    float gps_long;
    float gps_alt;
    float gps_speed;
    uint8_t gps_sats;
    uint16_t gps_hdg;

    RoverLocData();
    // serialize loc data onto the given vector
    void serialize(std::vector<uint8_t> *v);
};

class TelemetryMessage {
public:
    RoverTimestamp *timestamp;   // bytes 2-7
    RoverLocData *location;      // bytes 8-25
    int16_t signal_strength;       // bytes 26-27
    uint16_t free_memory;  // bytes 28-29
    uint8_t retries;       // byte 30
    std::string *status;

    TelemetryMessage();
    ~TelemetryMessage();
    void serialize(std::vector<uint8_t> *v);
};

class TelemetryAck {
public:
    RoverTimestamp *timestamp;
    bool ack;
    bool command_waiting;

    TelemetryAck();
    // deserializing constructor
    explicit TelemetryAck(std::vector<uint8_t> *v);
    ~TelemetryAck();
};

class CommandReady {
public:
    RoverTimestamp *timestamp;
    bool ready;

    CommandReady();
    __attribute__((unused)) CommandReady(RoverTimestamp *timestamp, bool ready);
    ~CommandReady();
    void serialize(std::vector<uint8_t> *v) const;
};

class CommandMessage {
    RoverTimestamp *timestamp;
    bool sequence_complete;
    std::string *command;

    CommandMessage();
    // deserializing constructor
    __attribute__((unused)) explicit CommandMessage(std::vector<uint8_t> *v);
    ~CommandMessage();
};

class CommandAck {
    RoverTimestamp *timestamp;
    bool ack;

    CommandAck();
    __attribute__((unused)) CommandAck(RoverTimestamp *timestamp, bool ack);
    ~CommandAck();
    void serialize(std::vector<uint8_t> *v) const;
};

/******************************************************************
 * Messaging
 ******************************************************************/
TelemetryMessage * collectTelemetry();

// debug toggles
#define USB_DEBUG // uncomment to enable debug messages over USB port
#define GPSECHO false  // set true to echo raw GPS data to console for debugging

void debug(const String& s);
uint16_t free_ram();

#define USE_ENCRYPTION // comment out to send unencrypted messages (packet size can be bigger)
#ifdef USE_ENCRYPTION
#define MAX_MESSAGE_SIZE 59
#else
#define MAX_MESSAGE_SIZE 250
#endif

// global setup for GPS
// creating a second Serial on SERCOM3
// see: https://learn.adafruit.com/using-atsamd21-sercom-to-add-more-spi-i2c-serial-ports/creating-a-new-serial
// NOTE: SERCOM3 is the ONLY device I could get to work as a second UART on the feather M4. After lengthy study
// of the datasheet, it looked like sercom4/alt and sercom0/alt should have worked, but in practice neither one
// did. It's possible that there's some other step needed (besides using PIO_SERCOM_ALT in the pinPeripheral
// call during setup) that I was missing.
#define PIN_SERIAL2_RX 13  // PA23
#define PAD_SERIAL2_RX (SERCOM_RX_PAD_1)
#define PIN_SERIAL2_TX 12  // PA22
#define PAD_SERIAL2_TX (UART_TX_PAD_0)

#endif
