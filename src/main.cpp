/******************************************************************
 * Prototype rover code
 * This code targets the following hardware...
 *
 * Adafruit Feather M4 Express board:
 * https://learn.adafruit.com/adafruit-feather-m4-express-atsamd51
 *
 * Adafruit 128x64 OLED display Featherwing:
 * https://learn.adafruit.com/adafruit-128x64-oled-featherwing
 *
 * Adafruit Radio FeatherWing - RFM69HCW 900MHz
 * https://learn.adafruit.com/radio-featherwing
 *
 * Adafruit DC Motor + Stepper FeatherWing
 * https://learn.adafruit.com/adafruit-stepper-dc-motor-featherwing
 *
 * Adafruit 9-DoF IMU ISM330DHCX + LIS3MDL Featherwing
 * https://learn.adafruit.com/st-9-dof-combo
 *
 * Adafruit Ultimate GPS Featherwing
 * https://learn.adafruit.com/adafruit-ultimate-gps-featherwing
 * Note: GPS firmware does not handle 2038 rollover, so if I'm
 * still alive then, I'll need to update. See:
 * https://learn.adafruit.com/adafruit-ultimate-gps-featherwing/f-a-q
 *
 * Adafruit Adalogger Featherwing
 * https://learn.adafruit.com/adafruit-adalogger-featherwing
 *
 * Adafruit INA219 Power Monitoring Featherwing
 * https://learn.adafruit.com/adafruit-ina219-current-sensor-breakout
 *
 * Adafruit BME680 temperature / humidity / pressure / gas sensor
 * https://learn.adafruit.com/adafruit-bme680-humidity-temperature-barometic-pressure-voc-gas/
 * 
 * Slamtec RoboPeak RPLIDAR
 * https://www.adafruit.com/product/4010
 * https://cdn-shop.adafruit.com/product-files/4010/4010_datasheet.pdf
 * http://www.slamtec.com/en/Support#rplidar-a1
 * http://www.robopeak.net/data/doc/rplidar/appnote/RPLDAPPN01-rplidar_appnote_arduinolib-enUS.pdf
 * https://github.com/robopeak/rplidar_arduino
 * See also: https://github.com/DanCrank/rplidar_arduino/tree/begin-returns-void
 *
 * Vehicle hardware is currently the same as the previous rover:
 * DFRobot Devastator tank chassis (6V metal gear motor version)
 * https://www.dfrobot.com/product-1477.html
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
 ******************************************************************/

#include <string>
#include <exception>
#include <stdexcept>
#include <vector>
#include <Arduino.h>
#include "wiring_private.h" // needed for setting up Serial2
#include <SPI.h>
#include <U8g2lib.h>
#include <RH_RF69.h>
#include <Adafruit_GPS.h>
#include "TimerInterrupt_Generic.h"
#include "ISR_Timer_Generic.h"
#include "RPLidar.h"
#include <encryption_key.h> // defines 16-byte array encryption_key and 2-byte array sync_words

#define USB_DEBUG

#define USE_ENCRYPTION
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

Uart Serial2(&sercom3, PIN_SERIAL2_RX, PIN_SERIAL2_TX, PAD_SERIAL2_RX, PAD_SERIAL2_TX);

void SERCOM3_0_Handler()
{
  Serial2.IrqHandler();
}
void SERCOM3_1_Handler()
{
  Serial2.IrqHandler();
}
void SERCOM3_2_Handler()
{
  Serial2.IrqHandler();
}
void SERCOM3_3_Handler()
{
  Serial2.IrqHandler();
}

Adafruit_GPS GPS(&Serial2);
#define GPSECHO false // set true to echo raw GPS data to console for debugging

// interrupt routine for ingesting GPS data
#define READGPS_INTERVAL_MS 1
void readGPSData(void)
{
    if (GPS.available()) {
        char c = GPS.read();
        if (GPSECHO) {
            Serial.write(c);
        }
    }
}
SAMDTimer GPSTimer(TIMER_TC3);

// global fn for checking free memory
extern "C" char *sbrk(int i);

unsigned short free_ram () {
    char stack_dummy = 0;
    return &stack_dummy - sbrk(0);
}

// send a debug message to the USB port
void debug(const String& s)
{
#ifdef USB_DEBUG
    String t = "00000000" + String(millis());
    t.remove(0, t.length() - 8);
    Serial.println(t + " | " + s);
    Serial.flush();
#endif
}

// global for keeping signal strength of last received message
int16_t last_signal_strength = 0;

// global for timing telemetry packets
uint32_t telemetry_timer = millis();

// Singleton instance of the radio driver
RH_RF69 rf69(11, 19);

// Singleton instance of the display driver
// https://github.com/olikraus/u8g2/wiki/u8g2setupcpp
U8G2_SH1107_64X128_2_HW_I2C u8g2(/* rotation=*/ U8G2_R3, /* reset=*/ U8X8_PIN_NONE, /* clock=*/ 21, /* data=*/ 22);

// Singleton instance of the RPLidar driver
RPLidar lidar;
const unsigned int RPLIDAR_MOTOR = 16;

// keep a global buffer representing the most recent scan of the surroundings
// each bin is a ten-degree segment (0-9.99, 10-19.99, etc.)
// if you want to use a different bin / buffer size, keep in mind that the
// code is currently optimized for an even number of bins and an even
// number of degrees in each bin. so good values of NAV_BUFFER_SIZE would
// be 18, 20, 30, 36, 60, 90, 180
struct nav_scan
{
  unsigned int distanceNear;
  unsigned int pointCount;
};
const unsigned int NAV_BUFFER_SIZE = 60;
nav_scan navBuffer[NAV_BUFFER_SIZE];

void initNavBuffer()
{
  debug("InitNavBuffer()");
  for (unsigned int i = 0; i < NAV_BUFFER_SIZE; i++)
  {
    navBuffer[i].distanceNear = 0;
    navBuffer[i].pointCount = 0;
  }
}

// setting to collect more than one complete LIDAR rotation in a "scan".
// this must be at least 1. values higher than 1 may help the vehicle make
// better maneuvering decisions, but interfere with the actual maneuvering
// because the time spent scanning extends the length of the decision-making
// cycle.
const unsigned int OVERSCAN = 1;

// LIDAR motor speed (0-255)
const unsigned int LIDAR_MOTOR_SPEED = 255;

// minimum distance, inside which LIDAR data points are thrown out. this is
// used to filter out spurious data points at unreasonably close distances
// (inside the perimeter of the vehicle or thereabouts). value in mm.
// the datasheet lists minimum measurable distance as 150mm, so that's a
// good initial value.
const unsigned int LIDAR_MINIMUM_DISTANCE = 100;

// minimum quality, beneath which LIDAR data points are thrown out. this is
// a score assigned by the device, and its meaning is not documented (it is
// an unsigned byte coming from the device). in practice, most data points
// are observed to be quality = 15.
const unsigned int LIDAR_MINIMUM_QUALITY = 4;

// message timing parameters
#define ACK_TIMEOUT 1000 // millis to wait for an ack msg
#define MSG_DELAY 100 // millis to wait between Rx and Tx, to give the other side time to switch from Tx to Rx
#define LISTEN_DELAY 50 // millis to wait between checks of the receive buffer when receiving
#define TELEMETRY_INTERVAL 5000 // millis to wait after a successful telemetry exchange before sending another

// message IDs
#define MESSAGE_TELEMETRY 0
#define MESSAGE_TELEMETRY_ACK 1
#define MESSAGE_COMMAND_READY 2
#define MESSAGE_COMMAND 3
#define MESSAGE_COMMAND_ACK 4

String message_type(unsigned char i) {
    switch (i) {
        case MESSAGE_TELEMETRY: return "MESSAGE_TELEMETRY";
        case MESSAGE_TELEMETRY_ACK: return "MESSAGE_TELEMETRY_ACK";
        case MESSAGE_COMMAND_READY: return "MESSAGE_COMMAND_READY";
        case MESSAGE_COMMAND: return "MESSAGE_COMMAND";
        case MESSAGE_COMMAND_ACK: return "MESSAGE_COMMAND_ACK";
        default: return "MESSAGE_UNKNOWN";
    }
}

// serialization / deserialization helper functions
void serialize_float(float *f, std::vector<unsigned char> *v) {
    auto *byte = reinterpret_cast<unsigned char *>(f);
    v->push_back(byte[0]);
    v->push_back(byte[1]);
    v->push_back(byte[2]);
    v->push_back(byte[3]);
}

float deserialize_float(std::vector<unsigned char> *v, size_t i)
noexcept(false)
{
    unsigned char arr[4] = {v->at(i), v->at(i+1), v->at(i+2), v->at(i+3)};
    return *arr;
}

void serialize_ushort(unsigned short *i, std::vector<unsigned char> *v) {
    auto *byte = reinterpret_cast<unsigned char *>(i);
    v->push_back(byte[0]);
    v->push_back(byte[1]);
}

unsigned short deserialize_ushort(std::vector<unsigned char> *v, size_t i)
noexcept(false)
{
    unsigned char arr[2] = {v->at(i), v->at(i+1)};
    return *arr;
}

void serialize_short(short *i, std::vector<unsigned char> *v) {
    auto *byte = reinterpret_cast<unsigned char *>(i);
    v->push_back(byte[0]);
    v->push_back(byte[1]);
}

short deserialize_short(std::vector<unsigned char> *v, size_t i)
noexcept(false)
{
    unsigned char arr[2] = {v->at(i), v->at(i+1)};
    return *arr;
}

void serialize_string(std::string *s, std::vector<unsigned char> *v) {
    for (char & it : *s)
        v->push_back(it);
    v->push_back(0);
}

std::string *deserialize_string(std::vector<unsigned char> *v, size_t i)
noexcept(false)
{
    auto *s = new std::string();
    char c;
    while ((c = v->at(i)) != 0) {
        s->push_back(c);
    }
    return s;
}

// RadioHead enforces the first 4 bytes of the payload of every message as TO, FROM, ID, FLAGS
// that means the max payload of a message is 60 bytes
class RoverTimestamp {
public:
    unsigned char year; // two-digit year. sue me on 1/1/3000
    unsigned char month;
    unsigned char day;
    unsigned char hour;
    unsigned char minute;
    unsigned char second;

    RoverTimestamp() {
        // this will be UTC time
        year = GPS.year;
        month = GPS.month;
        day = GPS.day;
        hour = GPS.hour;
        minute = GPS.minute;
        second = GPS.seconds;
    }

    // deserializing constructor
    RoverTimestamp(std::vector<unsigned char> *v, size_t i)
    noexcept(false)
    {
        year = v->at(i);
        month = v->at(i+1);
        day = v->at(i+2);
        hour = v->at(i+3);
        minute = v->at(i+4);
        second = v->at(i+5);
    }

    // serialize a timestamp onto the given vector
    void serialize(std::vector<unsigned char> *v) const {
        v->push_back(year);
        v->push_back(month);
        v->push_back(day);
        v->push_back(hour);
        v->push_back(minute);
        v->push_back(second);
    }
};

class RoverLocData {
public:
    float gps_lat;
    float gps_long;
    float gps_alt;
    float gps_speed;
    unsigned char gps_sats;
    unsigned short gps_hdg;

    RoverLocData() {
        gps_sats = GPS.satellites;
        if (GPS.fix) {
            gps_lat = GPS.latitudeDegrees; // there's also a N/S/E/W attached to these?
            gps_long = GPS.longitudeDegrees;
            gps_alt = GPS.altitude; // this is in meters, THANKS COMMUNISTS
            gps_speed = GPS.speed; // in knots for some reason, THANKS SAILORS TODO: convert
            gps_hdg = GPS.angle;
        } else {
            gps_lat = 0.0; // a big bucket of zeroes means the GPS doesn't have a fix yet
            gps_long = 0.0;
            gps_alt = 0.0;
            gps_speed = 0.0;
            gps_hdg = 0;
        }
    }

    // serialize loc data onto the given vector
    void serialize(std::vector<unsigned char> *v) {
        serialize_float(&gps_lat, v);
        serialize_float(&gps_long, v);
        serialize_float(&gps_alt, v);
        serialize_float(&gps_speed, v);
        v->push_back(gps_sats);
        serialize_ushort(&gps_hdg, v);
    }
};

// TODO - make a virtual base class to factor out timestamp and any other common stuff
class TelemetryMessage {
public:
    RoverTimestamp *timestamp;
    RoverLocData *location;
    short signal_strength;
    unsigned short free_memory;
    std::string *status;

    TelemetryMessage() {
        timestamp = new RoverTimestamp();
        location = new RoverLocData();
        status = new std::string();
        this->signal_strength = last_signal_strength;
        free_memory = free_ram();
    }

    ~TelemetryMessage() {
        delete timestamp;
        delete location;
        delete status;
    }

    void serialize(std::vector<unsigned char> *v) {
        v->reserve(status->length() + 27);
        v->push_back(MESSAGE_TELEMETRY);
        timestamp->serialize(v);
        location->serialize(v);
        serialize_short(&signal_strength, v);
        serialize_ushort(&free_memory, v);
        serialize_string(status, v);
    }
};

class TelemetryAck {
public:
    TelemetryAck() {
        timestamp = new RoverTimestamp();
        ack = true;
        command_waiting = false;
    }

    // deserializing constructor
    explicit TelemetryAck(std::vector<unsigned char> *v)
    noexcept(false)
    {
        // check message type
        if (v->at(0) != MESSAGE_TELEMETRY_ACK)
            throw std::runtime_error(reinterpret_cast<const char *>(String(
                    "Wrong message type: expected MESSAGE_TELEMETRY_ACK, got ").concat(message_type(v->at(5)))));
        timestamp = new RoverTimestamp(v, 1);
        ack = ((v->at(7)) > 0);
        command_waiting = ((v->at(8)) > 0);
    }

    ~TelemetryAck() {
        delete timestamp;
    }

    RoverTimestamp *timestamp;
    bool ack;
    bool command_waiting;
};

class CommandReady {
public:
    CommandReady() {
        timestamp = new RoverTimestamp();
        ready = true;
    }

    __attribute__((unused)) CommandReady(RoverTimestamp *timestamp, bool ready) {
        this->timestamp = timestamp;
        this->ready = ready;
    }

    ~CommandReady() {
        delete timestamp;
    }

    RoverTimestamp *timestamp;
    bool ready;

    void serialize(std::vector<unsigned char> *v) const {
        v->reserve(5);
        v->push_back(MESSAGE_COMMAND_READY);
        timestamp->serialize(v);
        if (ready) v->push_back(1); else v->push_back(0);
    }
};

class CommandMessage {
    CommandMessage() {
        timestamp = new RoverTimestamp();
        sequence_complete = false;
        command = new std::string();
    }

    // deserializing constructor
    __attribute__((unused)) explicit CommandMessage(std::vector<unsigned char> *v)
    noexcept(false)
    {
        // check message type
        if (v->at(0) != MESSAGE_COMMAND)
            throw std::runtime_error(reinterpret_cast<const char *>(String(
                    "Wrong message type: expected MESSAGE_COMMAND, got ").concat(message_type(v->at(5)))));
        timestamp = new RoverTimestamp(v, 1);
        sequence_complete = ((v->at(7)) > 0);
        command = deserialize_string(v, 8);
    }

    ~CommandMessage() {
        delete timestamp;
        delete command;
    }

    RoverTimestamp *timestamp;
    bool sequence_complete;
    std::string *command;
};

class CommandAck {
    CommandAck() {
        timestamp = new RoverTimestamp();
        ack = true;
    }

    __attribute__((unused)) CommandAck(RoverTimestamp *timestamp, bool ack) {
        this->timestamp = timestamp;
        this->ack = ack;
    }

    ~CommandAck() {
        delete timestamp;
    }

    RoverTimestamp *timestamp;
    bool ack;

    void serialize(std::vector<unsigned char> *v) const {
        v->push_back(MESSAGE_COMMAND_ACK);
        timestamp->serialize(v);
        if (ack) v->push_back(1); else v->push_back(0);
    }};

// dump a byte vector representing a serialized message
void dump_buffer(std::vector<unsigned char> *v) {
#ifdef USB_DEBUG
    int i = 0;
    String s = "";
    for (unsigned char & it : *v) {
        s += String(it, HEX) + " ";
        if (i % 8 == 7) {
            debug(s);
            i = 0;
            s = "";
        }
    }
    if (s.length() > 0) debug (s);
#endif
}

// display a message on the OLED display
void display(const String& str) {
    debug("Display: " + str);
    u8g2.firstPage();
    do {
        u8g2.setCursor(0, 20);
        u8g2.print(str);
    } while ( u8g2.nextPage() );
}

void update_signal_strength() {
    last_signal_strength = rf69.lastRssi();
}

/*
 * LIDAR functions
 */

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
  unsigned long startTime = millis();
  debug("lidarScan()");
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
          debug("Scan complete; " + String(pointsCollected) + " points collected, " + String(pointsThrownOut) + " thrown out");
          debug("Scan took " + String(millis() - startTime) + " milliseconds at overscan=" + OVERSCAN);
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
      debug("lidarScan() timed out while waiting for data point");
      return false;
    }
  }
}

// return the bin index of the nearest obstruction, or -1 if no obstructions are visible
// inside the NAV_AVOID_CONE and within NAV_AVOID_DISTANCE millimeters.
// temporary values for hacking
#define NAV_AVOID_DISTANCE 99999
#define NAV_AVOID_CONE 30
int navFindObstruction(int sector = (NAV_BUFFER_SIZE / 2), unsigned int distance = NAV_AVOID_DISTANCE)
{
  debug("navFindObstruction()");
  int bin = -1;
  unsigned int range = distance + 1;
  for (int i = sector - NAV_AVOID_CONE; i <= sector + NAV_AVOID_CONE; i++)
  {
    int realIndex = (i + NAV_BUFFER_SIZE) % NAV_BUFFER_SIZE;  // handle indexes that span buffer boundaries
    if (navBuffer[realIndex].pointCount == 0)
    {
      debug("I have no data for sector " + String(i) + ", declaring that sector obstructed.");
      //bin = realIndex;
      //range = 0;
    } else if ((navBuffer[realIndex].distanceNear < distance) &&
             (navBuffer[realIndex].distanceNear < range)) {
      bin = realIndex;
      range = navBuffer[realIndex].distanceNear;
    }
  }
  if (bin == -1)
    debug("No obstruction detected");
  else
    debug("Nearest obstruction detected in sector " + String(bin) + ", range=" + String(range) + "mm");
  return bin;
}

TelemetryMessage * collectTelemetry() {
    auto *msg = new TelemetryMessage();
    // status
    // msg->status->assign("READY FOR ADVENTURE");
    // temporarily using this for lidar data until I add it to the telemetry message
    int obstruction = navFindObstruction();
    if (obstruction > 0) {
        msg->status->assign("Obstruction in sector ");
        msg->status->append(String(obstruction).c_str());
    } else
        msg->status->assign("No obstructions seen");
    return msg;
}

// send a telemetry packet and wait for acknowledgement; retry if NAK'ed or no ACK
// currently, this function will just retry forever, so there is no error return
void sendTelemetry() {
    debug("sendTelemetry()");
    TelemetryMessage *msg = collectTelemetry();
    auto *v = new std::vector<unsigned char>;
    // serialize the message
    msg->serialize(v);
    delete msg;
    //dump_buffer(v);
    // check the size - must be <= MAX_MESSAGE_SIZE
    if (v->size() > MAX_MESSAGE_SIZE) {
        debug("Telemetry message too large! Cannot send!");
        delete v;
        return;
    }
    bool complete = false;
    unsigned short tries = 0;
    while (!complete) {
        // send it
        if (tries == 0)
            debug("Sending telemetry packet...");
        else
            debug("Sending telemetry packet...RETRY " + String(tries));
        rf69.send(v->data(), v->size());
        debug("Telemetry packet sent, waiting for ACK");
        display("Waiting for ACK");
        // wait for ack
        unsigned char ack_buf[64];
        unsigned long start = millis();
        unsigned long now = start;
        uint8_t len = 0;
        while (now < start + ACK_TIMEOUT) {
            if (rf69.available())
            {
                debug("available() returned true");
                len = 64;
                bool r = rf69.recv(reinterpret_cast<uint8_t *>(&ack_buf), &len);
                debug("r=" + String(r) + ", len=" + String(len));
                if (r && (len > 0)) {
#ifdef USB_DEBUG
                    debug("Received response...");
                    for (int i = 0; i < 64; i += 8) {
                       String s;
                       s.reserve(24);
                       s.concat(String(ack_buf[i], HEX));
                       s.concat(' ');
                       s.concat(String(ack_buf[i + 1], HEX));
                       s.concat(' ');
                       s.concat(String(ack_buf[i + 2], HEX));
                       s.concat(' ');
                       s.concat(String(ack_buf[i + 3], HEX));
                       s.concat(' ');
                       s.concat(String(ack_buf[i + 4], HEX));
                       s.concat(' ');
                       s.concat(String(ack_buf[i + 5], HEX));
                       s.concat(' ');
                       s.concat(String(ack_buf[i + 6], HEX));
                       s.concat(' ');
                       s.concat(String(ack_buf[i + 7], HEX));
                       debug(s);
                   }
#endif
                    // deserialize
                    // this is memory inefficient, but copying to a vector here lets
                    // all the deserialization functions automatically do range checking
                    auto *ack_vec = new std::vector<unsigned char>();
                    for (unsigned char i : ack_buf) ack_vec->push_back(i);
                    try {
                        auto *ack = new TelemetryAck(ack_vec);
                        delete ack_vec;
                        debug("Received TACK (ack=" + String(ack->ack) + ", command_waiting=" + String(ack->command_waiting) + ")");
                        display("TACK (ack=" + String(ack->ack) + ",cmd=" + String(ack->command_waiting) + ")");
                        if (ack->ack) {
                            complete = true;
                            delete ack;
                            break;
                        }
                        delete ack;
                    } catch (std::exception &e) { debug("Error while deserializing TACK"); }
                    // if we get here, either the station NAK'ed us or sent a garbled ACK, so re-send
                    debug("NAK! re-sending last packet.");
                    display("NAK!");
                } else {
                    // here if available() said there was a message but recv() returned false...
                    debug("Error while receiving TACK");
                    display("ERR");
                }
                break;
            } else {
                // here if no message yet
                //debug("No reply yet...");
                delay(LISTEN_DELAY);
                now = millis();
            }
        }
        // if we get here and complete==false, we timed out waiting for ACK, so re-send
        tries++;
        delay(MSG_DELAY);
    }
    delete v;
    update_signal_strength();
    debug("sendTelemetry() complete.");
}

void setup() {
    // initialize OLED display
    u8g2.begin();
    u8g2.setFont(u8g2_font_courB08_tf);

#ifdef USB_DEBUG
    display("Waiting for serial debug connection");
    while (!Serial);
    // start USB port for debug messages
    Serial.begin(115200);
    debug("setup()");
#endif

    // initialize RFM69
    display("initializing RFM69");
    if (!rf69.init())
        display("init failed");
    if (!rf69.setFrequency(915))
        display("setFrequency failed");
    if (!rf69.setModemConfig(rf69.FSK_Rb9_6Fd19_2)) // FSK, Whitening, Rb = 9.6kbs,  Fd = 19.2kHz
        display("setModemConfig failed");
    rf69.setSyncWords(sync_words, 2);  // must define in encryption_key.h, 2-byte array
    rf69.setEncryptionKey(encryption_key); // must define in encryption_key.h, 16-byte array
    rf69.setTxPower(17, true);
    uint16_t version = rf69.deviceType();
#ifdef USB_DEBUG
    debug(String("RFM69 initialized: version 0x" + String(version, HEX)));
    rf69.printRegisters();
#endif
    // initialize GPS / RTC
    display("initializing GPS");
    // Assign RX and TX pins to SERCOM
    // see: https://learn.adafruit.com/using-atsamd21-sercom-to-add-more-spi-i2c-serial-ports/creating-a-new-serial
    pinPeripheral(PIN_SERIAL2_RX, PIO_SERCOM);
    pinPeripheral(PIN_SERIAL2_TX, PIO_SERCOM);
    // copied from https://platformio.org/lib/show/20/Adafruit%20GPS%20Library
    GPS.begin(9600);
    delay(1000);
    GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);
    GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ); // 1 Hz update rate
    GPS.sendCommand(PGCMD_ANTENNA);
    GPS.sendCommand(PMTK_Q_RELEASE);
    debug("GPS initialized");

    // start interrupt timer for GPS
    if (GPSTimer.attachInterruptInterval(READGPS_INTERVAL_MS, readGPSData))
        display("GPS interrupt started");
    else
        display("GPS interrupt failed");

    // initialize LIDAR
    display("initializing LIDAR");
    lidarInit();
    if (!lidarConnect())
        display("LIDAR connect failed");
    analogWrite(RPLIDAR_MOTOR, LIDAR_MOTOR_SPEED);
}

void loop() {
    if (GPS.newNMEAreceived()) {
        GPS.parse(GPS.lastNMEA()); // this also sets the newNMEAreceived() flag to false
    }
    if (!lidarScan())
        display("Lidar scan failed!");
    if (millis() - telemetry_timer > TELEMETRY_INTERVAL) {
        telemetry_timer = millis();
        sendTelemetry();
    }
}