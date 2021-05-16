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
 *
 * Adafruit Adalogger Featherwing
 * https://learn.adafruit.com/adafruit-adalogger-featherwing
 *
 * Adafruit INA219 Power Monitoring Featherwing
 * https://learn.adafruit.com/adafruit-ina219-current-sensor-breakout
 *
 * Slamtec RoboPeak RPLIDAR
 * https://www.adafruit.com/product/4010
 * https://cdn-shop.adafruit.com/product-files/4010/4010_datasheet.pdf
 * http://www.slamtec.com/en/Support#rplidar-a1
 * http://www.robopeak.net/data/doc/rplidar/appnote/RPLDAPPN01-rplidar_appnote_arduinolib-enUS.pdf
 * https://github.com/robopeak/rplidar_arduino
 * See also: https://github.com/DanCrank/rplidar_arduino/tree/begin-returns-void
 *
 * Vehicle hardware TBD
 *
 * Camera(s) TBD (Is there a solution for interfacing a camera with a Feather?)
 *
 * Other sensors TBD
 *
 * Pin assignments:
 * 0-1 reserved for hardware UART
 * 8 maybe reserved for onboard NeoPixel?
 * 21-22 reserved for I2C
 * 23-25 reserved for SPI
 *
 * LIDAR: will need to go to a software UART - RX (16), TX (17), connect motor control to 14
 * Radio: MOSI (24), MISO (23), SCK (25), jumper CS to 11 (labeled A), IRQ to 19 (labeled F),
 *          RST to 5 (labeled E) (so display pushbutton can reset the radio?)
 * Display: uses I2C, pushbuttons are on 5, 6, 9
 * Motor driver: uses I2C
 * Accelerometer: (I think) can use either I2C or SPI - stick with I2C for now
 * GPS: uses RX (0), TX (1)
 * Adalogger: RTC uses I2C, SD uses SPI with CS on 10
 * Current sensor: uses I2C
 *
 * TODO go through this whole thing and look for memory leaks
 ******************************************************************/

#include <RH_RF69.h>
#include <SPI.h>
#include <string>
#include <U8g2lib.h>
#include <exception>
#include <stdexcept>
#include <vector>

#define USB_DEBUG

#define USE_ENCRYPTION
#ifdef USE_ENCRYPTION
#define MAX_MESSAGE_SIZE 59
#else
#define MAX_MESSAGE_SIZE 250
#endif

extern "C" char *sbrk(int i); // used to measure free memory

unsigned short free_ram () {
    char stack_dummy = 0;
    return &stack_dummy - sbrk(0);
}

int16_t last_signal_strength = 0;

#define ACK_TIMEOUT 2000 // millis to wait for an ack msg
#define MSG_DELAY 250 // millis to wait between Rx and Tx, to give the other side time to switch from Tx to Rx
#define LISTEN_DELAY 100 // millis to wait between checks of the receive buffer when receiving
#define TELEMETRY_INTERVAL 5000 // millis to wait after a successful telemetry exchange before sending another

// Singleton instance of the radio driver
RH_RF69 rf69(11, 19);

// https://github.com/olikraus/u8g2/wiki/u8g2setupcpp
U8G2_SH1107_64X128_2_HW_I2C u8g2(/* rotation=*/ U8G2_R3, /* reset=*/ U8X8_PIN_NONE, /* clock=*/ 21, /* data=*/ 22);

// message definitions
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
    RoverTimestamp() {
        // TODO set this from the RTC
        hour = 0;
        minute = 0;
        second = 0;
    }

    // deserializing constructor
    RoverTimestamp(std::vector<unsigned char> *v, size_t i)
    noexcept(false)
    {
        hour = v->at(i);
        minute = v->at(i+1);
        second = v->at(i+2);
    }

    unsigned char hour;
    unsigned char minute;
    unsigned char second;

    // serialize a timestamp onto the given vector
    void serialize(std::vector<unsigned char> *v) const {
        v->push_back(hour);
        v->push_back(minute);
        v->push_back(second);
    }
};

class RoverLocData {
public:
    RoverLocData() {
        // TODO set these from the GPS
        gps_lat = 0.0;
        gps_long = 0.0;
        gps_alt = 0.0;
        gps_speed = 0.0;
        gps_sats = 0;
        mag_hdg = 0;
    }

    float gps_lat;
    float gps_long;
    float gps_alt;
    float gps_speed;
    unsigned char gps_sats;
    unsigned short mag_hdg;

    // serialize loc data onto the given vector
    void serialize(std::vector<unsigned char> *v) {
        serialize_float(&gps_lat, v);
        serialize_float(&gps_long, v);
        serialize_float(&gps_alt, v);
        serialize_float(&gps_speed, v);
        v->push_back(gps_sats);
        serialize_ushort(&mag_hdg, v);
    }
};

// TODO - make a virtual base class to factor out timestamp and any other common stuff
class TelemetryMessage {
public:
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

    RoverTimestamp *timestamp;
    RoverLocData *location;
    short signal_strength;
    unsigned short free_memory;
    std::string *status;

    void serialize(std::vector<unsigned char> *v) {
        v->reserve(status->length() + 24);
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
        ack = ((v->at(4)) > 0);
        command_waiting = ((v->at(5)) > 0);
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
        sequence_complete = ((v->at(4)) > 0);
        command = deserialize_string(v, 5);
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
    u8g2.firstPage();
    do {
        u8g2.setCursor(0, 20);
        u8g2.print(str);
    } while ( u8g2.nextPage() );
}

void setup() {
#ifdef USB_DEBUG
    // start USB port for debug messages
    Serial.begin(57600);
    debug("setup()");
#endif

    // initialize OLED display
    u8g2.begin();
    u8g2.setFont(u8g2_font_courB08_tf);

    // initialize RFM69
    display("initializing RFM69");
    if (!rf69.init())
        display("init failed");
    if (!rf69.setFrequency(915))
        display("setFrequency failed");
    if (!rf69.setModemConfig(rf69.FSK_Rb9_6Fd19_2)) // FSK, Whitening, Rb = 9.6kbs,  Fd = 19.2kHz
        display("setModemConfig failed");
    rf69.setTxPower(17, true);
    // TODO encryption key
//    uint8_t key[] = { 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08,
//                      0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08};
//    rf69.setEncryptionKey(key);
    uint version = rf69.deviceType();
    display(String("RFM69: 0x" + String(version, HEX)));
    delay(3000);
}

TelemetryMessage * collectTelemetry() {
    auto *msg = new TelemetryMessage();
    msg->location->gps_alt = 69000.0;
    msg->location->gps_sats = 42;
    // status
    msg->status->assign("READY FOR ADVENTURE");
    return msg;
}

void update_signal_strength() {
    last_signal_strength = rf69.lastRssi();
    display("Signal: " + String(last_signal_strength));
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
    while (!complete) {
        // send it
        debug("Sending telemetry packet...");
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
                len = 64;
                bool r = rf69.recv(reinterpret_cast<uint8_t *>(&ack_buf), &len);
//                debug("r=" + String(r) + ", len=" + String(len));
                if (r && (len > 0)) {
//                    debug("Received response...");
//                    for (int i = 0; i < 64; i += 8) {
//                        String s;
//                        s.reserve(24);
//                        s.concat(String(ack_buf[i], HEX));
//                        s.concat(' ');
//                        s.concat(String(ack_buf[i + 1], HEX));
//                        s.concat(' ');
//                        s.concat(String(ack_buf[i + 2], HEX));
//                        s.concat(' ');
//                        s.concat(String(ack_buf[i + 3], HEX));
//                        s.concat(' ');
//                        s.concat(String(ack_buf[i + 4], HEX));
//                        s.concat(' ');
//                        s.concat(String(ack_buf[i + 5], HEX));
//                        s.concat(' ');
//                        s.concat(String(ack_buf[i + 6], HEX));
//                        s.concat(' ');
//                        s.concat(String(ack_buf[i + 7], HEX));
//                        debug(s);
//                    }
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
        delay(MSG_DELAY);
    }
    delete v;
    update_signal_strength();
    debug("sendTelemetry() complete.");
}

void loop() {
#ifdef USB_DEBUG
    display("Waiting 30 seconds...");
    delay(20000);
    for (int i = 10; i >= 0; i--) {
        display(String(i));
        delay(1000);
    }
    //rf69.printRegisters();
#endif
    for (int i = 0; i < 1000000; i++) {
        sendTelemetry();
        delay(TELEMETRY_INTERVAL);
    }
}