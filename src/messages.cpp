/******************************************************************
 * message handling
 ******************************************************************/

#include "rover.h"
#include <Adafruit_GPS.h>
#include <RH_RF69.h>
#include <stdexcept>

extern Adafruit_GPS GPS;
extern RH_RF69 rf69;
extern int16_t lastSignalStrength;

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

/******************************************************************
 * serialization / deserialization helper functions
 ******************************************************************/
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

/******************************************************************
 * RoverTimestamp
 ******************************************************************/
RoverTimestamp::RoverTimestamp() {
    // this will be UTC time
    year = GPS.year;
    month = GPS.month;
    day = GPS.day;
    hour = GPS.hour;
    minute = GPS.minute;
    second = GPS.seconds;
}

// deserializing constructor
RoverTimestamp::RoverTimestamp(std::vector<unsigned char> *v, size_t i)
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
void RoverTimestamp::serialize(std::vector<unsigned char> *v) const {
    v->push_back(year);
    v->push_back(month);
    v->push_back(day);
    v->push_back(hour);
    v->push_back(minute);
    v->push_back(second);
}

/******************************************************************
 * RoverLocData
 ******************************************************************/
RoverLocData::RoverLocData() {
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
void RoverLocData::serialize(std::vector<unsigned char> *v) {
    serialize_float(&gps_lat, v);
    serialize_float(&gps_long, v);
    serialize_float(&gps_alt, v);
    serialize_float(&gps_speed, v);
    v->push_back(gps_sats);
    serialize_ushort(&gps_hdg, v);
}

/******************************************************************
 * TelemetryMessage
 ******************************************************************/
TelemetryMessage::TelemetryMessage() {
    timestamp = new RoverTimestamp();
    location = new RoverLocData();
    status = new std::string();
    this->signal_strength = lastSignalStrength;
    free_memory = free_ram();
    retries = 0;
}

TelemetryMessage::~TelemetryMessage() {
    delete timestamp;
    delete location;
    delete status;
}

void TelemetryMessage::serialize(std::vector<unsigned char> *v) {
    v->reserve(status->length() + 30);
    v->push_back(MESSAGE_TELEMETRY);
    timestamp->serialize(v);
    location->serialize(v);
    serialize_short(&signal_strength, v);
    serialize_ushort(&free_memory, v);
    v->push_back(retries);
    serialize_string(status, v);
}

/******************************************************************
 * TelemetryAck
 ******************************************************************/
TelemetryAck::TelemetryAck() {
    timestamp = new RoverTimestamp();
    ack = true;
    command_waiting = false;
}

// deserializing constructor
TelemetryAck::TelemetryAck(std::vector<unsigned char> *v)
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

TelemetryAck::~TelemetryAck() {
    delete timestamp;
}

/******************************************************************
 * CommandReady
 ******************************************************************/
CommandReady::CommandReady() {
    timestamp = new RoverTimestamp();
    ready = true;
}

__attribute__((unused)) CommandReady::CommandReady(RoverTimestamp *timestamp, bool ready) {
    this->timestamp = timestamp;
    this->ready = ready;
}

CommandReady::~CommandReady() {
    delete timestamp;
}

void CommandReady::serialize(std::vector<unsigned char> *v) const {
    v->reserve(5);
    v->push_back(MESSAGE_COMMAND_READY);
    timestamp->serialize(v);
    if (ready) v->push_back(1); else v->push_back(0);
}

/******************************************************************
 * CommandMessage
 ******************************************************************/
CommandMessage::CommandMessage() {
    timestamp = new RoverTimestamp();
    sequence_complete = false;
    command = new std::string();
}

// deserializing constructor
__attribute__((unused)) CommandMessage::CommandMessage(std::vector<unsigned char> *v)
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

CommandMessage::~CommandMessage() {
    delete timestamp;
    delete command;
}

/******************************************************************
 * CommandAck
 ******************************************************************/
CommandAck::CommandAck() {
    timestamp = new RoverTimestamp();
    ack = true;
}

__attribute__((unused)) CommandAck::CommandAck(RoverTimestamp *timestamp, bool ack) {
    this->timestamp = timestamp;
    this->ack = ack;
}

CommandAck::~CommandAck() {
    delete timestamp;
}

void CommandAck::serialize(std::vector<unsigned char> *v) const {
    v->push_back(MESSAGE_COMMAND_ACK);
    timestamp->serialize(v);
    if (ack) v->push_back(1); else v->push_back(0);
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
        //debug("Telemetry message too large! Cannot send!");
        delete v;
        return;
    }
    bool complete = false;
    unsigned char tries = 0;
    while (!complete) {
        // send it
        //if (tries == 0)
            //debug("Sending telemetry packet...");
        //else {
            //debug("Sending telemetry packet...RETRY " + String(tries));
            //to avoid having to re-serialize the message (and delete/re-allocate
            //the vector), poke the "tries" value directly into the
            //already-serialized vector
            //dump_buffer(v);
            //debug("Retries was: " + String(v->at(30)));
            v->at(30) = tries;
            //dump_buffer(v);
            //debug("Retries is now: " + String(v->at(30)));
        //}
        rf69.send(v->data(), v->size());
        //debug("Telemetry packet sent, waiting for ACK");
        // wait for ack
        unsigned char ack_buf[64];
        unsigned long start = millis();
        unsigned long now = start;
        uint8_t len = 0;
        while (now < start + ACK_TIMEOUT) {
            if (rf69.available())
            {
                //debug("available() returned true");
                len = 64;
                bool r = rf69.recv(reinterpret_cast<uint8_t *>(&ack_buf), &len);
                //debug("r=" + String(r) + ", len=" + String(len));
                if (r && (len > 0)) {
// #ifdef USB_DEBUG
//                     //debug("Received response...");
//                     for (int i = 0; i < 64; i += 8) {
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
// #endif
                    // deserialize
                    // this is memory inefficient, but copying to a vector here lets
                    // all the deserialization functions automatically do range checking
                    auto *ack_vec = new std::vector<unsigned char>();
                    for (unsigned char i : ack_buf) ack_vec->push_back(i);
                    try {
                        auto *ack = new TelemetryAck(ack_vec);
                        delete ack_vec;
                        //debug("Received TACK (ack=" + String(ack->ack) + ", command_waiting=" + String(ack->command_waiting) + ")");
                        if (ack->ack) {
                            complete = true;
                            delete ack;
                            break;
                        }
                        delete ack;
                    } catch (std::exception &e) { debug("Error while deserializing TACK"); }
                    // if we get here, either the station NAK'ed us or sent a garbled ACK, so re-send
                    //debug("NAK! re-sending last packet.");
                // } else {
                //     // here if available() said there was a message but recv() returned false...
                //     debug("Error while receiving TACK");
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
    updateSignalStrength();
    //debug("sendTelemetry() complete.");
}
