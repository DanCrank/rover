/******************************************************************
 * gps management
 ******************************************************************/

#include "rover.h"
#include <Adafruit_GPS.h>
#include <wiring_private.h>

extern StatusDisplay *statusDisplay;

// global second UART on SERCOM3 for the GPS
Uart Serial2(&sercom3, PIN_SERIAL2_RX, PIN_SERIAL2_TX, PAD_SERIAL2_RX, PAD_SERIAL2_TX);

bool GPSSetup = false;

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

void setupGPS() {
    // initialize GPS / RTC
    debug("initializing GPS");
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
    GPSSetup = true;

}

void checkForNewGPSData() {
    if (GPS.newNMEAreceived()) {
        GPS.parse(GPS.lastNMEA()); // this also sets the newNMEAreceived() flag to false
        statusDisplay->setGpsSats(GPS.satellites);
    }
}

// interrupt routine for ingesting GPS data
void readGPSData(void)
{
    if (GPS.available()) {
        int8_t c = GPS.read();
        if (GPSECHO) {
                Serial.write(c);
        }
    }
}
