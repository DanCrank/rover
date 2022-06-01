/******************************************************************
 * Prototype rover code
 *
 * See rover.h for pin assignments
 ******************************************************************/

#include "rover.h"
#include <U8g2lib.h>
#include <RH_RF69.h>
#include <Adafruit_GPS.h>
#include "TimerInterrupt_Generic.h"

extern StatusDisplay *statusDisplay;

// global fn for checking free memory
extern "C" int8_t *sbrk(int16_t i);

uint16_t free_ram () {
    int8_t stack_dummy = 0;
    return &stack_dummy - sbrk(0);
}

// 1ms interrupt timer to drive various things
SAMDTimer MilliTimer(TIMER_TC3);

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

void milliInterrupt(void) {
    readGPSData();
}

void setup() {
    setupDisplay();

#ifdef USB_DEBUG
    statusDisplay->display("Waiting for serial debug");
    while (!Serial);
    // start USB port for debug messages
    Serial.begin(115200);
    debug("setup()");
    statusDisplay->returnToDefaultDisplay();
#endif

    setupRfm69();
    setupGPS();
    setupLidar();

    // start interrupt timer
    if (MilliTimer.attachInterruptInterval(READGPS_INTERVAL_MS, milliInterrupt))
        debug("Interrupt started");
    else
        debug("Interrupt failed");
}

uint32_t telemetryTimer = millis();
uint32_t displayUpdateTimer = millis();

void loop() {
    checkForNewGPSData();
    if (!lidarScan()) {
        //debug("Lidar scan failed!");
        statusDisplay->setLidarOK(false);
    } else statusDisplay->setLidarOK(true);
    if (millis() - telemetryTimer > TELEMETRY_INTERVAL) {
        telemetryTimer = millis();
        sendTelemetry();
    }
    if (millis() - displayUpdateTimer > DISPLAY_UPDATE_INTERVAL) {
        displayUpdateTimer = millis();
        statusDisplay->updateDisplay();
    }
}