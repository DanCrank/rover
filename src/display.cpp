/******************************************************************
 * display management
 ******************************************************************/

#include "rover.h"
#include <algorithm>
#include <iterator>
#include <sstream>
#include <vector>
#include <U8g2lib.h>

// Singleton instance of the display driver
// https://github.com/olikraus/u8g2/wiki/u8g2setupcpp
U8G2_SH1107_64X128_F_HW_I2C u8g2(/* rotation=*/ U8G2_R2, /* reset=*/ U8X8_PIN_NONE, /* clock=*/ 21, /* data=*/ 22);

// Singleton instance of status data
StatusDisplay *statusDisplay;

void setupDisplay() {
    // initialize OLED display
    u8g2.begin();
    u8g2.setFont(u8g2_font_unifont_tf);
    statusDisplay = new StatusDisplay();
}

StatusDisplay::StatusDisplay() {
    lidarOK = false;
    gpsSats = 0;
    rssi = 0;
    defaultDisplay = true;
}

void StatusDisplay::setLidarOK(bool newLidarOK) {
    lidarOK.store(newLidarOK);
}

void StatusDisplay::setGpsSats(uint16_t newGpsSats) {
    gpsSats.store(newGpsSats);
}

void StatusDisplay::setRssi(int16_t newRssi) {
    rssi.store(newRssi);
}

void StatusDisplay::setObstruction(int16_t newObstruction) {
    obstruction.store(newObstruction);
}

// display a non-default message on the OLED display
void StatusDisplay::display(const std::string& str) {
    defaultDisplay = false;
    std::istringstream iss(str);
    std::vector<std::string> vec {std::istream_iterator<std::string>(iss), std::istream_iterator<std::string>()};
    u8g2_uint_t y = 12;
    u8g2.clearBuffer();
    for (std::vector<std::string>::iterator it = vec.begin();
         (it != vec.end() && y <= 80); ++it) {
        u8g2.drawStr(0, y, it->c_str());
        y += 12;
    }
    u8g2.sendBuffer();
}

void StatusDisplay::returnToDefaultDisplay() {
    defaultDisplay = true;
}

// update the default display
void StatusDisplay::updateDisplay() {
    if (defaultDisplay) {
        // assemble display contents into buffer
        if (lidarOK.load())
            sprintf(displayBuf[0], "LIDAR:OK");
        else
            sprintf(displayBuf[0], "LIDAR:NO");
        sprintf(displayBuf[1], "SAT:%d", gpsSats.load());
        sprintf(displayBuf[2], "RCV:%d", rssi.load());
        int16_t obs = obstruction.load();
        if (obs >= 0) {
            sprintf(displayBuf[3], "OBS:%d", obs);
        } else {
            sprintf(displayBuf[3], "OBS:NO");
        }
        // print them to the display
        u8g2.clearBuffer();
        for (unsigned short i = 0; i < 4; i++) {
            u8g2.drawStr(0, (i + 1) * 12, displayBuf[i]);
        }
        u8g2.sendBuffer();
    }
}
