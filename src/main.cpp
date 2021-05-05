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
 ******************************************************************/

#include <SPI.h>
#include <RH_RF69.h>
#include <U8g2lib.h>

//#define USB_DEBUG

// Singleton instance of the radio driver
RH_RF69 rf69(11, 19);

//https://github.com/olikraus/u8g2/wiki/u8g2setupcpp
U8G2_SH1107_64X128_2_HW_I2C u8g2(/* rotation=*/ U8G2_R3, /* reset=*/ U8X8_PIN_NONE, /* clock=*/ 21, /* data=*/ 22);

void debug(String str)
{
#ifdef USB_DEBUG
    String t = "00000000" + String(millis());
    t.remove(0, t.length() - 8);
    Serial.println(t + " | " + str);
    Serial.flush();
#endif
}

void display(const String& str) {
    u8g2.firstPage();
    do {
        u8g2.setCursor(0, 20);
        u8g2.print(str);
    } while ( u8g2.nextPage() );
}

void setup() {
#ifdef USB_DEBUG
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
    // Defaults after init are 434.0MHz, modulation GFSK_Rb250Fd250, +13dbM (for low power module)
    // No encryption
    if (!rf69.setFrequency(868.0))
        display("setFrequency failed");
    // standard config: FSK, Whitening, Rb = 9.6kbs, Fd = 19.2kHz
    if (!rf69.setModemConfig(rf69.FSK_Rb9_6Fd19_2))
        display("setModemConfig failed");
    // If you are using a high power RF69 eg RFM69HW, you *must* set a Tx power with the
    // ishighpowermodule flag set like this:
    rf69.setTxPower(18, true);
    // The encryption key has to be the same as the one in the server
//    uint8_t key[] = { 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08,
//                      0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08};
//    rf69.setEncryptionKey(key);
    uint version = rf69.deviceType();
    display(String("RFM69: 0x" + String(version, HEX)));
    delay(2000);
}

void loop() {
    for (int i = 1; i < 1000000; i++) {
        // generate a fake data packet
        String msg = "Tom Servo to Ground Control..." + String(i);
        uint8_t data[msg.length() + 1]; //remember the trailing null
        msg.getBytes(data, sizeof(data) + 1);
        // send a packet
        rf69.send(data, sizeof(data));
        rf69.waitPacketSent();
        display(String(sizeof(data)) + " bytes sent (" + String(i) + ")");
        delay(1000);
    }
}