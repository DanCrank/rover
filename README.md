Prototype rover code
This code targets the following hardware...

Adafruit Feather M4 Express board:
https://learn.adafruit.com/adafruit-feather-m4-express-atsamd51

Adafruit 128x64 OLED display Featherwing:
https://learn.adafruit.com/adafruit-128x64-oled-featherwing

Adafruit Radio FeatherWing - RFM69HCW 900MHz
https://learn.adafruit.com/radio-featherwing

Adafruit DC Motor + Stepper FeatherWing
https://learn.adafruit.com/adafruit-stepper-dc-motor-featherwing

Adafruit 9-DoF IMU ISM330DHCX + LIS3MDL Featherwing
https://learn.adafruit.com/st-9-dof-combo
https://www.st.com/resource/en/datasheet/lis3mdl.pdf

Adafruit Ultimate GPS Featherwing
https://learn.adafruit.com/adafruit-ultimate-gps-featherwing

Adafruit Adalogger Featherwing
https://learn.adafruit.com/adafruit-adalogger-featherwing

Adafruit INA219 Power Monitoring Featherwing
https://learn.adafruit.com/adafruit-ina219-current-sensor-breakout

Adafruit BME680 temperature / humidity / pressure / gas sensor
https://learn.adafruit.com/adafruit-bme680-humidity-temperature-barometic-pressure-voc-gas/

Slamtec RoboPeak RPLIDAR
https://www.adafruit.com/product/4010
https://cdn-shop.adafruit.com/product-files/4010/4010_datasheet.pdf
http://www.slamtec.com/en/Support#rplidar-a1
http://www.robopeak.net/data/doc/rplidar/appnote/RPLDAPPN01-rplidar_appnote_arduinolib-enUS.pdf
https://github.com/robopeak/rplidar_arduino
See also: https://github.com/DanCrank/rplidar_arduino/tree/begin-returns-void

Vehicle hardware is basically the same as the Mk I rover, but with an improved battery setup.
DFRobot Devastator tank chassis (6V metal gear motor version):
https://www.dfrobot.com/product-1477.html

Inside the chassis is a 1S LiPo battery to run the electronics and a 2S radio
control car battery to drive the motors. There is a buck converter wired to
the big battery to drive the LIDAR, which requires 5V.

Camera(s) TBD

Other sensors TBD
