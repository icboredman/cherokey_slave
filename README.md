# cherokey_slave
Arduino sketch running on a Teensy 3.2 - a slave microcontroller in a mobile robotic platform. The Teensy implements PID motor control, delivers odometry, monitors battery voltage and current consumption and manages charging. 
Communication to the master Raspberry Pi is implemented over a serial connection using packet-based [MessageSerial](https://github.com/icboredman/PacketSerial/tree/MessageSerial) protocol. 

More info in my blog post: https://BoredomProjects.net/index.php/projects/robot-navigation-using-stereo-vision-part-2

---
### Used Libraries
*Adafruit_BNO055_t3.h* from here: [BNO055 library adapted to i2c_t3 lib for Teensy](https://forums.adafruit.com/viewtopic.php?f=19&t=92153)

in addition:
[ADC for Teensy by Pedro Villanueva](https://github.com/pedvide/ADC)
[Adafruit Unified BNO055 Driver](https://github.com/adafruit/Adafruit_BNO055)
[Arduino PID Library by Brett Beauregard](https://github.com/br3ttb/Arduino-PID-Library)
[Fast CRC Arduino library by FrankBoesing](https://github.com/FrankBoesing/FastCRC)
[MessageSerial library, forked from PacketSerial by Christopher Baker](https://github.com/icboredman/PacketSerial)
[Analog Comparator library, forked from orangkucing](https://github.com/icboredman/analogComp)

also:
[Enhanced I2C library for Teensy](https://github.com/nox771/i2c_t3.git)
[Adafruit Unified Sensor Driver](https://github.com/adafruit/Adafruit_Sensor.git)
