# cherokey_slave
Arduino sketch running on a Teensy 3.2 - a slave microcontroller in a mobile robotic platform. The Teensy implements PID motor control, delivers odometry, monitors battery voltage and current consumption and manages charging. 
Communication to the master Raspberry Pi is implemented over a serial connection using packet-based [MessageSerial](https://github.com/icboredman/PacketSerial/tree/MessageSerial) protocol. 

More info in my blog post: https://BoredomProjects.net/index.php/projects/robot-navigation-using-stereo-vision
