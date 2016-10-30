/* ---------------------------------------------------
 * Robot Slave sketch running on Teensy 3.2
 * monitors battery voltage, current consumption and charging,
 * communicates this to Master Raspberry Pi over Serial connection.
 * 
 * boredman@boredomprojects.net
 * 
 * rev 3.2 - 2016.10.31
 *    - added /tf orientation transform broadcasting
 * 
 * rev 3.1 - 2016.10.16
 *    - using analogComp library for battery charging status measurements
 *        Teensy support from: https://github.com/orangkucing/analogComp
 *        added comparator polling from: https://github.com/rlagerweij/analogComp
 *        added 6-bit dac as reference
 *    - using ADC library from: https://github.com/pedvide/ADC
 *        background synchronous V-I measurements
 *        hardware averaging
 *      
 * rev 3.0 - 2016.10.10
 *    - use Teensy3.2 instead of Pro Mini
 *    - modified hardware and software for VCC=3.3V
 *    * inaccurate ADC measurements -> should change sampling speed
 * 
 * rev 2.3 - 2016.07.16
 *    - put back LED battery indicator
 *
 * rev 2.2 - 2016.07.16
 *    - adapted for use with ROS through 'rosserial' package
 *    - implemented messages:
 *        geometry_msgs/Twist (cmd_vel)
 *        sensor_msgs/BatteryState
 *    
 * rev 2.1 - 2016.07.11
 *    - implemented packet-based uart communication
 *      using RH_Serial driver from RadioHead:
 *      http://www.airspayce.com/mikem/arduino/RadioHead/classRH__Serial.html
 *    
 * rev 2.0 - 2016.07.10
 *    - communication over UART (instead of I2C)
 *    
 * rev 1.0 - 2016.05.18
 *    - initial version
 * ---------------------------------------------------
 */
#include <i2c_t3.h>
#include <Adafruit_BNO055_t3.h>
#include <EEPROM.h>

// Teensy analog comparator library
#include "analogComp.h"

// Teensy ADC library
#include "ADC.h"

#define USE_TEENSY_HW_SERIAL
#include <ros.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/BatteryState.h>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>

using namespace sensor_msgs;

const int digPin_LED = 13;
const int digPin_SHD = 6;

// power supply pins
const int anlgPin_IBat = A1;
const int anlgPin_VBat = A2;
const int anlgPin_VS   = A3;
const int compPin_Stat = 11;

// motor drive pins (PWM must be on pins 9 and 10)
const int pin_motor_pwm_left  = 10;  // M2-PWM
const int pin_motor_pwm_right =  9;  // M1-PWM
const int pin_motor_dir_left  =  8;  // M2-DIR
const int pin_motor_dir_right =  7;  // M1-DIR

#define VCC (3.31f)

#define BAT_N_CELLS    6
#define BAT_CELL_VMAX  1.4
#define BAT_CELL_VMIN  0.9

// linear speed in [m/s] corresponding to max pwm
#define top_speed_m_s  0.7
// half the distance between wheels in [m]
#define turn_radius_m  0.072

// velocity vector in units of [pwm counts]
int pwm_speed = 0;
int pwm_turn = 0;

bool got_new_vel = false;

void callback_cmd_vel(const geometry_msgs::Twist& cmd_vel)
{
  // top_speed in m/s corresponds to pwm count of 255
  pwm_speed = (int)(cmd_vel.linear.x * 255 / top_speed_m_s);
  // 1 rad/s at stand still corresponds to +/-(distance between wheels / 2)
  pwm_turn  = (int)(cmd_vel.angular.z * turn_radius_m * 255 / top_speed_m_s);
  got_new_vel = true;
}

ros::NodeHandle nh;

ros::Subscriber<geometry_msgs::Twist> sub_vel("cherokey/cmd_vel", &callback_cmd_vel );

BatteryState bat_msg;
ros::Publisher pub_bat("cherokey/battery", &bat_msg);

geometry_msgs::TransformStamped t;
char base_link[] = "/base_link";
char odom[] = "/odom";
double x = 0.0;
double y = 0.0;
double theta = 0.0;

tf::TransformBroadcaster odom_broadcaster;


#define ADC_SAMPLERATE_DELAY_MS (1000)
ADC *adc = new ADC(); // adc object
ADC::Sync_result adcRes;


// IMU sensor interface (I2C bus)
// https://forums.adafruit.com/viewtopic.php?f=19&t=92153
#define IMU_SENSOR_ID (55)
#define IMU_SAMPLERATE_DELAY_MS (50)
Adafruit_BNO055 bno = Adafruit_BNO055(WIRE_BUS, IMU_SENSOR_ID, BNO055_ADDRESS_A, 
                                      I2C_MASTER, I2C_PINS_18_19, I2C_PULLUP_INT, 
                                      I2C_RATE_1000, I2C_OP_MODE_ISR);

char str[100];

/*******************************************************
 * SETUP function runs once on power-up or reset
 *******************************************************/
void setup() 
{
  pinMode(digPin_LED, OUTPUT);
  digitalWrite(digPin_LED, LOW);

  pinMode(digPin_SHD, OUTPUT);
  digitalWrite(digPin_SHD, LOW);

  pinMode(pin_motor_pwm_right, OUTPUT);
  digitalWrite(pin_motor_pwm_right, LOW);
  pinMode(pin_motor_dir_right, OUTPUT);
  digitalWrite(pin_motor_dir_right, LOW);
  pinMode(pin_motor_pwm_left, OUTPUT);
  digitalWrite(pin_motor_pwm_left, LOW);
  pinMode(pin_motor_dir_left, OUTPUT);
  digitalWrite(pin_motor_dir_left, LOW);

  // Reduce PWM frequency to 50Hz or so
  // to reduce buzzing sounds from motors
  analogWriteFrequency(pin_motor_pwm_right, 50);

  // configure analog comparator for battery charging state
  analogComparator.setOn(0, 7);   // 0 = compPin_Stat (pin 11),  7 = 6bit DAC (ref)

  // configure ADC for V, I and VS measurements
  adc->setAveraging(32);
  adc->setResolution(10);
  adc->setConversionSpeed(ADC_VERY_LOW_SPEED);
  adc->setSamplingSpeed(ADC_VERY_LOW_SPEED);
  adc->setAveraging(32, ADC_1);
  adc->setResolution(10, ADC_1);
  adc->setConversionSpeed(ADC_VERY_LOW_SPEED, ADC_1);
  adc->setSamplingSpeed(ADC_VERY_LOW_SPEED, ADC_1);
  adc->startSynchronizedContinuous(anlgPin_IBat, anlgPin_VBat);

  // populate some constant values
  bat_msg.design_capacity = 2500.0;
  bat_msg.power_supply_technology = BatteryState::POWER_SUPPLY_TECHNOLOGY_NIMH;
  bat_msg.present = true;

  nh.initNode();

  nh.advertise(pub_bat);
  nh.subscribe(sub_vel);

  odom_broadcaster.init(nh);

  // Initialise IMU sensor
  if( ! bno.begin() )
  {
    nh.logerror("[TNSY] no IMU detected -> STOP");
    while(1)
      nh.spinOnce();
  }

  int eeAddress = 0;
  long imuID;

  EEPROM.get(eeAddress, imuID);

  if( imuID != IMU_SENSOR_ID )
  {
    nh.logerror("[TNSY] no IMU calibration data in EEPROM -> STOP");
    while(1)
      nh.spinOnce();
  }

  adafruit_bno055_offsets_t calibrationData;
  eeAddress += sizeof(long);
  EEPROM.get(eeAddress, calibrationData);
  bno.setSensorOffsets(calibrationData);
  delay(1000);

  uint8_t system_status, self_test_results, system_error;
  bno.getSystemStatus(&system_status, &self_test_results, &system_error);
  sprintf(str,"[TNSY] IMU status:%d self-test:%d error:%d",system_status,self_test_results,system_error);
  nh.loginfo(str);
  
  bno.setExtCrystalUse(true);

  nh.loginfo("[TNSY] move IMU slightly to calibrate magnetometers");

  uint8_t system, gyro, accel, mag;
  do {
    nh.spinOnce();
    digitalWrite(digPin_LED, HIGH);
    bno.getCalibration(&system, &gyro, &accel, &mag);
    sprintf(str,"[TNSY] S:%d G:%d A:%d M:%d",system,gyro,accel,mag);
    nh.loginfo(str);
    delay(450);
    digitalWrite(digPin_LED, LOW);
    delay(50);
  } while( system < 3 || gyro < 3 || accel < 3 || mag < 3 );  // !bno.isFullyCalibrated()
  digitalWrite(digPin_LED, LOW);

  nh.loginfo("[TNSY] Init done");
}


/*******************************************************
 * LOOP function runs over and over again forever
 *******************************************************/
void loop() 
{
//  static unsigned long time_prev;
//  unsigned long time_now = millis();
//  if( time_now - time_prev > IMU_SAMPLERATE_DELAY_MS )
//  {
//    time_prev = time_now;
//    
//    // Possible vector values can be:
//    // - VECTOR_ACCELEROMETER - m/s^2
//    // - VECTOR_MAGNETOMETER  - uT
//    // - VECTOR_GYROSCOPE     - rad/s
//    // - VECTOR_EULER         - degrees
//    // - VECTOR_LINEARACCEL   - m/s^2
//    // - VECTOR_GRAVITY       - m/s^2
//    bno::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
//  
//    /* Display the floating point data */
//    Serial.print("X: ");
//    Serial.print(euler.x());
//    Serial.print(" Y: ");
//    Serial.print(euler.y());
//    Serial.print(" Z: ");
//    Serial.print(euler.z());
//    Serial.print("  \t");
//  
//    euler = bno.getVector(Adafruit_BNO055::VECTOR_LINEARACCEL);
//  
//    /* Display the floating point data */
//    Serial.print("X: ");
//    Serial.print(euler.x());
//    Serial.print(" Y: ");
//    Serial.print(euler.y());
//    Serial.print(" Z: ");
//    Serial.print(euler.z());
//    Serial.print("  \t");
//  
//    /*
//    // Quaternion data
//    bno::Quaternion quat = bno.getQuat();
//    Serial.print("qW: ");
//    Serial.print(quat.w(), 4);
//    Serial.print(" qX: ");
//    Serial.print(quat.y(), 4);
//    Serial.print(" qY: ");
//    Serial.print(quat.x(), 4);
//    Serial.print(" qZ: ");
//    Serial.print(quat.z(), 4);
//    Serial.print("\t\t");
//    */
//  
//    /* Display calibration status for each sensor. */
//    uint8_t system, gyro, accel, mag = 0;
//    bno.getCalibration(&system, &gyro, &accel, &mag);
//    Serial.print("CALIBRATION: Sys=");
//    Serial.print(system, DEC);
//    Serial.print(" Gyro=");
//    Serial.print(gyro, DEC);
//    Serial.print(" Accel=");
//    Serial.print(accel, DEC);
//    Serial.print(" Mag=");
//    Serial.println(mag, DEC);
//
//  }

  static unsigned long time_prev_adc, time_prev_imu;
  unsigned long time_now = millis();

  if( time_now - time_prev_imu > IMU_SAMPLERATE_DELAY_MS )
  {
    time_prev_imu = time_now;

//    imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
    imu::Quaternion quat = bno.getQuat();
    imu::Vector<3> euler = quat.toEuler();
    theta = euler.x();
    
//    double dx = 0.0;
//    double dtheta = 0.0;
//    x += cos(theta)*dx*0.1;
//    y += sin(theta)*dx*0.1;
//    theta += dtheta*0.1;
//    if(theta > 3.14)
//      theta=-3.14;

    // tf odom->base_link
    t.header.frame_id = odom;
    t.child_frame_id = base_link;
  
    t.transform.translation.x = x;
    t.transform.translation.y = y;
  
    geometry_msgs::Quaternion Quat = geometry_msgs::Quaternion();
    Quat.x = quat.x();
    Quat.y = quat.y();
    Quat.z = quat.z();
    Quat.w = quat.w();
    t.transform.rotation = Quat;    // = tf::createQuaternionFromYaw(theta);

    t.header.stamp = nh.now();
  
    odom_broadcaster.sendTransform(t);
    nh.spinOnce();
  }


  if( time_now - time_prev_adc > ADC_SAMPLERATE_DELAY_MS )
  {
    time_prev_adc = time_now;

    // get results from most recent V-I measurements
    adcRes = adc->readSynchronizedContinuous();

    // negative voltage across current sensing resistor R11 (0.22 ohm)
    // is measured with a resistor divider 100K/(100K||100K) to (VCC||GND)
    float fI = (float)adcRes.result_adc0 * 3.0 - adc->getMaxValue(ADC_0);
    // the above subtracted from voltage measurement to get true battery voltage,
    // while factoring in its resistor divider (100K/50K)
    float fV = (float)adcRes.result_adc1 * 3.0 - fI;

    // interrupt continuous V-I and do one-shot VS measurement
    float fVS = analogRead(anlgPin_VS) * (39+39+4.7)/4.7;
    // restart continuous V-I measurements
    adc->startSynchronizedContinuous(anlgPin_IBat, anlgPin_VBat);

    bat_msg.current = (fI * VCC / 0.22) / adc->getMaxValue(ADC_0);
    bat_msg.voltage = (fV * VCC) / adc->getMaxValue(ADC_1);
    bat_msg.charge = (fVS * VCC) / adc->getMaxValue(ADC_0); // use for VS
    bat_msg.power_supply_status = ChargingState();
    bat_msg.percentage = BatteryPercentage(bat_msg.voltage);

    pub_bat.publish(&bat_msg);
  }

  nh.spinOnce();

  if( got_new_vel )
  {
    UpdateDrive();
    got_new_vel = false;
  }

  UpdateLED(bat_msg.percentage);
}



/*******************************************************
 * UpdateDrive()
 *   uses global variables pwm_speed and pwm_turn
 *   to calculate and set PWM values that drive motors.
 *******************************************************/
#define pwm_min 10
void UpdateDrive(void)
{
  int16_t pwm_right, pwm_left;
  int8_t  dir_right, dir_left;

  pwm_right = pwm_speed + pwm_turn;
  if( pwm_right < 0 )
  {
    pwm_right = -pwm_right;
    dir_right = HIGH;
  }
  else
    dir_right = LOW;

  if( pwm_right > 0 && pwm_right < pwm_min )
    pwm_right = pwm_min;
  if( pwm_right > 255 )
    pwm_right = 255;
  
  pwm_left = pwm_speed - pwm_turn;
  if( pwm_left < 0 )
  {
    pwm_left = -pwm_left;
    dir_left = HIGH;
  }
  else
    dir_left = LOW;

  if( pwm_left > 0 && pwm_left < pwm_min )
    pwm_left = pwm_min;
  if( pwm_left > 255 )
    pwm_left = 255;

  digitalWrite(pin_motor_dir_right, dir_right);
  analogWrite(pin_motor_pwm_right, pwm_right);
  digitalWrite(pin_motor_dir_left, dir_left);
  analogWrite(pin_motor_pwm_left, pwm_left);
}


/*******************************************************
 * BatteryPercentage()
 *   calculates remaining battery charge 
 *   as a value between 0.0 and 1.0
 *******************************************************/
float BatteryPercentage(float vbat)
{
  float percentage = (vbat/BAT_N_CELLS - BAT_CELL_VMIN) / (BAT_CELL_VMAX - BAT_CELL_VMIN);
  if( percentage < 0.0 )
    return 0.0;  
  else if( percentage > 1.0 )
    return 1.0;
  else
    return percentage;
}


/*******************************************************
 * UpdateLED()
 *  controls state of LED - indicator of battery voltage
 *  according to percentage:
 *  - ON time is always 100ms
 *  - OFF time varies between 100ms and 5.1sec proportionally to VBat
 *******************************************************/
void UpdateLED(float percentage)
{
  static byte state;
  static unsigned long last_time;

  if( state == LOW && (millis() - last_time) > (100 + percentage * 5000.0) )
  {
    digitalWrite(digPin_LED, HIGH);
    state = HIGH;
    last_time = millis();
  }
  else if( state == HIGH && (millis() - last_time) > 100 )
  {
    digitalWrite(digPin_LED, LOW);
    state = LOW;
    last_time = millis();
  }

}


/*******************************************************
 * ChargingState()
 *  calculates charging state based on voltage at 
 *  dual-color LED of Graupner 6425 charger.
 *  uses analog comparator of Teensy
 * returns: 
 *   a constant as defined by BatteryState::POWER_SUPPLY_STATUS_...
 *******************************************************/
uint8_t ChargingState()
{
  const int thrd1 = 4;    //0.2 V;
  const int thrd2 = 15;   //0.8 V;
  const int thrd3 = 36;   //1.9 V;

  analogComparator.configureDac(thrd2);
  delay(10);
  if( analogComparator.getOutput() )
  {
    analogComparator.configureDac(thrd3);
    delay(10);
    if( analogComparator.getOutput() )
      return BatteryState::POWER_SUPPLY_STATUS_FULL;
    else
      return BatteryState::POWER_SUPPLY_STATUS_NOT_CHARGING;
  }
  else
  {
    analogComparator.configureDac(thrd1);
    delay(10);
    if( analogComparator.getOutput() )
      return BatteryState::POWER_SUPPLY_STATUS_DISCHARGING;
    else
      return BatteryState::POWER_SUPPLY_STATUS_CHARGING;
  }
}


