/* ---------------------------------------------------
 * Robot Slave sketch running on Teensy 3.2
 * monitors battery voltage, current consumption and charging,
 * communicates this to Master Raspberry Pi over Serial connection.
 * 
 * boredman@boredomprojects.net
 * 
 * rev 3.5 - 2017.03.22
 *    - implemented proportional motor control loop
 *
 * rev 3.4 - 2016.12.17
 *    - ODOM message publishing working
 *    - nav_msgs::Odometry is to large,
 *      increasing buffer size in ros.h does not help,
 *      thereofore using geometry_msgs::InertiaStamped instead.
 *      Must implement translation service on Raspberry side.
 *      
 * rev 3.3 - 2016.12.11
 *    - TF transform broadcast working
 *    - position calculation using optical wheel encoders
 *    - orientation calculation using IMU data
 * 
 * rev 3.2 - 2016.10.31
 *    - implemented interface to BNO055 IMU sensor
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
#include <Adafruit_BNO055_t3.h>
#include <EEPROM.h>

// Teensy analog comparator library
#include "analogComp.h"

// Teensy ADC library
#include "ADC.h"

#define USE_TEENSY_HW_SERIAL
#include <ros.h>
#include <sensor_msgs/BatteryState.h>  // for POWER_SUPPLY_STATUS_ constants

#include "base_serial.h"

// instance of serial driver
Base_Serial uart(Serial1);


using namespace sensor_msgs;

const int digPin_LED = 13;
const int digPin_SHD = 6;

// power supply pins
const int anlgPin_IBat = A1;
const int anlgPin_VBat = A2;
const int anlgPin_VS   = A0;
const int compPin_Stat = 11;

// motor drive pins (PWM must be on pins 9 and 10)
const int pin_motor_pwm_left  = 10;  // M2-PWM
const int pin_motor_pwm_right =  9;  // M1-PWM
const int pin_motor_dir_left  =  8;  // M2-DIR
const int pin_motor_dir_right =  7;  // M1-DIR

// encoder pins
const int pin_enc_fr_left = 22;   // PTC1
const int pin_enc_bk_left = 20;   // PTD5
const int pin_enc_fr_right = 23;  // PTC2
const int pin_enc_bk_right = 21;  // PTD6

// direct pin interrupt implementation did not work,
// because of some conflict with PWM.
// Therefore, implemented periodic interrupt which samples
// all four pins and updates counters on pin change.
IntervalTimer encoderTimer;

#define VCC (3.31f)

#define BAT_N_CELLS    6
#define BAT_CELL_VMAX  1.4
#define BAT_CELL_VMIN  0.9

// linear speed in [m/s] corresponding to max pwm
#define TOP_SPEED_M_S  0.7

// distance between wheels in [m]
#define WHEEL_BASE_M   0.144
#define WHEEL_DIAM_MM  63.0
#define ENCODER_STEPS  40

// velocity vector in real units
double cmd_vel_linear_x = 0.0;
double cmd_vel_angular_z = 0.0;
double vx_m_s = 0.0;
double vth_rad_s = 0.0;

// motors & encoders
int16_t pwm_right, pwm_left;
bool dir_right, dir_left;
volatile unsigned long enc_cntr_fr_left, enc_cntr_fr_right;
volatile unsigned long enc_cntr_bk_left, enc_cntr_bk_right;

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

  // Initialise IMU sensor
  if( ! bno.begin() )
  {
//    nh.logerror("[TNSY] no IMU detected -> STOP");
//    while(1)
//      nh.spinOnce();
    while(1) ;
  }

  int eeAddress = 0;
  long imuID;

  EEPROM.get(eeAddress, imuID);

  if( imuID != IMU_SENSOR_ID )
  {
//    nh.logerror("[TNSY] no IMU calibration data in EEPROM -> STOP");
//    while(1)
//      nh.spinOnce();
    while(1) ;
  }

  adafruit_bno055_offsets_t calibrationData;
  eeAddress += sizeof(long);
  EEPROM.get(eeAddress, calibrationData);
  bno.setSensorOffsets(calibrationData);
  delay(1000);

  uint8_t system_status, self_test_results, system_error;
  bno.getSystemStatus(&system_status, &self_test_results, &system_error);
//  sprintf(str,"[TNSY] IMU status:%d self-test:%d error:%d",system_status,self_test_results,system_error);
//  nh.loginfo(str);
  
  bno.setExtCrystalUse(true);

//  nh.loginfo("[TNSY] move IMU slightly to calibrate magnetometers");

  uint8_t system, gyro, accel, mag;
  do {
//    nh.spinOnce();
    digitalWrite(digPin_LED, HIGH);
    bno.getCalibration(&system, &gyro, &accel, &mag);
//    sprintf(str,"[TNSY] S:%d G:%d A:%d M:%d",system,gyro,accel,mag);
//    nh.loginfo(str);
    delay(450);
    digitalWrite(digPin_LED, LOW);
    delay(50);
  } while( system < 3 || gyro < 3 || accel < 3 || mag < 3 );  // !bno.isFullyCalibrated()
  digitalWrite(digPin_LED, LOW);

  pinMode(pin_enc_fr_left, INPUT_PULLUP);
  pinMode(pin_enc_bk_left, INPUT_PULLUP);
  pinMode(pin_enc_fr_right, INPUT_PULLUP);
  pinMode(pin_enc_bk_right, INPUT_PULLUP);

  // periodic interrupt servicing wheel encoders
  // min encoder half-period at max speed (0.7m/s) is 7ms
  encoderTimer.begin(EncoderService, 1000);

//  nh.loginfo("[TNSY] Init done");

//  Serial.begin(57600);

  uart.serial().begin(115200);
  if( ! uart.init() )
    while(1);
//  Serial.print("Starting...");
}



/*******************************************************
 * LOOP function runs over and over again forever
 *******************************************************/
void loop() 
{
  static unsigned long time_prev_adc, time_prev_imu;
  unsigned long time_now = millis();
  static float bat_percentage;

  if( time_now - time_prev_imu > IMU_SAMPLERATE_DELAY_MS )
  {
    unsigned long dt_ms = (time_now - time_prev_imu);
    time_prev_imu = time_now;

    imu::Quaternion quat = bno.getQuat();
    imu::Vector<3> euler = quat.toEuler();
    double theta = euler.x();

//    // get linear acceleration in "base_link" frame
//    imu::Vector<3> accel = bno.getVector(Adafruit_BNO055::VECTOR_LINEARACCEL);
//    static double ax;
//    ax = accel.x();
//    //vy += accel.y() * dt;
//    vy = 0.0;
//    if( pwm_speed == 0 )
//      vx = 0;
//    else
//      vx += ax * dt;
//
//    // calculate position while translating to "odom" frame
//    x += (vy * sin(theta) + vx * cos(theta)) * dt;
//    y += (vy * cos(theta) - vx * sin(theta)) * dt;

    noInterrupts();
    long cntr_fr_left  = (long)enc_cntr_fr_left;
    long cntr_bk_left  = (long)enc_cntr_bk_left;
    long cntr_fr_right = (long)enc_cntr_fr_right;
    long cntr_bk_right = (long)enc_cntr_bk_right;
    interrupts();

    static long last_fr_left, last_fr_right;
    static long last_bk_left, last_bk_right;

    long dif_fr_left  = cntr_fr_left  - last_fr_left;
    long dif_bk_left  = cntr_bk_left  - last_bk_left;
    long dif_fr_right = cntr_fr_right - last_fr_right;
    long dif_bk_right = cntr_bk_right - last_bk_right;

    last_fr_left = cntr_fr_left;
    last_bk_left = cntr_bk_left;
    last_fr_right = cntr_fr_right;
    last_bk_right = cntr_bk_right;
    
    double dif_left  = (dif_fr_left + dif_bk_left) / 2.0;
    double dif_right = (dif_fr_right + dif_bk_right) / 2.0;

    double dx_mm = (dif_right + dif_left) / 2.0 * PI * WHEEL_DIAM_MM / ENCODER_STEPS;
//  double dth_mrad = (dif_right - dif_left) / WHEEL_BASE_M * PI * WHEEL_DIAM_MM / ENCODER_STEPS;

    static double last_theta;
    double dth = theta - last_theta;
    if( dth > PI )
      dth = dth - 2*PI;
    if( dth < -PI )
      dth = dth + 2*PI;
    last_theta = theta;

    uart.odom.theta = theta;
    uart.odom.dx_mm = dx_mm;
    uart.odom.dth = dth;
    uart.odom.dt_ms = dt_ms;
    uart.sendOdom();

    vx_m_s = dx_mm / dt_ms;
    vth_rad_s = dth * 1000.0 / dt_ms;

    // motor control loop
    UpdateDrive();
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

    // scale results into milli(binary)-volts or -amperes
    // so, no need to divide by ADC resolution (i.e. 1023)
    uart.power.battery_miA = (int16_t)(fI * VCC / 0.22);
    uart.power.battery_miV = (uint16_t)(fV * VCC);
    uart.power.vsupply_miV = (uint16_t)(fVS * VCC); // use for VS
    uart.power.charger_state = ChargingState();
    bat_percentage = BatteryPercentage(fV * VCC / adc->getMaxValue(ADC_1));
    uart.power.bat_percentage = (uint8_t)(bat_percentage * 100);
    uart.sendPower();
  }

  if( uart.recvDrive() )
  {
    cmd_vel_linear_x = (double)uart.drive.speed_mm_s / 1000.0;
    cmd_vel_angular_z = (double)uart.drive.turn_mrad_s / 1000.0;
  }

  UpdateLED(bat_percentage);
}



#define pwm_min 1
#define cntr_loop_rate 50.0
/*******************************************************
 * UpdateDrive()
 *   uses global variables pwm_speed and pwm_turn
 *   to calculate and set PWM values that drive motors.
 *******************************************************/
void UpdateDrive(void)
{
  static int pwm_speed, pwm_turn;

  if( cmd_vel_linear_x == 0.0 )
    pwm_speed = 0;
  else
    pwm_speed += cntr_loop_rate * (cmd_vel_linear_x - vx_m_s);

  if( cmd_vel_angular_z == 0.0 )
    pwm_turn = 0;
  else
    pwm_turn += cntr_loop_rate * (cmd_vel_angular_z - vth_rad_s);

  pwm_right = pwm_speed + pwm_turn;
  if( pwm_right < 0 )
  {
    pwm_right = -pwm_right;
    dir_right = HIGH;
  }
  else if( pwm_right > 0 )
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
  else if( pwm_left > 0 )
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
 *   a constant as defined in sensor_msgs::BatteryState
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


static void EncoderService(void)
{
  static bool last_fr_left, last_bk_left, last_fr_right, last_bk_right;

  bool pin_fr_left  = digitalRead(pin_enc_fr_left);
  bool pin_bk_left  = digitalRead(pin_enc_bk_left);
  bool pin_fr_right = digitalRead(pin_enc_fr_right);
  bool pin_bk_right = digitalRead(pin_enc_bk_right);
  
  if( pin_fr_left != last_fr_left )
  {
    last_fr_left = pin_fr_left;
    if( dir_left )
      enc_cntr_fr_left--;
    else
      enc_cntr_fr_left++;
  }
  if( pin_bk_left != last_bk_left )
  {
    last_bk_left = pin_bk_left;
    if( dir_left )
      enc_cntr_bk_left--;
    else
      enc_cntr_bk_left++;
  }
  if( pin_fr_right != last_fr_right )
  {
    last_fr_right = pin_fr_right;
    if( dir_right )
      enc_cntr_fr_right--;
    else
      enc_cntr_fr_right++;
  }
  if( pin_bk_right != last_bk_right )
  {
    last_bk_right = pin_bk_right;
    if( dir_right )
      enc_cntr_bk_right--;
    else
      enc_cntr_bk_right++;
  }
}



