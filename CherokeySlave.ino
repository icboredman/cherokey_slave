/* ---------------------------------------------------
 * Robot Slave sketch running on Teensy 3.2
 * monitors battery voltage, current consumption and charging,
 * communicates this to Master Raspberry Pi over Serial connection.
 * 
 * boredman@boredomprojects.net
 * 
 * rev 3.9 - 2019.02.18
 *    - USB communication, without MessageSerial
 * 
 * rev 3.8 - 2017.05.06
 *    - battery charge state using coulomb counting
 *
 * rev 3.7 - 2017.04.25
 *    - PID motor control
 *
 * rev 3.6 - 2017.04.09
 *    - replaced rosserial with MessageSerial
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
#include <analogComp.h>

// Teensy ADC library
#include <ADC.h>

// Arduino PID library
#include <PID_v1.h>

// define actual messages and create corresponding Message objects
typedef struct Power {
  uint16_t battery_miV;
  int16_t  battery_miA;
  uint16_t battery_mOhm;
  uint8_t  charger_state;
  uint8_t  bat_percentage;
} tPower;

typedef struct {
  float theta;
  float dx;
  float dth;
  uint16_t dt_ms;
} tOdom;

typedef struct {
  tPower power;
  tOdom  odom;
} tTxPacket;

typedef struct {
  int16_t speed_mm_s;
  int16_t turn_mrad_s;
} tDrive;


//copied here from sensor_msgs/BatteryState.h
enum { POWER_SUPPLY_STATUS_UNKNOWN =  0 };
enum { POWER_SUPPLY_STATUS_CHARGING =  1 };
enum { POWER_SUPPLY_STATUS_DISCHARGING =  2 };
enum { POWER_SUPPLY_STATUS_NOT_CHARGING =  3 };
enum { POWER_SUPPLY_STATUS_FULL =  4 };

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

#define BAT_N_CELLS    7
#define BAT_CELL_VMAX  1.4
#define BAT_CELL_VMIN  0.9
#define BAT_CAPACITY   (2850.0 * 1024/1000)   //in [miA*h]

// linear speed [m/s] corresponding to max pwm
#define TOP_SPEED_M_S  0.7

// distance between wheels [m]
#define WHEEL_BASE_M   0.144
#define WHEEL_DIAM_M   0.063
#define ENCODER_STEPS  40

// use encoders rather than IMU to calculate dth
//#define D_THETA_USE_ENCODERS

// requested (setpoint) velocity vectors
double cmd_speed = 0.0;
double cmd_turn = 0.0;
#define cmd_speed_min 0.01
#define cmd_turn_min  0.01

// measured (input) velocity vectors
double vx = 0.0;
double vth = 0.0;

// pwm (output) velocity vectors
double pwm_speed = 0.0;
double pwm_turn = 0.0;

// PID control constants
double spdKp = 100;
double spdKi = 2000;
double spdKd = 0;
double trnKp = 100;
double trnKi = 2000;
double trnKd = 0;

// PID objects
PID spdPID(&vx, &pwm_speed, &cmd_speed, spdKp, spdKi, spdKd, DIRECT);
PID trnPID(&vth, &pwm_turn, &cmd_turn, trnKp, trnKi, trnKd, DIRECT);

// motors & encoders
int16_t pwm_right, pwm_left;
bool dir_right, dir_left;
volatile unsigned long enc_cntr_fr_left, enc_cntr_fr_right;
volatile unsigned long enc_cntr_bk_left, enc_cntr_bk_right;

#define ADC_SAMPLERATE_DELAY_MS (250)
ADC *adc = new ADC(); // adc object
// variables used in ADC ISR
volatile float adc_I_avg, adc_V_avg;
volatile int32_t adc_I_min, adc_I_max;
volatile int32_t adc_V_min, adc_V_max;
volatile float adc_I_coulomb;
volatile bool adc_reset_min_max;

// IMU sensor interface (I2C bus)
// https://forums.adafruit.com/viewtopic.php?f=19&t=92153
#define IMU_SENSOR_ID (55)
#define IMU_SAMPLERATE_DELAY_MS (50)
Adafruit_BNO055 bno = Adafruit_BNO055(WIRE_BUS, IMU_SENSOR_ID, BNO055_ADDRESS_A, 
                                      I2C_MASTER, I2C_PINS_18_19, I2C_PULLUP_INT, 
                                      I2C_RATE_1000, I2C_OP_MODE_ISR);


/*******************************************************
 * SETUP function runs once on power-up or reset
 *******************************************************/
void setup() 
{
  // initialize USB serial port
  Serial.begin(115200);

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

  // configure ADC for simultaneous V and I measurements
  adc->setAveraging(32);
  adc->setResolution(10);
  adc->setConversionSpeed(ADC_CONVERSION_SPEED::VERY_LOW_SPEED);
  adc->setSamplingSpeed(ADC_SAMPLING_SPEED::VERY_LOW_SPEED);
  adc->setAveraging(32, ADC_1);
  adc->setResolution(10, ADC_1);
  adc->setConversionSpeed(ADC_CONVERSION_SPEED::VERY_LOW_SPEED, ADC_1);
  adc->setSamplingSpeed(ADC_SAMPLING_SPEED::VERY_LOW_SPEED, ADC_1);
  // the above settings will produce result every ~430 us (at 96MHz cpu)
  // ISR for ADC_0 will also read result of ADC_1
  adc->enableInterrupts(ADC_0);
  adc_reset_min_max = true;
  adc->startSynchronizedContinuous(anlgPin_IBat, anlgPin_VBat);
  adc_I_coulomb = 0.0;

  // Initialise IMU sensor
  if( ! bno.begin() )
  {
//    strncpy(text.data.str, "[TNSY] no IMU detected -> STOP", sizeof(text.data.str));
//    text.send();
    while(1) ;
  }

  int eeAddress = 0;
  long imuID;

  EEPROM.get(eeAddress, imuID);

  if( imuID != IMU_SENSOR_ID )
  {
//    strncpy(text.data.str, "[TNSY] no IMU calibration data in EEPROM -> STOP", sizeof(text.data.str));
//    text.send();
    while(1) ;
  }

  adafruit_bno055_offsets_t calibrationData;
  eeAddress += sizeof(long);
  EEPROM.get(eeAddress, calibrationData);
  bno.setSensorOffsets(calibrationData);
  delay(1000);

  uint8_t system_status, self_test_results, system_error;
  bno.getSystemStatus(&system_status, &self_test_results, &system_error);
//  snprintf(text.data.str, sizeof(text.data.str),
//           "[TNSY] IMU status:%d self-test:%d error:%d",
//           system_status, self_test_results, system_error);
//  text.send();
  
  bno.setExtCrystalUse(true);

//  strncpy(text.data.str, "[TNSY] move IMU slightly to calibrate magnetometers", sizeof(text.data.str));
//  text.send();

  uint8_t system, gyro, accel, mag;
  do {
    digitalWrite(digPin_LED, HIGH);
    bno.getCalibration(&system, &gyro, &accel, &mag);
//    snprintf(text.data.str, sizeof(text.data.str),
//             "[TNSY] S:%d G:%d A:%d M:%d",
//             system, gyro, accel, mag);
//    text.send();
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

  // PID sample time is slightly less than loop time, to ensure Compute() always executes
  spdPID.SetSampleTime(IMU_SAMPLERATE_DELAY_MS-1);
  spdPID.SetOutputLimits(-1000,1000);
  spdPID.SetMode(AUTOMATIC);
  trnPID.SetSampleTime(IMU_SAMPLERATE_DELAY_MS-1);
  trnPID.SetOutputLimits(-1000,1000);
  trnPID.SetMode(AUTOMATIC);

//  strncpy(text.data.str, "[TNSY] Init done", sizeof(text.data.str));
//  text.send();
}



/*******************************************************
 * LOOP function runs over and over again forever
 *******************************************************/
void loop() 
{
  static unsigned long time_prev_adc, time_prev_imu;
  unsigned long time_now = millis();
  static float bat_percentage;

  static tTxPacket txPacket;
  static tDrive drive;
  
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

    double dx = (dif_right + dif_left) / 2.0 * PI * WHEEL_DIAM_M / ENCODER_STEPS;
#ifdef D_THETA_USE_ENCODERS
    double dth = (dif_right - dif_left) / WHEEL_BASE_M * PI * WHEEL_DIAM_M / ENCODER_STEPS;
#else
    static double last_theta;
    double dth = theta - last_theta;
    if( dth > PI )
      dth = dth - 2*PI;
    if( dth < -PI )
      dth = dth + 2*PI;
    last_theta = theta;
#endif

    txPacket.odom.theta = theta;
    txPacket.odom.dx = dx;
    txPacket.odom.dth = dth;
    txPacket.odom.dt_ms = dt_ms;
    //send both odom and power here:
    Serial.write((uint8_t*)&txPacket, sizeof(tTxPacket));
    Serial.send_now();

    double dt = dt_ms / 1000.0;
    vx  = dx / dt;
    vth = dth / dt;

    // motor control loop
    spdPID.Compute();
    trnPID.Compute();
    UpdateDrive();
  }


  if( time_now - time_prev_adc > ADC_SAMPLERATE_DELAY_MS )
  {
    time_prev_adc = time_now;

    uint8_t charger_state = ChargingState();
/*
    // interrupt continuous V-I and do one-shot VS measurement
    float fVS = analogRead(anlgPin_VS) * (39+39+4.7)/4.7;
    // restart continuous V-I measurements
    adc->startSynchronizedContinuous(anlgPin_IBat, anlgPin_VBat);
*/
    // calculate battery's internal resistance: R = (V1-V2)/(I1-I2)
    // which could be used to estimate battery aging
    noInterrupts();
    int32_t di = adc_I_max - adc_I_min;
    int32_t dv = adc_V_max - adc_V_min;
    adc_reset_min_max = true;
    interrupts();
    static float R_avg;
    // only update R_avg when there's significant difference in current
    if( di >= 4 )
    {
      float R = (float)dv / (float)di / 0.22 - 1.0;
      R_avg = R_avg * 0.9 + R * 0.1;
    }

    // coulomb counting
    if( charger_state == POWER_SUPPLY_STATUS_FULL )
      adc_I_coulomb = 0.0;
    // convert from [counts*us] to [miA*h]
    float charge = adc_I_coulomb * VCC / 0.22 / (1e3 * ADC_SAMPLERATE_DELAY_MS * 3600);

    // convert results into milli(binary)-volts or -amperes
    // so, no need to divide by ADC resolution (i.e. 1023)
    txPacket.power.battery_miA = (int16_t)(adc_I_avg * VCC / 0.22);
    txPacket.power.battery_miV = (uint16_t)(adc_V_avg * VCC);
    txPacket.power.battery_mOhm = (uint16_t)(R_avg * 1000);
    txPacket.power.charger_state = charger_state;
    bat_percentage = BatteryPercentageC(charge);
    txPacket.power.bat_percentage = (uint8_t)(bat_percentage * 100);
    //power will be sent together with odom, which happens more often.
  }


  uint8_t *pdata = (uint8_t*)&drive;
  uint8_t nbytes = 0;

  while (Serial.available())
  {
    *pdata++ = Serial.read();
    if (++nbytes == sizeof(tDrive))
    {
//      cmd_speed = (double)drive.speed_mm_s / 1000.0;
//      cmd_turn = (double)drive.turn_mrad_s / 1000.0;
      break;
    }
  }


  UpdateLED(bat_percentage);

}



/*******************************************************
 * UpdateDrive()
 *   uses global variables pwm_speed and pwm_turn
 *   to calculate and set PWM values that drive motors.
 *******************************************************/
void UpdateDrive(void)
{
  if( cmd_speed > -cmd_speed_min && cmd_speed < cmd_speed_min )
  {
    spdPID.SetMode(MANUAL);
    pwm_speed = 0;
  }
  else
    spdPID.SetMode(AUTOMATIC);

  if( cmd_turn > -cmd_turn_min && cmd_turn < cmd_turn_min )
  {
    trnPID.SetMode(MANUAL);
    pwm_turn = 0;
  }
  else
    trnPID.SetMode(AUTOMATIC);

  pwm_right = pwm_speed + pwm_turn;
  if( pwm_right < 0 )
  {
    pwm_right = -pwm_right;
    dir_right = HIGH;
  }
  else if( pwm_right > 0 )
    dir_right = LOW;

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

  if( pwm_left > 255 )
    pwm_left = 255;

  digitalWrite(pin_motor_dir_right, dir_right);
  analogWrite(pin_motor_pwm_right, pwm_right);
  digitalWrite(pin_motor_dir_left, dir_left);
  analogWrite(pin_motor_pwm_left, pwm_left);
}


/*******************************************************
 * BatteryPercentage()
 *   calculates remaining battery charge based on Voltage
 *   as a value between 0.0 and 1.0
 *******************************************************/
float BatteryPercentageV(float vbat)
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
 * BatteryPercentage()
 *   calculates remaining battery charge based on Coulomb counting
 *   as a value between 0.0 and 1.0
 * coulomb is a negative number when discharging, in units of [miA*h]
 *******************************************************/
float BatteryPercentageC(float coulomb)
{
  float percentage = (BAT_CAPACITY - (-coulomb)) / BAT_CAPACITY;
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

  uint8_t new_state;
  
  analogComparator.configureDac(thrd2);
  delay(10);
  if( analogComparator.getOutput() )
  {
    analogComparator.configureDac(thrd3);
    delay(10);
    if( analogComparator.getOutput() )
      new_state = POWER_SUPPLY_STATUS_FULL;
    else
      new_state = POWER_SUPPLY_STATUS_NOT_CHARGING;
  }
  else
  {
    analogComparator.configureDac(thrd1);
    delay(10);
    if( analogComparator.getOutput() )
      new_state = POWER_SUPPLY_STATUS_DISCHARGING;
    else
      new_state = POWER_SUPPLY_STATUS_CHARGING;
  }

  // shift register to filter new_state:
  static uint8_t accumulator[8]; // power of 2!
  static uint8_t idx = 0;
  static uint8_t last_state;
  
  accumulator[idx++] = new_state;
  idx &= 7; // wrap around

  for (int i=0; i<6; i++)
  {
    if (accumulator[i] != accumulator[i+1])
      return last_state;
  }
  last_state = new_state;
  return new_state;
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


// ADC ISR runs every ~430 us (at 96MHz cpu speed)
void adc0_isr()
{
  static unsigned long isr_last_start;
  unsigned long isr_start = micros();

  ADC::Sync_result adc_result;
  adc_result = adc->readSynchronizedContinuous();
  int32_t result_I = adc_result.result_adc0;
  int32_t result_V = adc_result.result_adc1;
/*
  // get first result
  int32_t result_I = adc->analogReadContinuous(ADC_0);
  // wait for second result to complete
  while( ! adc->isComplete(ADC_1) ) {}
  // get second result
  int32_t result_V = adc->analogReadContinuous(ADC_1);
*/
  // flag adc_reset_min_max is used as a signal to reset min and max calculations
  if( result_I > adc_I_max || adc_reset_min_max )
  {
    adc_I_max = result_I;
    adc_V_max = result_V;
  }
  if( result_I < adc_I_min || adc_reset_min_max )
  {
    adc_I_min = result_I;
    adc_V_min = result_V;
  }
  if( adc_reset_min_max )
    adc_reset_min_max = false;

  // scale and shift results to convert to I and V (still in units of adc counts)
  // negative voltage across current sensing resistor R11 (0.22 ohm)
  // is measured with a resistor divider 100K/(100K||100K) to 1/2 VCC
  float adc_I = (float)result_I * 3.0 - adc->getMaxValue(ADC_0);
  // the above subtracted from voltage measurement to get true battery voltage,
  // while factoring in its resistor divider (100K/50K)
  float adc_V = (float)result_V * 3.0 - adc_I;

  // running average calculations for I and V
  adc_I_avg = adc_I_avg * 0.99 + adc_I * 0.01;
  adc_V_avg = adc_V_avg * 0.99 + adc_V * 0.01;

  // coulomb counting in units of [counts*us]
  adc_I_coulomb += adc_I * (float)(isr_start - isr_last_start);

  isr_last_start = isr_start;
}
