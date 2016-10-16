/* ---------------------------------------------------
 * Robot Slave sketch running on Teensy 3.2
 * monitors battery voltage, current consumption and charging,
 * communicates this to Master Raspberry Pi over Serial connection.
 * 
 * boredman@boredomprojects.net
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

// Teensy analog comparator library
#include "analogComp.h"

// Teensy ADC library
#include "ADC.h"

#define USE_TEENSY_HW_SERIAL
#include <ros.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/BatteryState.h>

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

// http://answers.ros.org/question/217986/rosserial-with-teensy-31-comm-port/
class NewHardware : public ArduinoHardware
{
  public:
  NewHardware():ArduinoHardware(&Serial1, 115200){}; // Specify which port you want to use
};
ros::NodeHandle_<NewHardware> nh;

ros::Subscriber<geometry_msgs::Twist> sub_vel("cherokey/cmd_vel", &callback_cmd_vel );

BatteryState bat_msg;
ros::Publisher pub_bat("cherokey/battery", &bat_msg);


ADC *adc = new ADC(); // adc object
ADC::Sync_result adcRes;


/*******************************************************
 * SETUP function runs once on power-up or reset
 *******************************************************/
void setup() 
{
  pinMode(digPin_LED, OUTPUT);

  pinMode(digPin_SHD, OUTPUT);
  digitalWrite(digPin_SHD, LOW);

  // populate some constant values
  bat_msg.design_capacity = 2500.0;
  bat_msg.power_supply_technology = BatteryState::POWER_SUPPLY_TECHNOLOGY_NIMH;
  bat_msg.present = true;

  nh.initNode();
  nh.advertise(pub_bat);
  nh.subscribe(sub_vel);

  pinMode(pin_motor_pwm_right, OUTPUT);
  digitalWrite(pin_motor_pwm_right, LOW);
  pinMode(pin_motor_dir_right, OUTPUT);
  digitalWrite(pin_motor_dir_right, LOW);
  pinMode(pin_motor_pwm_left, OUTPUT);
  digitalWrite(pin_motor_pwm_left, LOW);
  pinMode(pin_motor_dir_left, OUTPUT);
  digitalWrite(pin_motor_dir_left, LOW);

  // Reduce PWM frequency for timer1 (pins 9 and 10) to about 30 Hz,
  // to prevent buzzing sounds from motors.
  // This should not cause interference to delay() amd millis(),
  // as those functions use timer0 and timer2.
  // (http://playground.arduino.cc/Code/PwmFrequency)
//  TCCR1B = TCCR1B & 0b11111000 | 0x05;  // about 30 Hz

  // 50Hz or so
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
}


/*******************************************************
 * LOOP function runs over and over again forever
 *******************************************************/
void loop() 
{
  static unsigned long time_prev;
  unsigned long time_now = millis();
  if( time_now - time_prev > 1000 )
  {
    time_prev = time_now;

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


