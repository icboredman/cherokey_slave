/* ---------------------------------------------------
 * Robot Slave sketch running on Pro Mini
 * monitors battery voltage, current consumption and charging,
 * communicates this to Master Raspberry Pi over I2C connection.
 * 
 * boredman@boredomprojects.net
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
 * uses
 *    "Wire" library for I2C communication
 * ---------------------------------------------------
 */

#include <ros.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/BatteryState.h>

using namespace sensor_msgs;

const int digPin_LED = 13;
const int digPin_SHD = 2;

// power supply pins
const int anlgPin_VBat = 0;
const int anlgPin_IBat = 1;
const int anlgPin_Stat = 2;
const int anlgPin_Vs   = 3;

// motor drive pins (PWM must be on pins 9 and 10)
const int pin_motor_pwm_right =  9;  // M1-PWM
const int pin_motor_dir_right = 11;  // M1-DIR
const int pin_motor_pwm_left  = 10;  // M2-PWM
const int pin_motor_dir_left  =  8;  // M2-DIR

#define VCC (5.0f)

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

  nh.getHardware()->setBaud(115200);
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
  TCCR1B = TCCR1B & 0b11111000 | 0x05;  // about 30 Hz
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
    
    int bat_miV, bat_miA;
    BatteryVIavg(&bat_miV, &bat_miA, anlgPin_VBat, anlgPin_IBat, 100);
    int vs_miV = analogReadAvg(anlgPin_Vs, 100) * (39+39+4.7)/4.7 * VCC;
    int state_adc = analogReadAvg(anlgPin_Stat, 100);

    bat_msg.voltage = bat_miV / 1024.0;
    bat_msg.current = bat_miA / 1024.0;
    bat_msg.charge = vs_miV / 1024.0; // use for VS
    bat_msg.power_supply_status = ChargingState(state_adc);
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
 *  - LED pulsed according to battery voltage.
 *    rate of pulsation is determined by function voltage2millis()
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
 * BatteryVIavg()
 *  measures and reports battery Voltage and Current.
 * parameters:
 *   pinV - analog pin for voltage measurements
 *   pinI - analog pin for current measurements
 *          (as voltage across a current sensing resistor)
 *   avg  - number of averages to take before returning
 * results returned using:
 *   miV  - battery voltage in units of [volts x 1024]
 *   miA  - battery current in units of [amperes x 1024]
 *******************************************************/
void BatteryVIavg(int* miV, int* miA, int pinV, int pinI, int avg)
{
  int adcV, adcI;
  float fV, fI;
  float sumV=0, sumI=0;

  for(int i=0; i<avg; i++)
  {
    // sample ADC pins
    adcI = analogRead(pinI);
    adcV = analogRead(pinV);

    // negative voltage across current sensing resistor R11 (0.22 ohm)
    // is measured with a resistor divider (100K/100K) to 5V
    fI = (float)adcI * 2.002 - (5.0/VCC)*1024;
    // the above subtracted from voltage measurement to get true battery voltage,
    // while factoring in its resistor divider (100K/100K)
    fV = (float)adcV * 1.995 - fI;
    
    // accumulate averages (still in adc counts)
    sumI += fI;
    sumV += fV;
  }

  // scale results into milli(binary)-volts or -amperes
  // so, no need to divide by ADC resolution (i.e. 1024)
  *miV = (int)((sumV / avg) * VCC);
  *miA = (int)((sumI / avg) * VCC / 0.22);
}


/*******************************************************
 * analogReadAvg()
 *  implements analogRead() functionality, but with averaging.
 *******************************************************/
int analogReadAvg(int pin, int avg)
{
  long sum = 0;
  for(int i=0; i<avg; i++)
  {
    sum += analogRead(pin);
  }
  return (int)(sum / avg);
}


/*******************************************************
 * ChargingState()
 *  calculates charging state using voltage on 
 *  dual-color LED of Graupner 6425 charger.
 * parameter:
 *   adc - raw analog input voltage measurement
 * returns: 
 *   a constant as defined by BatteryState::POWER_SUPPLY_STATUS_...
 *******************************************************/
uint8_t ChargingState(int adc)
{
  const int thrd1 = 80;
  const int thrd2 = 200;
  const int thrd3 = 800;
  
  if( adc < thrd1 )
    return BatteryState::POWER_SUPPLY_STATUS_CHARGING;
  else if( adc >= thrd1 && adc < thrd2 )
    return BatteryState::POWER_SUPPLY_STATUS_DISCHARGING;
  else if( adc >= thrd2 && adc < thrd3 )
    return BatteryState::POWER_SUPPLY_STATUS_NOT_CHARGING;
  else //if( adc >= thrd3 )
    return BatteryState::POWER_SUPPLY_STATUS_FULL;
}

