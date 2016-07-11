/* ---------------------------------------------------
 * Robot Slave sketch running on Pro Mini
 * monitors battery voltage, current consumption and charging,
 * communicates this to Master Raspberry Pi over I2C connection.
 * 
 * boredman@boredomprojects.net
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

#include <RH_Serial.h>

// instance of serial driver
RH_Serial uart(Serial);


const int anlgPin_VBat = 0;
const int anlgPin_IBat = 1;
const int anlgPin_Stat = 2;
const int anlgPin_Vs   = 3;
const int digPin_LED = 13;
const int digPin_SHD = 2;

// motor drive pins
const int pin_motor_pwm_right = 11;  // M1-PWM
const int pin_motor_dir_right =  9;  // M1-DIR
const int pin_motor_pwm_left  = 10;  // M2-PWM
const int pin_motor_dir_left  =  8;  // M2-DIR

#define VCC (5.0f)


class cRobot 
{
  public:
  enum eCommand {
    CMD_SETLED = 1,
    CMD_STATUS = 2,
    CMD_SETSHD = 3,
    CMD_DRIVE  = 4
  };
  enum eChargerState {
    CHGR_OFF = 10,
    CHGR_INIT = 11,
    CHGR_CHARGE = 12,
    CHGR_TRICKLE = 13,
  };
  struct {
    uint16_t battery_miV;
    int16_t  battery_miA;
    uint16_t vsupply_miV;
    uint16_t charger_state;
  } power;
  struct {
    int16_t speed;
    int16_t turn;
  } drive;
} robot;


byte led_config;

/*******************************************************
 * SETUP function runs once on power-up or reset
 *******************************************************/
void setup() 
{

  pinMode(digPin_LED, OUTPUT);
  led_config = 0x5A;

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

  robot.drive.speed = 0;
  robot.drive.turn = 0;
  
  Serial.begin(115200);
  uart.serial().begin(115200);
  if( ! uart.init() )
    while(1);
}


/*******************************************************
 * LOOP function runs over and over again forever
 *******************************************************/
void loop() 
{
  int bat_miV, bat_miA;
  BatteryVIavg(&bat_miV, &bat_miA, anlgPin_VBat, anlgPin_IBat, 100);
  int state_cnt = analogReadAvg(anlgPin_Stat, 100);
  byte state = ChargingState(state_cnt);
  int vs_miV = analogReadAvg(anlgPin_Vs, 100) * (39+39+4.7)/4.7 * VCC;

  noInterrupts();
  robot.power.battery_miV = bat_miV;
  robot.power.battery_miA = bat_miA;
  robot.power.vsupply_miV = vs_miV;
  robot.power.charger_state = state;
  interrupts();

  UpdateLED();

  UpdateDrive();

  uint8_t buffer[100];
  uint8_t len;
  
  if( uart.recv(buffer, &len) )
  {
    switch( buffer[0] )
    {
      case cRobot::CMD_SETLED :
          if( len == 2 )
            led_config = buffer[1];
          break;
      case cRobot::CMD_STATUS :
          if( len == 1 )
            uart.send((uint8_t*)&(robot.power), sizeof(robot.power));
          break;
      case cRobot::CMD_SETSHD :
          if( len == 2 )
            digitalWrite(digPin_SHD, buffer[1]);
          break;
      case cRobot::CMD_DRIVE  :
          if( len == 1 + sizeof(robot.drive) )
            memcpy(&(robot.drive), &buffer[1], sizeof(robot.drive));
          break;
      default :
          break;
    }
  }

}


/*******************************************************
 * i2cReceiveEvent()
 *  this function is registered as an event
 *  gets called whenever data is received from master
 * array i2cmd[] will be filled with received data:
 *   [0] - total number of bytes received
 *   [1] - command to be executed
 *   [2]..[n] - any additional parameters
 *******************************************************/
/*
void i2cReceiveEvent(int howMany) 
{
  i2cmd[0] = howMany;
  for(int i=1; i<howMany+1 && i<sizeof(i2cmd); i++)
    i2cmd[i] = Wire.read();
}
*/


/*******************************************************
 * i2cRequestEvent()
 *  this function is registered as an event
 *  gets called whenever data is requested by master
 *  executes command previously received with i2cReceiveEvent()
 *******************************************************/
/*
void i2cRequestEvent() 
{
  switch(i2cmd[1])
  {
    case cRobot::CMD_SETLED :
        if(i2cmd[0] == 2)
        {
          led_config = i2cmd[2];
          Wire.write(0);  // success
        }
        else
          Wire.write(-1); // error
        break;
    case cRobot::CMD_STATUS :
        Wire.write((byte*)&(robot.data), sizeof(robot.data)); 
        break;
    case cRobot::CMD_SETSHD:
        if(i2cmd[0] == 2)
        {
          digitalWrite(digPin_SHD, i2cmd[2]);
          Wire.write(0);  // success
        }
        else
          Wire.write(-1); // error
        break;
    default :
        break;
  }
}
*/

#define pwm_min 300

/*******************************************************
 *
 *******************************************************/
void UpdateDrive(void)
{
  int16_t pwm_right, pwm_left;
  int8_t  dir_right, dir_left;

  pwm_right  = robot.drive.speed + robot.drive.turn;
  if( pwm_right < 0 )
  {
    pwm_right = -pwm_right + pwm_min;
    dir_right = HIGH;
  }
  else if( pwm_right > 0 )
  {
    pwm_right = pwm_right + pwm_min;
    dir_right = LOW;
  }
  
  pwm_left  = robot.drive.speed - robot.drive.turn;
  if( pwm_left < 0 )
  {
    pwm_left = -pwm_left + pwm_min;
    dir_left = HIGH;
  }
  else if( pwm_left > 0 )
  {
    pwm_left = pwm_left + pwm_min;
    dir_left = LOW;
  }

  analogWrite(pin_motor_pwm_right, highByte(pwm_right));
  analogWrite(pin_motor_pwm_left, highByte(pwm_left));
  digitalWrite(pin_motor_dir_right, dir_right);
  digitalWrite(pin_motor_dir_left, dir_left);
}



#define min_pulse_ms 100

/*******************************************************
 * UpdateLED()
 *  controls state of LED - indicator of battery voltage
 *  according to global variable led_config:
 *    0    - LED always OFF
 *    0xFF - LED always ON
 *    0x5A - LED pulsed according to battery voltage.
 *           rate of pulsation is determined by function
 *           voltage2millis()
 *******************************************************/
void UpdateLED(void)
{
  static byte state;
  static unsigned long last_time;

  if( led_config == 0x5A )
  {
    if( state == LOW && (millis() - last_time) > voltage2millis() )
    {
      digitalWrite(digPin_LED, HIGH);
      state = HIGH;
      last_time = millis();
    }
    else if( state == HIGH && (millis() - last_time) > min_pulse_ms )
    {
      digitalWrite(digPin_LED, LOW);
      state = LOW;
      last_time = millis();
    }
  }
  else
  {
    digitalWrite(digPin_LED, state = led_config ? HIGH : LOW);
  }

}


/*******************************************************
 * voltage2millis()
 *  calculates the duration of pulses for battery voltage
 *  indicator LED.
 *  lower the voltage -> shorter the duration.
 *******************************************************/
unsigned long voltage2millis(void)
{
  // lowest battery level is 0.9V per cell
  if( robot.power.battery_miV <= (uint16_t)(6 * 0.9 * 1024) )
    return min_pulse_ms; // alarm!
  else
    return (robot.power.battery_miV - (uint16_t)(6 * 0.9 * 1024)) * 2 + min_pulse_ms;
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
 *  determines charging state by measuring voltage on 
 *  dual-color LED of Graupner 6425 charger.
 * parameter:
 *   adc - raw analog input voltage measurement
 * returns: 
 *   a constant as defined by cRobot::eChargerState
 *******************************************************/
cRobot::eChargerState ChargingState(int adc)
{
  const int thrd1 = 80;
  const int thrd2 = 200;
  const int thrd3 = 800;
  
  if( adc < thrd1 )
    return cRobot::CHGR_CHARGE;
  else if( adc >= thrd1 && adc < thrd2 )
    return cRobot::CHGR_OFF;
  else if( adc >= thrd2 && adc < thrd3 )
    return cRobot::CHGR_INIT;
  else //if( adc >= thrd3 )
    return cRobot::CHGR_TRICKLE;
}

