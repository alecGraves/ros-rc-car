/* 
 * rosserial::std_msgs::Uint16 message
 * Receives a pwm signal from input pins and publishes it
 * Then calculates motor output signal and publishes it
 * The motor signal is meant to be recieved and converted by an h bride
 */

#include <ros.h>
#include <std_msgs/UInt16.h>

const int LED_PIN = 13;
const int STEERING_PIN = 2;
const int THROTTLE_PIN = 3;
const int MODE_PIN = 4;
const int LEFT_MOTOR_PIN = 9;
const int RIGHT_MOTOR_PIN = 10;

const int HIGH_PWM = 2000;
const int MID_PWM = 1500;
const int LOW_PWM = 1000;
const int DEAD_BAND = 100;
const int STATIC_OFFSET = 10;
const int RX_PWM_ERROR = 400;
const int SPEED_LIMIT = 100;

unsigned int SteeringPulse;
unsigned int ThrottlePulse;
unsigned int ModePulse;
unsigned int ThrottleRight;
unsigned int ThrottleLeft;
unsigned int Factor;
boolean Autonomous;


ros::NodeHandle nh;
std_msgs::UInt16 PulseMsg;

ros::Publisher SteeringPub("steering_pwm", &PulseMsg);
ros::Publisher ThrottlePub("throttle_pwm", &PulseMsg);
ros::Publisher ModePub("mode_pwm", &PulseMsg);

void StartRos()
{
  nh.initNode();
  nh.advertise(SteeringPub);
  nh.advertise(ThrottlePub);
  nh.advertise(ModePub);
}

void PubPulse(const int &_channel, const uint16_t &_pulse)
{
  PulseMsg.data = _pulse;
  
  if (_channel == STEERING_PIN)
  {
    SteeringPub.publish( &PulseMsg );
  }
  else if (_channel == THROTTLE_PIN)
  {
    ThrottlePub.publish( &PulseMsg);
  }
  else if (_channel == MODE_PIN)
  {
    ModePub.publish( &PulseMsg);
  }
  
}

void GetInput()
{
  SteeringPulse = pulseIn(STEERING_PIN , HIGH, 20000) - RX_PWM_ERROR;
  PubPulse(STEERING_PIN, SteeringPulse);
  
  ThrottlePulse = pulseIn(THROTTLE_PIN , HIGH, 20000) - RX_PWM_ERROR;
  PubPulse(THROTTLE_PIN, ThrottlePulse);
  
  ModePulse = pulseIn(MODE_PIN , HIGH, 20000) - RX_PWM_ERROR;
  PubPulse(MODE_PIN, ModePulse);
}

void Output()
{
  //activation controlled by mode channel
  if (abs(ModePulse - MID_PWM) < 100 && G2G == true) //ModePulse ~= mid pwm
  {
    digitalWrite(LED_PIN, HIGH);

    //throttle
     if(ThrottlePulse > 1500 + DEAD_BAND) 
     {
        ThrottlePulse = map(ThrottlePulse, 1500 + DEAD_BAND, 1880, 0, SPEED_LIMIT);
     }
     else if(ThrottlePulse < 1480-DEAD_BAND) 
     {
        ThrottlePulse = map(ThrottlePulse, 1500 - DEAD_BAND, 1080, 0, -1*SPEED_LIMIT);
     } 
     else 
     {
       ThrottlePulse = 0;
     }
     
     //steer
     if(SteeringPulse > 1500 + DEAD_BAND) // Turn left
     {
        Factor = map(SteeringPulse, 1500 + DEAD_BAND, 1880, 0, 100);
        // Right motor  CW (tune with 80)
        ThrottleRight = MID_PWM + ThrottlePulse + 1*ThrottlePulse*Factor/100;
        // Left  motor CCW
        ThrottleLeft  = MID_PWM - ThrottlePulse + 1*ThrottlePulse*Factor/100;
     }
     else if(steer < 1500 - DEAD_BAND) // Turn right 
     {
        Factor = map(SteeringPulse, 1500 - DEAD_BAND, 1080, 0, 100);
        ThrottleLeft  = MID_PWM - ThrottlePulse - 1*ThrottlePulse*Factor/100;
        ThrottleRight = MID_PWM + ThrottlePulse - 1*ThrottlePulse*Factor/100;
     }
     else
     {
       ThrottleLeft  = MID_PWM - ThrottlePulse;
       ThrottleRight = MID_PWM + ThrottlePulse + STATIC_OFFSET;
     }
     
     //control motors
     digitalWrite(RIGHT_MOTOR_PIN, HIGH);
     delayMicroseconds(ThrottleRight);
     digitalWrite(RIGHT_MOTOR_PIN, LOW);
     delayMicroseconds(20000 - throttleRight);
     digitalWrite(LEFT_MOTOR_PIN, HIGH);
     delayMicroseconds(ThrottleLeft);
     digitalWrite(LEFT_MOTOR_PIN, LOW);
     delayMicroseconds(20000 - throttleLeft);
  } 
  else //ModePulse is not at mid
  {
    digitalWrite(leftMotorPin, LOW);
    digitalWrite(rightMotorPin, LOW); 
    digitalWrite(LED, HIGH); // Blinking LED = switch in off mode
    delay(100);
    digitalWrite(LED, LOW);
    delay(100);
  }
}

void setup()
{
  pinMode(STEERING_PIN, INPUT);
  pinMode(THROTTLE_PIN, INPUT);
  pinMode(MODE_PIN, INPUT);
  pinMode(LED_PIN, OUTPUT);
  pinMode(LEFT_MOTOR_PIN, OUTPUT);
  pinMode(RIGHT_MOTOR_PIN, OUTPUT);
  
  StartRos();
}

void loop()
{
  GetInput();
  Output();
  
  nh.spinOnce();
  delay(10);
}

