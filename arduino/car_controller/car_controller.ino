/* 
 * Rosserial communication for rc car:
 * Takes in pwm signals from a reciever and publishes 
 *  the data to the onboard computer.
 * Also calculates propper output to an h-bridge to 
 *  control the rc car.
 */

#include <ros.h>
#include <std_msgs/UInt16.h>
#include <std_msgs/Bool.h>

const int LED_PIN = 13;
const int STEERING_PIN = 2; // 670 - 1240
const int THROTTLE_PIN = 3; //570 - 1330
const int LEFT_MOTOR_PIN = 9;
const int RIGHT_MOTOR_PIN = 10;

const int HIGH_PWM = 2000;
const int MID_PWM = 1500;
const int LOW_PWM = 1000;
const int DEAD_BAND = 100;
const int STATIC_OFFSET = 10;
const int RX_PWM_ERROR = 400;
const int SPEED_LIMIT = 100;
const int ARMED = 1000; // 462

unsigned int SteeringPulse;
unsigned int ThrottlePulse;
unsigned int ThrottleRight;
unsigned int ThrottleLeft;
unsigned int Factor;
boolean Autonomous;
boolean Full = false;
boolean Rec = false;

ros::NodeHandle nh;
std_msgs::UInt16 PulseMsg;
std_msgs::Bool ArmedMsg;

ros::Publisher ArmedPub("armed", &ArmedMsg);
ros::Publisher SteeringPub("steering_pwm", &PulseMsg);

void StartRos()
{
  nh.initNode();
  nh.advertise(ArmedPub);
  nh.advertise(SteeringPub);
}

void GetInput()
{
  SteeringPulse = pulseIn(STEERING_PIN , HIGH, 20000) - RX_PWM_ERROR;

  ThrottlePulse = pulseIn(THROTTLE_PIN , HIGH, 20000) - RX_PWM_ERROR;

  if (ThrottlePulse > 1000)
  {
    ArmedMsg.data = true;
  }
  else
  {
    ArmedMsg.data = false;
  }
}

void Output()
{
  /// Publish the ROS messages:
  SteeringPub.publish( &PulseMsg );
  ArmedPub.publish( &ArmedMsg );
  if (ArmedMsg.data)
  {
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
      else if(SteeringPulse < 1500 - DEAD_BAND) // Turn right 
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
      delayMicroseconds(20000 - ThrottleRight);
      digitalWrite(LEFT_MOTOR_PIN, HIGH);
      delayMicroseconds(ThrottleLeft);
      digitalWrite(LEFT_MOTOR_PIN, LOW);
      delayMicroseconds(20000 - ThrottleLeft);
  } 
  else //ModePulse is not at mid
  {
    digitalWrite(LEFT_MOTOR_PIN, LOW);
    delay(100);
    digitalWrite(RIGHT_MOTOR_PIN, LOW); 
  }
}

void setup()
{
  pinMode(STEERING_PIN, INPUT);
  pinMode(THROTTLE_PIN, INPUT);
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
  //delay(10);
}
