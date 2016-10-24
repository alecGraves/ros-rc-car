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
#include <servo.h>

const int LED_PIN = 6;
const int STEERING_PIN = 2; // 670 - 1240
const int THROTTLE_PIN = 3; //570 - 1330
const int FORWARD_MOTOR_PIN = 9;
const int REVERSE_MOTOR_PIN = 10;
const int MOTOR_PIN = 11;
const int SERVO_PIN = 15;

const int HIGH_PWM = 2000;
const int MID_PWM = 1500;
const int LOW_PWM = 1000;
const int DEAD_BAND = 100;
const int STATIC_OFFSET = 10;
const int RX_PWM_ERROR = 0;
const int SPEED_LIMIT = 100;
const int ARMED = 1000; // 462

Servo Servo1;

unsigned int SteeringPulse;
unsigned int ThrottlePulse;
unsigned int ThrottleRight;
unsigned int ThrottleLeft;
unsigned int Factor;
bool Reverse;
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

  if (abs(ThrottlePulse - 1500) > DEAD_BAND)
  {
    ArmedMsg.data = true;
  }
  else 
  {
    ArmedMsg.data = false;
  }
}

void setDirection(bool forward) // 1 for forward, 0 for reverse
{
  ///set direction using logic pins of hbridge
  if (forward == Forward)
  {
    // do nothing
  }
  else
  {
    // change directions
    if (Forward == true)
    {
      digitalWrite(FORWARD_MOTOR_PIN, LOW);
      digitalWrite(REVERSE_MOTOR_PIN, HIGH);
    }
    else
    {
      digitalWrite(REVERSE_MOTOR_PIN, LOW);
      digitalWrite(FORWARD_MOTOR_PIN, HIGH);
    }
    //store current state
    Forward = forward;
  }
}

void Output()
{
  /// Publish the ROS messages:
  SteeringPub.publish( &PulseMsg );
  ArmedPub.publish( &ArmedMsg );

  if (ArmedMsg.data) // ArmedMsg.data is assigned in GetInputs()
  {
    //throttle magnitide
    //map(in, in_min, in_max, out_min, out_max)
    ThrottlePulse = map(abs(1500-ThrottlePulse, 0, 500, 0, 255);

    //throttle direction
    if(ThrottlePulse > 1500) //Deadband considered in ArmedMsg.data
    {
      setDirection(1); // 1 for forward
    }
    else 
    {
      setDirection(0); // 0 for reverse
    }
    
    //steering servo direction
    SteeringPulse = map(SteeringPulse, 1000, 2000, 30, 150);

    //control servo
    Servo1.write(SteeringPulse);

    //control motors
    analogWrite(MOTOR_PIN, ThrottlePulse);
  } 
  else //ModePulse is not at mid
  {
    digitalWrite(MOTOR_PIN, LOW);
    delay(100);
  }
}

void setup()
{
  pinMode(STEERING_PIN, INPUT);
  pinMode(THROTTLE_PIN, INPUT);
  pinMode(LED_PIN, OUTPUT);
  pinMode(FORWARD_MOTOR_PIN, OUTPUT);
  pinMode(REVERSE_MOTOR_PIN, OUTPUT);
  pinMode(MOTOR_PIN, OUTPUT);
  pinMode(SERVO_PIN, OUTPUT);
  
  Servo1.attach(SERVO_PIN);

  StartRos();
}

void loop()
{
  GetInput();
  Output();
  
  nh.spinOnce();
  //delay(10);
}
