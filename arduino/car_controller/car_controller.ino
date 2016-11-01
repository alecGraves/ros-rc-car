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
#include <Servo.h>

const int LED_PIN = 6;
const int STEERING_PIN = 2;
const int THROTTLE_PIN = 3;
const int FORWARD_MOTOR_PIN = 9;
const int REVERSE_MOTOR_PIN = 10;
const int MOTOR_PIN = 11;
const int SERVO_PIN = A0;
const int MODE_PIN = 5;
const int HIGH_PWM = 2000;
const int MID_PWM = 1500;
const int LOW_PWM = 1000;
const int DEAD_BAND = 100;
const int STATIC_OFFSET = 10;
const int RX_PWM_ERROR = -100;
const int SPEED_LIMIT = 100;


const boolean DoDebug = true;
const boolean ReverseThrottle = true;
Servo Servo1;

unsigned int SteeringPulse;
unsigned int ThrottlePulse;
unsigned int ModePulse;
unsigned int ThrottleRight;
unsigned int ThrottleLeft;
boolean Forward;

enum Mode { disarmed, armed, recording, autonomous };

Mode CurrentMode;


ros::NodeHandle nh;
std_msgs::UInt16 PulseMsg;
std_msgs::Bool RecMsg;

ros::Publisher RecPub("moving", &RecMsg);
ros::Publisher SteeringPub("steering_pwm", &PulseMsg);
ros::Publisher ThrottlePub("throttle_pwm", &PulseMsg);
ros::Publisher ModePub("mode_pwm", &PulseMsg);


void StartRos()
{
  nh.initNode();
  nh.advertise(RecPub);
  nh.advertise(SteeringPub);
  if (DoDebug)
  {
    nh.advertise(ThrottlePub);
    nh.advertise(ModePub);
  }
}

void GetInput()
{
  SteeringPulse = pulseIn(STEERING_PIN , HIGH, 20000) - RX_PWM_ERROR;

  if (ReverseThrottle)
  {
    ThrottlePulse = 3000-(pulseIn(THROTTLE_PIN , HIGH, 20000) - RX_PWM_ERROR);
  }
  else
  {
    ThrottlePulse = pulseIn(THROTTLE_PIN , HIGH, 20000) - RX_PWM_ERROR;
  }

  ModePulse = pulseIn(MODE_PIN , HIGH, 20000) - RX_PWM_ERROR;
}

void SetDirection(boolean forward, bool init = false) // 1 for forward, 0 for reverse
{
  if (init)//initialize state
  {
    digitalWrite(REVERSE_MOTOR_PIN, LOW);
    digitalWrite(FORWARD_MOTOR_PIN, HIGH);
    Forward = true;
  }
  else
  {
    ///set direction using logic pins of hbridge
    if (forward == Forward) //requested and current state are the same
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
}

void Output()
{
  //mode?
  if (ModePulse > 1600)
  {
    CurrentMode = disarmed;
    RecMsg.data = false;
  }
  else if (ModePulse > 1400)
  {
    CurrentMode = armed;
    RecMsg.data = false;
  }
  else // Rec
  {
    CurrentMode = recording;
    RecMsg.data = true; //Rec
  }

  /// Publish the ROS messages:
  PulseMsg.data = SteeringPulse;
  SteeringPub.publish(&PulseMsg);
  RecPub.publish(&RecMsg);

  if (DoDebug)
  {
    PulseMsg.data = ThrottlePulse;
    ThrottlePub.publish(&PulseMsg);
    PulseMsg.data = ModePulse;
    ModePub.publish(&PulseMsg);
  }

  if (CurrentMode == armed  || CurrentMode == recording)
  {
    digitalWrite(LED_PIN, HIGH); //armed

    if (abs(ThrottlePulse-MID_PWM) > DEAD_BAND) //not at mid
    {
      //throttle direction
      if(ThrottlePulse > MID_PWM)
      {
        SetDirection(1); //1 for forward
      }
      else 
      {
        SetDirection(0); //0 for reverse
      }
      //throttle magnitide
      //map(in, in_min, in_max, out_min, out_max)
      if (ThrottlePulse > MID_PWM + 300)
      {
        ThrottlePulse = MID_PWM + 300;
      }
      else if (ThrottlePulse < MID_PWM - 300)
      {
        ThrottlePulse = MID_PWM - 300;
      }
      ThrottlePulse = map(abs(MID_PWM-ThrottlePulse), 0, 300, 0, 255);
      //control motors
      //analogWrite(MOTOR_PIN, ThrottlePulse);
      analogWrite(MOTOR_PIN, 255);

    }
    else //throttle at mid
    {
      //write LOW pin
      digitalWrite(MOTOR_PIN, LOW);
    }

    
    
    //steering servo direction
    SteeringPulse = map(SteeringPulse, LOW_PWM, HIGH_PWM, 30, 150);

    //control servo
    Servo1.write(SteeringPulse);
  } 
  else //disarmed
  {
    //disable motor and blink
    digitalWrite(LED_PIN, LOW);
    digitalWrite(MOTOR_PIN, LOW);
    delay(100);
    digitalWrite(LED_PIN, HIGH);
    delay(100);
    digitalWrite(LED_PIN, LOW);
  }
}

void setup()
{
  pinMode(STEERING_PIN, INPUT);
  pinMode(THROTTLE_PIN, INPUT);
  pinMode(MODE_PIN, INPUT);
  pinMode(LED_PIN, OUTPUT);
  pinMode(FORWARD_MOTOR_PIN, OUTPUT);
  pinMode(REVERSE_MOTOR_PIN, OUTPUT);
  pinMode(MOTOR_PIN, OUTPUT);
  pinMode(SERVO_PIN, OUTPUT);
  
  Servo1.attach(SERVO_PIN);
  
  SetDirection(1, true);//init direction
  
  StartRos();
}

void loop()
{
  GetInput();
  Output();
  
  nh.spinOnce();
  //delay(10);
}
