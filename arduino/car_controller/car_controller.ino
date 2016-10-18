/* 
 * rosserial::std_msgs::Float64 Test
 * Receives a Float64 input, subtracts 1.0, and publishes it
 */

#include <ros.h>
#include <std_msgs/UInt16.h>
#include <std_msgs/Bool.h>

const int LED_PIN = 13;
const int STEERING_PIN = 2; // 670 - 1240
const int THROTTLE_PIN = 3; //570 - 1330
//const int MODE_PIN = 4;
const int LEFT_MOTOR_PIN = 9;
const int RIGHT_MOTOR_PIN = 10;

const int HIGH_PWM = 2000;
const int MID_PWM = 1500;
const int LOW_PWM = 1000;
const int DEAD_BAND = 100;
const int STATIC_OFFSET = 10;
const int RX_PWM_ERROR = 400;
const int SPEED_LIMIT = 100;
const int ARMED = 500; // 462

unsigned int SteeringPulse;
unsigned int ThrottlePulse;
//unsigned int ModePulse;
unsigned int ThrottleRight;
unsigned int ThrottleLeft;
unsigned int Factor;
boolean Armed;
boolean Autonomous;
boolean full = false;
boolean rec = false;

ros::NodeHandle nh;
std_msgs::UInt16 PulseMsg;

ros::Publisher SteeringPub("steering_pwm", &PulseMsg);
ros::Publisher ThrottlePub("throttle_pwm", &PulseMsg);
//ros::Publisher ModePub("mode_pwm", &PulseMsg);
ros::Subscriber<std_msgs::Bool> FullSub("full_msg", &fullCb);
ros::Subscriber<std_msgs::Bool> RecSub("rec_msg", &recCb);

void StartRos()
{
  nh.initNode();
  nh.advertise(SteeringPub);
  nh.advertise(ThrottlePub);
  //nh.advertise(ModePub);
  nh.subscribe(FullSub)
}

void fullCb(const std_msgs::Bool& full_msg)
{
  if(full_msg==true && full==false)
  {
    full = true;
  }
  else if(full_msg==false && full==true)
  {
    full = false;
  }
}

void recCb(const std_msgs::Bool& rec_msg)
{
  if(rec_msg==true && rec==false)
  {
    rec = true;
  }
  else if(rec_msg==false && rec==true)
  {
    rec = false;
  }
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
  /*
  else if (_channel == MODE_PIN)
  {
    ModePub.publish( &PulseMsg);
  }
  */
}

void GetInput()
{
  SteeringPulse = pulseIn(STEERING_PIN , HIGH, 20000) - RX_PWM_ERROR;
  PubPulse(STEERING_PIN, SteeringPulse);
  
  ThrottlePulse = pulseIn(THROTTLE_PIN , HIGH, 20000) - RX_PWM_ERROR;
  PubPulse(THROTTLE_PIN, ThrottlePulse);
  
  /*
  ModePulse = pulseIn(MODE_PIN , HIGH, 20000) - RX_PWM_ERROR;
  PubPulse(MODE_PIN, ModePulse);
  */
}

void Output()
{
  //activation controlled by mode channel
  if (ThrottlePulse > ARMED) // Check if armed/recording
  {
    if(!full)
    {
      if(rec)
      {
        digitalWrite(LED_PIN, HIGH); // turn on light
      }
      else
    }

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
    digitalWrite(RIGHT_MOTOR_PIN, LOW); 
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
