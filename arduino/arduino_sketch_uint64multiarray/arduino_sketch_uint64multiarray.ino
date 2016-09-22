/* 
 * Reciever (PWM) Data transmission with rosserial  
 * 
 * This sketch is designed to work readily with arduino UNO
 * 
 * Visit https://github.com/shadySource/rx-arduino-ros for info
 *
 * Created by Alec Graves
 */

#include <ros.h>
#include <std_msgs/UInt64MultiArray.h>

const int NUM_CHANNELS = 7;
const int DIM = 1;

//this program assumes you populate pin(s) 0 to NUM_CHANNELS.
//you should only use digital pins (0-13 on arduino UNO)

ros::NodeHandle  Nh;

std_msgs::UInt64MultiArray RecieverData;
std_msgs::MultiArrayDimension _dim;

ros::Publisher PubRecieverData( "reciever_data", &RecieverData);

int pin;

char msg_label[] = "microseconds";

void ConfigMsg()
{
  _dim.label = msg_label;
  _dim.size = NUM_CHANNELS;
  _dim.stride = NUM_CHANNELS;
  RecieverData.layout.dim = &_dim;
  RecieverData.layout.dim_length = DIM;
  RecieverData.data_length = NUM_CHANNELS; 
} 

void AllocArr()
{
  int len = sizeof( uint64_t ) * NUM_CHANNELS;
  RecieverData.data = (uint64_t*) ::operator new( len );
  uint64_t *_datPtr;
  for (int i = 0; i < NUM_CHANNELS; i ++)
  {
    _datPtr[i]=0;
  }
}

void setup()
{

  Nh.initNode();
  Nh.advertise(PubRecieverData);

  delay(100);
  
  ConfigMsg();
  AllocArr();
}

///////////////////////////////////////////////////////
// Gather and publish the data.
//
// Since pwms are output from the reciever at 10-20ms, 
//   it could take 100 ms to cycle through 5 channels.
//
// A better way is with Interrupt Service Routines, 
//   but the arduino UNO only has 2 interrupt ports.
//
///////////////////////////////////////////////////////
void loop()
{
  for(int i = 0; i < NUM_CHANNELS; i++)
  {
    pin = i;
    pin += 2;
    RecieverData.data[i]= pulseIn(pin, HIGH, 20000);
    // pulseIn measures microseconds between pulses
    // pulseIn returns 0 if timeout is reached
    // 20000 is the timeout in microseconds
    // i is the pin, so this assumes you use 
    //   pin(s) 2 to NUM_CHANNELS.
  } 
  PubRecieverData.publish(&RecieverData);
  
  Nh.spinOnce();
  
}
