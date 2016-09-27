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
#include <sensor_msgs/ChannelFloat32.h>

const int NUM_CHANNELS = 5;
//this program assumes you populate pin(s) 0 to NUM_CHANNELS.
//you should only use digital pins (4-13 on arduino UNO)

ros::NodeHandle  Nh;

sensor_msgs::ChannelFloat32 RecieverData;

ros::Publisher PubRecieverData( "reciever_data", &RecieverData);

char msg_label[] = "miliseconds";

float Data[NUM_CHANNELS];
  
void setup()
{
  Nh.initNode();
  Nh.advertise(PubRecieverData);
  delay(100);
  RecieverData.name = msg_label;
  RecieverData.values_length = NUM_CHANNELS;
  for (int i = 0; i < NUM_CHANNELS; i++)
  {
    int inpin = i;
    inpin +=4;
    pinMode(inpin, INPUT);
  }
}

void loop()
{
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

  for (int i = 0; i < NUM_CHANNELS; i++)
  {
    int inpin = i;
    inpin += 4;
    Data[i] = float(pulseIn(inpin, HIGH, 20000));
    // pulseIn returns 0 if timeout is reached.
    // 20000 is the timeout in microseconds.
    // i is the pin, so this assumes you use 
    //   pin(s) 2 to NUM_CHANNELS.
  }
  RecieverData.values = Data;
  PubRecieverData.publish(&RecieverData);
  
  Nh.spinOnce();
  
}
