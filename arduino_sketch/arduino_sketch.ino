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
//#include <std_msgs/MultiArrayDimension.h>
//#include <std_msgs/MultiArrayLayout.h>

const int NUM_CHANNELS = 1;
//this program assumes you populate pin(s) 0 to NUM_CHANNELS.
//you should only use digital pins (1-13 on arduino UNO)

ros::NodeHandle  Nh;

std_msgs::UInt64MultiArray RecieverData;
ros::Publisher PubRecieverData( "RecieverData", &RecieverData);

void setup()
{
  Nh.initNode();
  Nh.advertise(PubRecieverData);

  RecieverData.layout.dim[0].size = NUM_CHANNELS;

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
  for(int i = 0; i < NUM_CHANNELS; i++)
  {
    RecieverData.data[i] = pulseIn(i, HIGH, 20000);
    // pulseIn returns 0 if timeout is reached.
    // 20000 is the timeout in microseconds.
    // i is the pin, so this assumes you use 
    //   pin(s) 0 to NUM_CHANNELS.
  }
 
  PubRecieverData.publish(&RecieverData);
  Nh.spinOnce();
}
