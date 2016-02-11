#include "ros/ros.h"
#include "std_msgs/Bool.h"
#include "std_msgs/UInt16.h"
#include <rosbag/bag.h>
#include <string>
#include <sensor_msgs/Image.h>

using namespace std;

rosbag::Bag bag;
std_msgs::UInt16 steering_msg;
std_msgs::Bool moving;
bool newBag = true;

void steering_callback(const std_msgs::UInt16::ConstPtr& msg)
{
	steering_msg.data = msg->data;
}

void armed_callback(const std_msgs::Bool::ConstPtr& msg)
{
	moving.data = msg->data;
}

void image_callback(const sensor_msgs::Image::ConstPtr& msg)
{
	if(moving.data && !newBag)
    {
        bag.write("usb_cam/image_raw",ros::Time::now(), msg);
        bag.write("steering_pwm",ros::Time::now(), steering_msg);
    }
}

int main(int argc, char **argv)
{
	steering_msg.data = 0;

	ros::init(argc, argv, "record_video");
	ros::NodeHandle nh;
	std::string bag_filename = std::to_string(ros::Time::now().toSec()) + ".bag";
	double creation_time = ros::Time::now().toSec();
	double time_diff = 3;

	ros::Subscriber steering_sub = nh.subscribe("steering_pwm", 1, steering_callback);
	ros::Subscriber armed_sub = nh.subscribe("moving", 1, armed_callback);	
	ros::Subscriber camera_sub = nh.subscribe("usb_cam/image_raw", 1, image_callback);

	ros::Rate loop_rate(10);

	while(ros::ok())
	{
		if(moving.data && newBag)
		{
				bag_filename = std::to_string(ros::Time::now().toSec()) + ".bag";
				bag.open(bag_filename, rosbag::bagmode::Write);
				newBag = false;
				creation_time = ros::Time::now().toSec();
		}
		else if(!moving.data && !newBag)
		{
			bag.close();
			newBag = true;
		}
		
		ros::spinOnce();
	}	

	bag.close();
	return 0;
}
