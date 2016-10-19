// df -hT /home | grep -o '[0-9]*%' | grep -o '[0-9]*'

#include "ros/ros.h"
#include "std_msgs/Bool.h"
#include "std_msgs/UInt16.h"
#include <rosbag/bag.h>
#include <string>
#include <sensor_msgs/Image.h>

//using namespace std;

rosbag::Bag bag;
std_msgs::UInt16 steering_msg;
std_msgs::Bool armed_msg;
bool newBag = true;

void steering_callback(const std_msgs::UInt16::ConstPtr& msg)
{
	steering_msg.data = msg->data;
}

void armed_callback(const std_msgs::Bool::ConstPtr& msg)
{
	armed_msg.data = msg->data;
}

void image_callback(const sensor_msgs::Image::ConstPtr& msg)
{
	if(/*armed_msg.data &&*/ !newBag)
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
	std::string bag_filename = std::to_string(ros::Time::now().toSec());

	ros::Publisher Full_Msg = nh.advertise<std_msgs::Bool>("full_msg", 1000);
	ros::Publisher Rec_Msg = nh.advertise<std_msgs::Bool>("rec_msg",1000);
	ros::Subscriber steering_sub = nh.subscribe("steering_pwm", 1, steering_callback);
	ros::Subscriber armed_sub = nh.subscribe("armed", 1, armed_callback);	
	ros::Subscriber camera_sub = nh.subscribe("usb_cam/image_raw", 1, image_callback);

	ros::Rate loop_rate(10);

	while(ros::ok())
	{
		std_msgs::Bool full_msg;
		std_msgs::Bool rec_msg;
		
		full_msg.data = false;
		rec_msg.data = true;	

		Full_Msg.publish(full_msg);
		Rec_Msg.publish(rec_msg);
		
		if(/*armed_msg.data &&*/ newBag)
		{
			bag_filename = std::to_string(ros::Time::now().toSec());
			bag.open(bag_filename, rosbag::bagmode::Write);
			newBag = false;
		}
		else if(/*!armed_msg.data &&*/ !newBag)
		{
			//bag.close();
			//newBag = true;
		}
		
		ros::spinOnce();
	}	
	bag.close();
	return 0;
}
