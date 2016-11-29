#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <rosbag/bag.h>
#include <sensor_msgs/Image.h>
#include <std_msgs/Bool.h>
#include <std_msgs/UInt16.h>
#include <string>

using namespace std;

rosbag::Bag bag;
std_msgs::UInt16 steering_msg;
std_msgs::Bool moving;
sensor_msgs::ImageConstPtr camera_msg;
ros::Publisher image_pub;
bool newBag = true;

void downscale_image(const sensor_msgs::Image::ConstPtr& original_image)
{
	//Convert from the ROS image message to a CvImage suitable for working with OpenCV for processing
	cv_bridge::CvImagePtr cv_ptr;
	try
	{
		//Always copy, returning a mutable CvImage
		//OpenCV expects color images to use BGR channel order.
		cv_ptr = cv_bridge::toCvCopy(original_image, sensor_msgs::image_encodings::BGR8);
	}
	catch (cv_bridge::Exception& e)
	{
		//if there is an error during conversion, display it
		ROS_ERROR("color_tracker.cpp::cv_bridge exception: %s", e.what());
		return;
	}
	//downscale resolution by 1/10:
	cv::resize(cv_ptr->image, cv_ptr->image, cv::Size(), 0.1, 0.1);
	//flip the image (the camera was mounted upside-down):
	cv::flip(cv_ptr->image, cv_ptr->image, -1);//-1 for 2-axis flip
	
	//store to global variable:
	camera_msg = cv_ptr->toImageMsg();
}

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
	downscale_image(msg);
	if(moving.data && !newBag)
    {
		image_pub.publish(camera_msg);
        bag.write("usb_cam/image_raw",ros::Time::now(), camera_msg);
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

	image_pub = nh.advertise<sensor_msgs::Image>("stored_image", 1);
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
