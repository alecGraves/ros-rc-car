// df -hT /home | grep -o '[0-9]*%' | grep -o '[0-9]*'

#include "ros/ros.h"
#include "std_msgs/Bool.h"

int main(int argc, char **argv)
{

	ros::init(argc, argv, "record_video");

	ros::NodeHandle nh;

	ros::Publisher Full_Msg = nh.advertise<std_msgs::Bool>("full_msg", 1000);
	ros::Publisher Rec_Msg = nh.advertise<std_msgs::Bool>("rec_msg",1000);
	ros::Rate loop_rate(10);

	int counter = 0;	

	while(ros::ok())
	{
		std_msgs::Bool full_msg;
		std_msgs::Bool rec_msg;
		
		full_msg.data = false;
		rec_msg.data = true;	

		Full_Msg.publish(full_msg);
		Rec_Msg.publish(rec_msg);

		ros::spinOnce();
	}	

	return 0;
}
