#include <ros/ros.h>
#include <sensor_msgs/JointState.h>

int main(int argc, char** argv)
{
	ros::init(argc, argv, "back_and_forth_movement");
	ROS_INFO("back and forth movement started");
	
	ros::NodeHandle nh;
	ros::Publisher joint_pub = nh.advertise<sensor_msgs::JointState>("/joint_commands",1000);
	
	
	while(ros::ok)
	{
			ros::spinOnce();
	}
}
