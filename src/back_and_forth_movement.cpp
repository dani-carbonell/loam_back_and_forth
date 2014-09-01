#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <string.h>

#define PI 3.1415

using namespace std;

class BackAndForthMovement{
	public:
		float pos_limit; // position limit (rad)
		float w_limit; // angular velocity limit (rad/s)
		
		sensor_msgs::JointState joint_command_; // joint to send
		sensor_msgs::JointState joint_state_;   // joint to receive
		
		ros::NodeHandle nh;
		ros::Publisher joint_pub_;
		ros::Subscriber joint_sub_;
		int state_;
		float desired_freq_;
		string name_;
		
		
		
		BackAndForthMovement()
		{
			ROS_INFO("back and forth movement started");
			
			pos_limit = PI / 2;
			w_limit = 0.1;
			state_ = 1;
			desired_freq_ = 30.0;
			name_ = "H42";
			
			joint_pub_ = nh.advertise<sensor_msgs::JointState>("/joint_commands",1);
			joint_sub_ = nh.subscribe<sensor_msgs::JointState>("/joint_states", 1, &BackAndForthMovement::jointStateCallback, this);
			
			controlLoop();
		}
		
		void controlLoop()
		{
			ros::Rate r(desired_freq_);
			while(ros::ok)
			{
				if(state_==1)
					initialize();
					
				ros::spinOnce();
				r.sleep();
			}
		}
		
		/*!
		 *  /brief Initializes joint position to 0
		 */
		void initialize()
		{
			// if not centered
			if(joint_state_.position[0] != 0.0)
			{
				sensor_msgs::JointState joint_center;
				joint_center.name[0] = name_;
				joint_center.position[0] = 0.0;
				// @todo: set velocity depending the direction it has to move
				joint_pub_.publish(joint_center);
			}
			// if centered
			else
			{
				turnLeft();
			}
		}
		
		void turnLeft()
		{
			
		}
		
		void turnRight()
		{
			
		}
		
		void jointStateCallback(const sensor_msgs::JointState joint_state)
		{
			if(joint_state.name.size() == 0)
				ROS_WARN("Joint state received is empty");
			else if(joint_state.name.size() > 1)
				ROS_WARN("Node only supports one joint");
				
			// copy joint state msg
			joint_state_.name[0] = joint_state.name[0];
			joint_state_.position[0] = joint_state.position[0];
			joint_state_.velocity[0] = joint_state.velocity[0];
			joint_state_.effort[0] = joint_state.effort[0];
		}
		
	
}; // end class


int main(int argc, char** argv)
{
	ros::init(argc, argv, "back_and_forth_movement");
	BackAndForthMovement backAndForthMovement;
		
	//while(ros::ok)
	//{
			//ros::spinOnce();
			
	//}
}
