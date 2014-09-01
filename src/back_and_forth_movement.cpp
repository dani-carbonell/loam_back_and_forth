#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <string.h>

#define PI 3.1415

using namespace std;

class BackAndForthMovement{
	public:
		float pos_limit_; // position limit (rad)
		float w_limit_; // angular velocity limit (rad/s)
		
		sensor_msgs::JointState joint_command_; // joint to send
		sensor_msgs::JointState joint_state_;   // joint to receive
		
		ros::NodeHandle nh;
		ros::Publisher joint_pub_;
		ros::Subscriber joint_sub_;
		int state_;
		double desired_freq_;
		string name_;
		bool joint_received_;
		ros::Rate *loop_rate_;
		
		
		
		BackAndForthMovement()
		{
			ROS_INFO("back and forth movement started");
			
			pos_limit_ = PI / 2;
			w_limit_ = 0.1;
			state_ = 1;
			desired_freq_ = 100.0;
			name_ = "H42";
			joint_received_ = false;
			loop_rate_ = new ros::Rate(desired_freq_);
			
			joint_pub_ = nh.advertise<sensor_msgs::JointState>("/joint_commands",1);
			joint_sub_ = nh.subscribe<sensor_msgs::JointState>("/joint_states", 1, &BackAndForthMovement::jointStateCallback, this);

			controlLoop();
		}
		
		
		void controlLoop()
		{
			//ros::Rate loop_rate_(30);
			//loop_rate_.reset();
			while(ros::ok())
			{
				ROS_INFO("debug"); 
				// only move the joint when a joint_state msg is received
				if(joint_received_)
				{
					
					if(state_ == 1)
						initialize();
					else if(state_ == 2)
						turnLeft();
					else if(state_ == 3)
						turnRight();
				}
				joint_received_ = false;
				
				
				ros::spinOnce();
				loop_rate_->sleep();
				
			}
			ros::shutdown();
		}
		
		/*!
		 *  /brief Initializes joint position to 0
		 */
		void initialize()
		{
			ROS_INFO("Initializing joint"); 
			
			// if not centered
			if(joint_state_.position[0] != 0.0)
			{
				
				sensor_msgs::JointState center_joint;
				center_joint.name[0] = name_;
				center_joint.position[0] = 0.0;
				// @todo: set velocity depending the direction it has to move
				joint_pub_.publish(center_joint);
			}
			// if centered
			else
			{
				state_ = 2; // turn left
			}
		}
		
		void turnLeft()
		{
			ROS_INFO("Turning left");
			
			// limit not reached
			if(joint_state_.position[0] < pos_limit_)
			{
				sensor_msgs::JointState left_joint;
				left_joint.name[0] = name_;
				left_joint.position[0] = pos_limit_;
				left_joint.velocity[0] = w_limit_;
				joint_pub_.publish(left_joint);
			}
			// limit reached
			else 
			{
				state_ = 3; // turn right
			}
		}
		
		void turnRight()
		{
			ROS_INFO("Turning right");
			
			// limit not reached
			if(joint_state_.position[0] > -pos_limit_)
			{
				sensor_msgs::JointState right_joint;
				right_joint.name[0] = name_;
				right_joint.position[0] = -pos_limit_;
				right_joint.velocity[0] = -w_limit_;
				joint_pub_.publish(right_joint);
			}
			// limit reached
			else 
			{
				state_ = 3; // turn right
			}
			
		}
		
		void jointStateCallback(const sensor_msgs::JointState joint_state)
		{
			ROS_INFO("joint callback");
			joint_received_ = true;
			
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
		
	return 0;

}
