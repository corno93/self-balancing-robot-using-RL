
#include "ros/ros.h"
#include "std_msgs/String.h"
#include <stdio.h>
#include <wiringSerial.h>
#include <wiringPi.h>
#include <sensor_msgs/Imu.h>
#include <tf/LinearMath/Matrix3x3.h>

class Controller
{

	public:
		Controller();
		~Controller();
		void init();
		void write_serial_command(char* command);

		//ros stuff (subscribe to IMU data topic)	
  		ros::NodeHandle n;	//make private? eh..
		ros::Subscriber sub;
		void IMU_callback(const sensor_msgs::Imu::ConstPtr& msg);
		
		//serialPutchar parameter
		int fd;	
};



Controller::Controller()
{
	//Ros Init (subscribe to IMU topic)
	sub = n.subscribe("data", 1000, &Controller::IMU_callback, this);
	
	//Serial Init
	if ((fd = serialOpen("/dev/ttyACM0",9600))<0)
	{
		ROS_INFO("Unable to open serial device");
	}	
	if (wiringPiSetup() == -1)
	{
		ROS_INFO("Unable to start wiringPi");
	}

}

Controller::~Controller()
{
	ROS_INFO("Deleting node");
}


void Controller::IMU_callback(const sensor_msgs::Imu::ConstPtr& msg)
{
	ROS_INFO("IMU Seq: [%d]", msg->header.seq);
	ROS_INFO("IMU orientation x: [%f], y: [%f], z: [%f], w: [%f]", msg->orientation.x, msg->orientation.y, msg->orientation.z, msg->orientation.w);
	
	tf::Quaternion q(msg->orientation.x, msg->orientation.y, msg->orientation.z, msg->orientation.w);
	double roll, pitch, yaw;
	tf::Matrix3x3(q).getRPY(roll, pitch, yaw);
	ROS_INFO("THE PITCH IS: %f", pitch*(180/M_PI));

}		

void Controller::write_serial_command(char* command)
{
	fflush(stdout);
	serialPuts(fd, command);
	
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "control");

	ROS_INFO("CONTROL NODE CREATED");
  	//TODO: Determine frequency that it writes.
	ros::NodeHandle n;
	ros::Rate loop_rate(10);

	Controller PID_control;

	char* command = "+100+100";
	
	while (ros::ok())
	{
		PID_control.write_serial_command(command);

		ros::spin();

		loop_rate.sleep();
	}
	return 0;
}


