//THIS IS THE PID CONTROL NODE


#include "ros/ros.h"
#include "std_msgs/String.h"
#include <stdio.h>
#include <wiringSerial.h>
#include <wiringPi.h>
#include <sensor_msgs/Imu.h>



class Controller
{

	public:
		Controller();
		~Controller();
		void init();
		void write_serial_command(char command);
	
		//Ros Node Handle (subscribes to IMU /data topic)
  		ros::NodeHandle n;	//make private? eh..
		void IMU_callback(const sensor_msgs::Imu::ConstPtr& msg);
		
		//serialPutchar parameter
		int fd;	
};



Controller::Controller()
{
	//Ros Init (subscribe to IMU topic)
	ros::Subscriber sub = n.subscribe("/data", 1000, &Controller::IMU_callback, this);

	//Serial Init
	if ((fd = serialOpen("/dev/ttyACM1",9600))<0)
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
	ROS_INFO("I heard: ");
}		

void Controller::write_serial_command(char command)
{
	fflush(stdout);
	serialPutchar(fd, command);
	
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "control");

	ROS_INFO("CONTROL NODE CREATED");
  	//TODO: Determine frequency that it writes.
	ros::NodeHandle n;
	ros::Rate loop_rate(10);

	Controller PID_control;

	char command;
	command = 20;
	
	while (ros::ok())
	{
		PID_control.write_serial_command(command);

		ros::spin();

		loop_rate.sleep();
	}
	return 0;
}


