//THIS IS THE CONTROL NODE


#include "ros/ros.h"
#include "std_msgs/String.h"
#include <stdio.h>
#include <wiringSerial.h>
#include <wiringPi.h>

class Controller
{

	public:
		//Replace type of message with message containing the encoder values
		void init();
		void write_serial_command(char command);
	
		//Ros Node Handle
  		ros::NodeHandle n;
		
		//serialPutchar parameter
		int fd;	
};



/*char command Motors::step()
{
	

}*/


void Controller::init()
{
	//Ros Init (subscribe to IMU topic)
//	ros::Subscriber sub = n.subscribe("encoder_topic", 1000, this->ecoder_callback);

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

void Controller::write_serial_command(char command)
{
	fflush(stdout);
	serialPutchar(fd, command);
	
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "control");


  //TODO: Determine frequency that it writes.
  ros::NodeHandle n;
  ros::Rate loop_rate(10);

  Controller PID_control;
  PID_control.init();

  char command;
  command = 20;

  while (ros::ok())
  {
//	command = motors.step();
	PID_control.write_serial_command(command);
  //  	ROS_INFO("%s", msg.data.c_str());

	ros::spinOnce();

	loop_rate.sleep();
  }
  return 0;
}


