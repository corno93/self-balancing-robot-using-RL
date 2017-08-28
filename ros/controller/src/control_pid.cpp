#include "ros/ros.h"
#include "std_msgs/String.h"
#include <stdio.h>
#include <wiringSerial.h>
#include <wiringPi.h>
#include <sensor_msgs/Imu.h>
#include <tf/LinearMath/Matrix3x3.h>

#define PID_delta 0.25

class Controller
{

	public:
		Controller();
		~Controller();
		void init();
		void write_serial_command(std::string const& command);

		//ros stuff (subscribe to IMU data topic)	
  		ros::NodeHandle n;	//make private? eh..
		ros::Subscriber sub;
		void IMU_callback(const sensor_msgs::Imu::ConstPtr& msg);
		
		//serialPutchar parameter
		int fd;

		//IMU variables
		double roll;
		double pitch;
		double yaw;
};



Controller::Controller()
	:	roll(0.0)
	    ,	pitch(0.0)
	    ,	yaw(0.0)
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
//	ROS_INFO("IMU Seq: [%d]", msg->header.seq);
//	ROS_INFO("IMU orientation x: [%f], y: [%f], z: [%f], w: [%f]", msg->orientation.x, msg->orientation.y, msg->orientation.z, msg->orientation.w);
	
	tf::Quaternion q(msg->orientation.x, msg->orientation.y, msg->orientation.z, msg->orientation.w);
	//double roll, pitch, yaw;
	tf::Matrix3x3(q).getRPY(roll,this->pitch, yaw);
	ROS_INFO("the pitch in the callback is: %f", pitch*(180/M_PI));

}		

void Controller::write_serial_command(std::string const& command)
{
	fflush(stdout);
	serialPuts(fd, command.c_str());
	
}


class PID : public Controller
{
	public:
	float kp, ki, kd;
	float error_prev, integral_sum;
	PID(float, float, float);
	~PID();
	float updatePID(float ref, float actual);
	void init();
};

PID::PID(float kp_, float ki_, float kd_)
{
	kp = kp_;
	ki = ki_;
	kd = kd_;
	this->init();
	ROS_INFO("pid instance created");
}

PID::~PID()
{
	ROS_INFO("PID node destroyed");
}

void PID::init()
{
	error_prev = 0;
	integral_sum = 0;
}

float PID::updatePID(float ref, float actual)
{
	float error, error_kp, error_ki, error_kd, derivative;
	error = ref - actual;

	error_kp = error * kp;

	integral_sum += error * PID_delta;
	error_ki = integral_sum * ki;

	derivative = (error - error_prev)/PID_delta;
	error_kd = derivative * kd;
	error_prev = error;
	
	return (error_kp + error_ki + error_kd);
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "control");

	ROS_INFO("CONTROL NODE CREATED");
  	//TODO: Determine frequency that it writes.
	ros::NodeHandle n;
	ros::Rate loop_rate(4); //run node at 4 hz

	PID pid(1.0,0.0,0.0);

	std::string command = "-100+200";
	float reference = 0;
	float pid_cmd;
	ROS_INFO("before ros::ok loop");
	while (ros::ok())
	{
		ros::spinOnce(); //update pitch

		pid_cmd = pid.updatePID(reference, pid.pitch);
		ROS_INFO("pid_cmd: %f", pid_cmd);	
		ROS_INFO("pitch in the pid class is: %f", pid.pitch);
		pid.write_serial_command(command);


		loop_rate.sleep();
	}
	return 0;
}


