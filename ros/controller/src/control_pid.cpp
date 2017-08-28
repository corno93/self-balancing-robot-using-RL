#include "ros/ros.h"
#include "std_msgs/String.h"
#include <stdio.h>
#include <wiringSerial.h>
#include <wiringPi.h>
#include <sensor_msgs/Imu.h>
#include <tf/LinearMath/Matrix3x3.h>
#include <string>


#define PID_delta 0.25


namespace patch
{
	template < typename T > std::string to_string( const T& n)
	{
		std::ostringstream stm;
		stm << n;
		return stm.str();
	}
}



class Controller
{

	public:
		Controller();
		~Controller();
		void init();
		void write_serial_command(std::string const& command);
		std::string motor_cmd_generator(float);

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
	tf::Matrix3x3(q).getRPY(roll, pitch, yaw);
	//ROS_INFO("the pitch in the callback is: %f", pitch*(180/M_PI));
	pitch = pitch*(180/M_PI);
}		

std::string Controller::motor_cmd_generator(float cmd)
{
	std::string m1, m2, result;
	float cmd_abs = std::abs(cmd);
	if (std::abs(cmd) >= 100)
	{
		m1 = patch::to_string(cmd_abs);
		m2 = m1;
	} else if (std::abs(cmd) >= 10)
	{
		m1 = "0" + patch::to_string(cmd_abs);
		m2 = m1;
	} else if (std::abs(cmd) >= 1)
	{
		m1 = "00" + patch::to_string(cmd_abs);
		m2 = m1;
	}
	if (cmd > 0)
	{
		result = "+" + m1 + "+" + m2;
	}else
	{
		result = "-" + m1 + "-" + m2;
	}
	return result;

}

void Controller::write_serial_command(std::string const& command)
{
	fflush(stdout);
	serialPuts(fd, command.c_str());
	
}


class PID : public Controller
{
	public:
	float kp;
	float ki;
	float kd;
	float pitch_ref;
	float error_prev;
	float integral_sum;
	PID(float, float, float);
	~PID();
	float updatePID();
	void init();
};

PID::PID(float kp_, float ki_, float kd_):
	pitch_ref(0)
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

float PID::updatePID()
{
	float error, error_kp, error_ki, error_kd, derivative;
	
	error = pitch_ref - pitch;

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

	ros::NodeHandle n;
	ros::Rate loop_rate(4); //run node at 4 hz

	PID pid(1.0,0.0,0.0);

	std::string command;
	float pid_cmd;
		
	while (ros::ok())
	{
		ros::spinOnce(); //update pitch

		//pid_cmd = pid.updatePID();
		pid_cmd = 50;
		ROS_INFO("pid_cmd: %f", pid_cmd);	
		ROS_INFO("pitch in the pid class is: %f", pid.pitch);
		command = pid.motor_cmd_generator(pid_cmd);
		ROS_INFO("command gen: %s", command);
		pid.write_serial_command(command);


		loop_rate.sleep();
	}
	return 0;
}


