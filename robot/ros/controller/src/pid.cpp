/**
	This script is the high level PID control loop. 
	From subscribing to the imu/data topic the error in the robot's pitch angle is found and a PID gain to compensate
	is computed. This value is published onto the rpm_cmd topic which is then handled by the speed controller.

	This script also subscribes to the pid_tuner topic where users can enter floats for each P,I and D gain variable. 

	@author Alex Cornelio

*/


#include "ros/ros.h"
#include "std_msgs/String.h"
#include <stdio.h>
#include <wiringSerial.h>
#include <wiringPi.h>
#include <sensor_msgs/Imu.h>
#include <tf/LinearMath/Matrix3x3.h>
#include <string>
#include <sstream>
#include <stdlib.h>
#include <iostream>
#include <math.h>

#include "robot_teleop_tuner/pid_values.h"
#include "arduino_feedback/feedback.h"
#include "controller/PidData.h"
#include <std_msgs/Int16.h>

// set frequency in Hz and time difference for the PID sampling rate
#define FREQUENCY 100
#define PID_DELTA 0.01

// set gains
#define PROP_GAIN 10
#define INT_GAIN 0.3
#define DERIV_GAIN 0.1 

#define STOP_PWM 128
#define REFERENCE_PITCH 0

// fix imu angled offset
#define PITCH_FIX 5.5 
// maximum pitch angle for the robot to stop
#define PITCH_THRESHOLD 20


/**
	Class encapsulates basic functionality for all controllers
*/
class Controller
{

	public:
		Controller();
		~Controller();
		void init();

		//ros stuff
  		ros::NodeHandle n;	
		ros::Subscriber sub_imu;
		void IMU_callback(const sensor_msgs::Imu::ConstPtr& msg);

		//IMU variables
		double roll;
		double pitch;
		double yaw;
		double pitch_dot;

		//Other
		int episodes;
		int time_steps;
};


/**
	Constructor, initalise variables and subscribe to imu topic
*/
Controller::Controller()
	:	roll(0.0)
	    ,	pitch(0.0)
	    ,	yaw(0.0)
	    ,   episodes(0)
	    ,   time_steps(0)
{
	sub_imu = n.subscribe("imu/data", 1000, &Controller::IMU_callback, this);
}

/**
	Destructor
*/
Controller::~Controller()
{
	ROS_INFO("Deleting node");
}

/**
	Transform the msg from the imu topic to a pitch in degrees, minusing any offset in the imu. Also get angluar velocity
*/
void Controller::IMU_callback(const sensor_msgs::Imu::ConstPtr& msg)
{

	tf::Quaternion q(msg->orientation.x, msg->orientation.y, msg->orientation.z, msg->orientation.w);
	tf::Matrix3x3(q).getRPY(roll, pitch, yaw);
	pitch = pitch*(180/M_PI) - PITCH_FIX;
	pitch_dot_imu_rad = (msg->angular_velocity.x);
	pitch_dot = -pitch_dot_imu_rad*(180/M_PI);
}		


/**
	PID class encapsulates all required for the PID algorithm, and is derived from the controller class. 
*/
class PID : public Controller
{
	public:
	float kp;
	float ki;
	float kd;

	float kvp;
	float kvi;
	float kvd;

	float pitch_ref;
	float error_prev;
	float integral_sum;
	int encoder_1;
	int encoder_2;
	float rpm1;
	float rpm2;

	controller::PidData msg;
	ros::Subscriber sub_pid;
	ros::Subscriber sub_arduino_data;
	PID(float, float, float);
	~PID();
	float updatePID();
	void init();
	int saturate(int);
	void pid_callback(const robot_teleop_tuner::pid_values::ConstPtr& pid_input);
	void encoder_callback(const arduino_feedback::feedback::ConstPtr& encoder_counts);
	float saturateIntSum(float);
};

/**
	Constructor
	Store P, I, D gains, initalise, subscribe to the pid tuner and arduino data topics
*/
PID::PID(float kp_, float ki_, float kd_):
	pitch_ref(0), kvp(0.1)
{
	kp = kp_;
	ki = ki_;
	kd = kd_;
	this->init();
	ROS_INFO("pid instance created");
	sub_pid = n.subscribe("/pid_tuner", 1000, &PID::pid_callback, this);
	sub_arduino_data = n.subscribe("/arduino_data", 1000, &PID::encoder_callback, this);
}

/**
	Overwrite P, I, D and Vp variables from user using the robot_teleop_tuner
*/
void PID::pid_callback(const robot_teleop_tuner::pid_values::ConstPtr& pid_input)
{
    kp = pid_input->p;
    ki = pid_input->i;
    kd = pid_input->d;
    kvp = pid_input->pv;
}

/**
	Record encoder counters and actual rpms of each motor
*/
void PID::encoder_callback(const arduino_feedback::feedback::ConstPtr& encoder_counts)
{
   encoder_1 = encoder_counts->encoder1;
   encoder_2 = encoder_counts->encoder2;
   rpm1 = encoder_counts->actual_rpm1;
   rpm2 = encoder_counts->actual_rpm2;
   ROS_INFO("rpm 1 %f", rpm1);
   ROS_INFO("rpm 2 %f", rpm2);
 
}

/**
	Destructor
*/
PID::~PID()
{
	ROS_INFO("PID node destroyed");
}

/**
	Initalise PID algorithm 
*/
void PID::init()
{
	error_prev = 0;
	integral_sum = 0;
}


/**
	Compute the PID gains.
*/
float PID::updatePID()
{
	float error,error_v, error_kvp, error_kp, error_ki, error_kd, derivative, pid_cmd;
	
	// error
	error = pitch - REFERENCE_PITCH;
	msg.error = error;

	// proportional gain
	error_kp = error * kp;
	msg.error_proportional = error_kp;

	// integral gain
	integral_sum += error * PID_DELTA;
	integral_sum = saturateIntSum(integral_sum);
	error_ki = integral_sum * ki;
	msg.integral_sum = integral_sum;
	msg.error_integral = error_ki;

	// derivative gain
	derivative = (error - error_prev)/PID_DELTA;
	error_kd = derivative * kd;
	msg.derivative = derivative;
	msg.error_derivative = error_kd;
	msg.error_prev = error_prev;
	error_prev = error;

	// velocity gain - experimental
	// error_v = (rpm1 - rpm2) / 2;	
	// error_kvp = error_v * kvp;
	// error_kvp = 0;
	// msg.error_kvp = error_kvp;	
	// msg.error_v = error_v;
	
	pid_cmd = (error_kp + error_ki + error_kd);// + error_kvp);
	msg.pid_cmd = pid_cmd;	
	return pid_cmd;
}

/**
	Saturate PID command if its over 255 or below 1
*/
int PID::saturate(int pid_cmd)
{
	if (pid_cmd >= 255)
	{
		pid_cmd = 255;
	}else if (pid_cmd <= 1)
	{
		pid_cmd = 1;
	}
	return pid_cmd;
}

/**
	Saturate the integral sum if its too high. 
*/
float PID::saturateIntSum(float integral_sum)
{
	if (integral_sum > 50)
	{
	  integral_sum = 50;
	}
	else if (integral_sum < -50)
	{
	  integral_sum = -50;
	}
	return integral_sum;
}


/**
	Main function
*/
int main(int argc, char **argv)
{
	// config ros initalisation, freq and topics
	ros::init(argc, argv, "control");
	ros::NodeHandle n;
	ros::Rate loop_rate(FREQUENCY); 
	ros::Publisher pid_data = n.advertise<controller::PidData>("/pid_data", 1000);
	ros::Publisher pwm_command = n.advertise<std_msgs::Int16>("/pwm_cmd", 1000);

	PID pid(PROP_GAIN,INT_GAIN,DERIV_GAIN);
	int pid_cmd;
	std::string command;
    std_msgs::Int16 pwm_msg;
	
	// loop until stopped
	while (ros::ok())
	{
		//update pitch
		ros::spinOnce(); 

		pid.time_steps++;
		pid.msg.time_steps = pid.time_steps;

		// compute new pid command and send to topic
		pid_cmd = pid.updatePID() + STOP_PWM;
		pid_cmd = pid.saturate(pid_cmd);
		pid.msg.motor_cmd = pid_cmd;	

		// stop the robot once its passed an large angle, just so it does not destroy itself
		if (std::abs(pid.pitch) > PITCH_THRESHOLD)
		{
			pwm_msg.data = STOP_PWM;
		}else{
			pwm_msg.data = pid_cmd;
		}

		// publish messages
		pwm_command.publish(pwm_msg);
		pid_data.publish(pid.msg);

		loop_rate.sleep();
	}
	return 0;
}
