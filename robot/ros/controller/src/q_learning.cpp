/**
	This script is the high level Q-learning control loop. 
	From subscribing to the imu/data topic the robot knows what state it is in (pitch angle and pitch angle velocity).
	The robot then trys to learn an optimal value function that aims to achieve stabilisation by 
	playing around with rpm actions and receiving rewards. 

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
#include <numeric>


#include <q_model_install/Q_state.h>
#include "robot_teleop_tuner/pid_values.h"
#include "controller/Q_state.h"
#include "controller/State.h"
#include "controller/PidData.h"
#include <std_msgs/Int16.h>

#include <deque>
#include <vector>
#include <time.h>
#include <cmath>
#include <algorithm>

//uses the gazebo sim model
#define EPSILON 0.6
#define ALPHA 0.6
#define GAMMA 0.6

#define FREQUENCY 25
#define RL_DELTA 0.04
#define STOP_RPM 0

#define PITCH_FIX 5.5
#define REFERENCE_PITCH -1.0

#define PITCH_THRESHOLD 6.5
#define ACTIONS 7
#define ACTIONS_HALF 3
#define ACTION_BIAS 3
#define RUNNING_AVG 3

// actions in units rpm
float actions[ACTIONS] =  {-45, -30,-15,  0, 15,  30, 45}; 

#define MAX_EPISODE 150

// 2D state space
#define STATE_NUM_PHI 11
#define STATE_NUM_PHI_D 11

//Define pitch angle states
float phi_states[STATE_NUM_PHI] = {-5, -3, -2, -1, -0.5, 0, 0.5, 1, 2, 3, 5};
//Define pitch angle velocity states
float phi_d_states[STATE_NUM_PHI_D] = {-2, -1.5, -1, -0.6, -0.2,  0, 0.2, 0.6, 1, 1.5, 2};



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
	Class encapsulates all todo with RL algorithms
*/
class RL
{
  public:
  	// RL related variables
    float alpha;
    int episode_num;
    int time_steps;
    int wins;
    int loses;
    float discount_factor;
    float epsilon;
    float pitch_dot;
    float prev_pitch;
    char current_state;
    char next_state;
    controller::State msg;
    float action;
    char action_idx;
    float reward_per_ep;
    int running_avg_cntr;
    float pitch_dot_filtered;	
    std::vector<std::vector<float> > Q;
    ros::Publisher q_state_publisher;

    // ros variables
    ros::NodeHandle n;	
    ros::Subscriber sub_q;
    std::deque<float> pitch_dot_data;

    RL();
    ~RL();

    char virtual choose_action(char) = 0;
    void TD_update(char, int, char, float);
    char get_state(float, float);
    float get_reward(float, float);
    void read_model(void);
    void Q_callback(const q_model_install::Q_state::ConstPtr& q_model);
    float running_avg_pitch_dot(void);
};

/**
	Constructor. 
	Initalise everything
*/
RL::RL()
  :  Q((STATE_NUM_PHI+1)*(STATE_NUM_PHI_D+1), std::vector<float>(ACTIONS,0)), 
     episode_num(0), time_steps(0), wins(0),
     loses(0), discount_factor(GAMMA), alpha(ALPHA),
     epsilon(EPSILON), pitch_dot(0.0), prev_pitch(0.0),
     reward_per_ep(0.0), pitch_dot_data(RUNNING_AVG, 0.0), running_avg_cntr(0)


/**
	Destructor
*/
RL::~RL()
{
}

/**
	Use a running average filter on the pitch velocity
*/
float RL::running_avg_pitch_dot(void)
{
  if (pitch_dot_data.size() < RUNNING_AVG){
  pitch_dot_data.push_back(pitch_dot);
  running_avg_cntr++;
  }else{  
   running_avg_cntr = RUNNING_AVG;
   pitch_dot_data.pop_front();
   pitch_dot_data.push_back(pitch_dot);
  }

 return (std::accumulate(pitch_dot_data.begin(), pitch_dot_data.end(), 0.0)) / running_avg_cntr;


}


/**
	Compute the reward for the state transition
*/
float RL::get_reward(float pitch, float pitch_dot_imu)
{
	float squared_error_pitch = pow((pitch - REFERENCE_PITCH),2);
  	float squared_error_pitch_dot = pow((pitch_dot_imu - 0), 2);
 	float reward_sgn;
  	if (pitch_dot_imu < 0 && pitch < REFERENCE_PITCH)
    	squared_error_pitch_dot = -squared_error_pitch_dot;
  	else if (pitch_dot_imu > 0 && pitch > REFERENCE_PITCH)
    	squared_error_pitch_dot = -squared_error_pitch_dot;

  	return (-squared_error_pitch + squared_error_pitch_dot);

  	//return (1 - std::exp(pow(pitch - REFERENCE_PITCH,2)));

}
 
/**
	Virtual function. This function will be different for every RL algorithm
*/
char RL::choose_action(char)
{
}

/**
	Compute the temporal difference update for the state transition
*/
void RL::TD_update(char curr_state, int action, char next_state, float reward)
{
	int max_action_idx;
	float td_target;
	float td_error;
	float Q_val;

	// get index value of Q next_state row with max reward value
	max_action_idx = distance(Q[next_state].begin(), max_element(Q[next_state].begin(), Q[next_state].end()));

	// compute update and write to Q at current state
	td_target = reward + discount_factor*Q[next_state][max_action_idx];
	td_error = td_target - Q[curr_state][action];
	Q[curr_state][action]+= td_error*alpha;

	// collect all data 
	msg.max_action_idx = max_action_idx;
	msg.td_target = td_target;
	msg.td_error = td_error;
	msg.td_update = Q[curr_state][action];
	msg.alpha = alpha;
	msg.discount_factor = discount_factor;    
}


/**
	Return the state number the robot has landed in
*/
char RL::get_state(float pitch_, float pitch_dot_)
{
  int i, j;
  i = 0;
  j = 0;

  // find what pitch angle state its in
  for (int phi_idx = 0; phi_idx < STATE_NUM_PHI; phi_idx++)
  {
    if (pitch_ <= phi_states[phi_idx])
    {
      i = phi_idx;
      break;
    }else if (pitch_ > phi_states[STATE_NUM_PHI - 1])
    {
      i = STATE_NUM_PHI;
      break;
     }
   }

  // find what pitch angular velocity state its in
  for (int phi_d_idx = 0; phi_d_idx < STATE_NUM_PHI_D; phi_d_idx++)
  {
    if (pitch_dot_ <= phi_d_states[phi_d_idx])
    {
      j = phi_d_idx;
      break;
    }else if (pitch_dot_ > phi_d_states[STATE_NUM_PHI_D - 1])
    {
      j = STATE_NUM_PHI_D;
      break;
    }
 }

  return( j + (STATE_NUM_PHI_D + 1) * i);
}


/**
	Increment episode count and re-initalise everything.
*/
void RL::next_ep(void)
{
	ROS_INFO("RESTART SIM - pitch is: %f!", controller.pitch);
	episode_num++;
	msg.episodes = controller.episode_num;

	//initalise appropriate variables
	time_steps = 0;
	controller.prev_pitch = 0;
	controller.pitch_dot = 0;
	controller.reward_per_ep = 0;

	//clear message
	msg.pitch = 0;
	msg.pitch_dot = 0;
	msg.action = 0;
	msg.action_idx = 0;
	msg.current_state = 0;
	msg.next_state = 0;
	msg.td_update = 0;
	msg.td_target = 0;
	msg.td_error = 0;

}



/**
	Class encapsulates all to do with the Q learning algorithm.
	It is derived from the Controller and RL classes
*/
class QLearning: public Controller, public RL
{
  	public:
    QLearning();
    ~QLearning();
    std::vector<float> q_row;
    std::vector<int> max_value_idxs;
    
    char choose_action(char);
    void take_action(int);
};

/**
	Constructor
*/
QLearning::QLearning()
{
    this->q_row.resize(6);
} 

/**
	Descructor
*/
QLearning::~QLearning()
{
}

/**
	QLearning chooses a random action if it wants to explore or the best action if it wants to exploit
	Return action number
*/
char QLearning::choose_action(char curr_state)
{
	int random_choice;
	float random_num;
	float max_q;
	int action_choice;
	char position_bias;
	char position_lower_bound;
	char position_upper_bound;

	// generate random number (0 - 1) to decide whether to explore or exploit
	random_num = fabs((rand()/(float)(RAND_MAX)));	
	msg.action_choice = random_num;

	this->q_row = Q[curr_state];
	std::vector<float> q_row_final;  

	// implement 'position bias' that ensures the robot will only choose an action that will turn itself in the correct direction
	if (pitch >= 0)
	{
		position_bias = ACTION_BIAS;
		position_lower_bound = ACTION_BIAS;
		position_upper_bound = ACTIONS;
	}else{

		position_bias = 0;
		position_lower_bound = 0;
		position_upper_bound = ACTION_BIAS + 1;
	}

	// explore or exploit
	if (random_num < epsilon)
	{
		// explore
		random_choice = rand()%(ACTIONS_HALF) + position_bias;
		ROS_INFO("random action choice: %d", random_choice);
		msg.random_action = random_choice;
		return random_choice;
	}
	else
	{
		//exploit
		this->q_row = Q[curr_state];
		std::vector<float>::const_iterator first  = q_row.begin() + position_lower_bound;
		std::vector<float>::const_iterator end  = q_row.begin() + position_upper_bound;
		std::vector<float> q_row_final(first, end);

		max_q = *std::max_element(q_row_final.begin(), q_row_final.end());

		for (int i = 0; i < q_row_final.size(); i++)
		{
		    if (max_q == q_row_final[i])
		    {
		        max_value_idxs.push_back(i);     
		  	}
	    }

		action_choice = max_value_idxs[0] + position_bias;
		max_value_idxs.clear();
		return action_choice;
	
	}
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
	ros::Publisher pwm_command = n.advertise<std_msgs::Int16>("/pwm_cmd", 1000);
	ros::Publisher state_publisher = n.advertise<controller::State>("/State", 1000);

	char state;
	float reward;
    std_msgs::Int16 pwm_msg;
	double restart_delta_prev, restart_delta, epsilon_delta_prev, epsilon_delta;

	QLearning controller;
	

	// loop until stopped
	while (ros::ok())
	{
		ros::spinOnce(); //update pitch
		controller.msg.pitch = controller.pitch;
	  	controller.msg.pitch_dot = controller.pitch_dot;
	  	controller.msg.pitch_dot_imu = controller.pitch_dot_imu;	


	  //check for next episode - when the robot has fallen past the PITCH_THRESHOLD
	if (std::abs(controller.pitch) > PITCH_THRESHOLD)
		{
		  restart_delta = ros::Time::now().toSec();
		  if (restart_delta - restart_delta_prev > 0.5)
			{
				controller.next_ep()
			}
			restart_delta_prev = restart_delta;
	   	    pwm_msg.data = STOP_RPM;
		    pwm_command.publish(pwm_msg);
			controller.motors = false;
			controller.pitch_dot_data.clear();
			controller.running_avg_cntr = 0;

		}

	// apply control if segway is still in pitch range
	if (std::abs(controller.pitch) <= PITCH_THRESHOLD && controller.motors == true)
	{
		// class member variables
		controller.pitch_dot = (controller.pitch - controller.prev_pitch)/RL_DELTA;

		// message member variables
		controller.msg.pitch = controller.pitch;
		controller.msg.pitch_dot = controller.pitch_dot;
		controller.msg.epsilon = controller.epsilon;
		controller.msg.time_steps = controller.time_steps;
		controller.msg.error = controller.pitch - REFERENCE_PITCH;
		controller.msg.prev_pitch = controller.prev_pitch;
		controller.msg.pitch_dot_imu = controller.pitch_dot_imu;	

		// get state of the system
		controller.pitch_dot_filtered = controller.running_avg_pitch_dot();
		controller.msg.pitch_dot_filtered = controller.pitch_dot_filtered;
		//			state = controller.get_state(controller.pitch, controller.pitch_dot_filtered);
		state = controller.get_state(controller.pitch, controller.pitch_dot_imu);


		// debugging data:	
		// ROS_INFO("episode: %d", controller.episode_num);
		// ROS_INFO("time step: %d", controller.time_steps);	
		// ROS_INFO("pitch: %f", controller.pitch);
		// ROS_INFO("pitch prev: %f", controller.prev_pitch);
	 	// ROS_INFO("pitch dot: %f", controller.pitch_dot);
		// ROS_INFO("pitch dot filtered %f", controller.pitch_dot_filtered);
		// ROS_INFO("pitch dot imu  %f", controller.pitch_dot_imu);
		// ROS_INFO("state: %d", state);

		// first iteration
		if (controller.time_steps < 1)
		{
		   	// set the current state
		    controller.current_state = state;

			// select action
			controller.action_idx = controller.choose_action(controller.current_state);
			controller.action = actions[controller.action_idx];

			// take action (ie. publish action)
			pwm_msg.data = controller.action;  
			pwm_command.publish(pwm_msg);

		}
		else if (controller.time_steps >= 1)
		{
			// set the  next state 
			controller.next_state = state;
			controller.msg.next_state = controller.next_state;

			// get reward
			reward = controller.get_reward(controller.pitch, controller.pitch_dot_imu);
			controller.msg.reward = reward;
			controller.reward_per_ep+=reward;
			controller.msg.reward_per_ep = controller.reward_per_ep;

			//TD update
			controller.TD_update(controller.current_state, controller.action_idx, controller.next_state, reward);

			// publish state data
			state_publisher.publish(controller.msg);

			// Now move from the next state to the current state
			controller.current_state = controller.next_state;

			// select action
			controller.action_idx = controller.choose_action(controller.current_state);
			controller.action = actions[controller.action_idx];

			// take action (ie. publish action)
			pwm_msg.data = controller.action;
			pwm_command.publish(pwm_msg);

		}
		 
		// set prev pitch
		controller.prev_pitch = controller.pitch;

		// add data to message (state, action, action_idx,  to msg
		controller.msg.current_state = controller.current_state;
		controller.msg.action = controller.action;
		controller.msg.action_idx = controller.action_idx;

		//increment timestep
		controller.time_steps++;

		//decrease epsilon every 30 eps
		if (controller.episode_num % 30 == 0 && controller.episode_num > 1)
		{
			controller.epsilon = controller.epsilon/10;
	  	    controller.msg.epsilon = controller.epsilon;
	  	    epsilon_delta_prev = epsilon_delta;
			
		}
		
		// check if times up
		if (controller.episode_num == MAX_EPISODE)
		{
		 	ROS_INFO("SIMULATION COMPLETE AT %d EPISODES", controller.episode_num);
			pwm_msg.data = STOP_RPM;
		}

	}	
	loop_rate.sleep();
	}
	return 0;
}
