/*********************************************************************
 *  Copyright (c) 2015 Robosavvy Ltd.
 *  Author: Vitor Matos
 *
 *  Based on diff_drive_plugin
 *
 *  MODIFICATION: 
 *  - control segway about pitch angle using a RL q-learning controller

 *********************************************************************/

#include "gazebo_rsv_balance/gazebo_rsv_balance.h"

#include <string>
#include <map>

#include <ros/ros.h>
#include <sdf/sdf.hh>

#include <vector>
#include <time.h>
#include <cmath>
#include <algorithm>

#define RL_DELTA 0.05
#define FREQ 20
#define STATES 8
#define ACTIONS 8
#define WHEEL_RADIUS 0.19

// 2D state space
#define STATE_NUM 9
char phi_states[STATE_NUM] = {-9, -6, -3, -1.5, 0, 1.5, 3, 6, 9};
char phi_d_states[STATE_NUM] = {-30,-20, -10,-5, 0, 5, 10, 20,30};



// TODO: put this in other cpp and hpp files
class reinforcement_learning
{
  public:
    float alpha;
    int episode_num;
    int time_steps;
    int wins;
    int loses;
    float discount_factor;
    float epsilon;
    float pitch;
    float pitch_dot;
    float prev_pitch;
    rsv_balance_msgs::State msg;

    char actions[ACTIONS] = {-80,-60,-40,-20,20,40,60, 80};
    int rewards[STATES] = {0,50,100,1000,1000,100,50,0};
    reinforcement_learning();
    ~reinforcement_learning();

    std::vector<std::vector<float> > Q;

    char virtual choose_action(char) = 0;
    void TD_update(char, char, char, int);
    char get_state(float, float);
    char get_next_state(float,float, char);
    int get_reward(char);
};

reinforcement_learning::reinforcement_learning()
  :  Q((STATE_NUM+1)*(STATE_NUM+1), std::vector<float>(ACTIONS,0)), 
     episode_num(0), time_steps(0), wins(0),
     loses(0), discount_factor(0.3), alpha(0.3),
     epsilon(0.3), pitch_dot(0.0), prev_pitch(0.0)
{
}

reinforcement_learning::~reinforcement_learning()
{
}

int reinforcement_learning::get_reward(char next_state)
{
  for (int i = 0; i < REWARD_1; i++)
  {
    if (next_state == reward_1[i])
    {
      return 1000;
    }
  }
  for (int i = 0; i < REWARD_2; i++)
  {
    if (next_state == reward_2[i])
    {
    return 500;
    }
  }
  for (int i = 0; i < REWARD_3; i++)
  {
  if (next_state == reward_3[i])
    {
      return 100;
    }
  }
  for (int i = 0; i < REWARD_4; i++)
  {
  if (next_state == reward_4[i])
    {
      return 10;
    }
  }
  return -100;
}

char reinforcement_learning::choose_action(char)
{
}

void reinforcement_learning::TD_update(char curr_state, char action, char next_state, int reward)
{
  int max_action_idx;
  float td_target;
  float td_error;

  //next_state_idx = get_state_index(next_state);
  max_action_idx = distance(Q[next_state].begin(), max_element(Q[next_state].begin(), Q[next_state].end()));
  td_target = reward + discount_factor*Q[next_state][max_action_idx];
  td_error = td_target - Q[curr_state][action];
  Q[curr_state][action]+= td_error*alpha;

}

char reinforcement_learning::get_state(float pitch, float pitch_dot)
{
  char i, j;
  for (char phi_idx = 0; phi_idx < STATE_NUM; phi_idx++)
  {
    if (pitch <= phi_states[phi_idx])
    {
      i = phi_idx;
      break;
    }else if (pitch > phi_states[STATE_NUM])
    {
      i = STATE_NUM;
    }
   }
  for (char phi_d_idx = 0; phi_d_idx < STATE_NUM; phi_d_idx++)
  {
    if (pitch_dot <= phi_d_states[phi_d_idx])
    {
      j = phi_d_idx;
      break;
    }else if (pitch_dot > phi_d_states[STATE_NUM])
    {
      j = STATE_NUM;
    }
  }
  return( j + (STATE_NUM + 1) * i);
}



char reinforcement_learning::get_next_state(float pitch, float pitch_dot, char action_idx)
{
  float x_current[6][1] = {
    {0},
    {0},
    {pitch},
    {pitch_dot},
    {-actions[action_idx]},
    {actions[action_idx]}};

  char next_state;
  const char u = 1;
  float A_x_current[6][1];
  float x_next[6][1];
  float next_pitch;
  float next_pitch_dot;
  // A*x_current
  for (int i = 1; i < 6; i++)
  {
    for (int j = 1; j < 6; j++)
      {
        A_x_current[i][0]+= A_model[i][j] * x_current[j][0];
      }
  } 
  //x_current + (A*current + B*u)*dt
  for (int i = 1; i < 6; i ++)
  {
    x_next[i][0] = x_current[i][0] + (A_x_current[i][0] + B[i][0])*RL_DELTA;
  }
 
  next_pitch = x_next[2][0];
  next_pitch_dot = x_next[3][0];
  std::cout<<"next pitch"<<next_pitch<<std::endl;
  std::cout<<"next pd " <<next_pitch_dot<<std::endl;
  msg.next_pitch = next_pitch;
  msg.next_pitch_dot = next_pitch_dot;
  next_state = get_state(next_pitch, next_pitch_dot);  

  return next_state;

}

class q_learning: public reinforcement_learning
{
  public:
    q_learning();
    ~q_learning();
    std::vector<float> q_row;
    std::vector<int> max_value_idxs;

    char choose_action(char);
    void take_action(int);
};

q_learning::q_learning()
{
    this->q_row.resize(6);
}

q_learning::~q_learning()
{
}

char q_learning::choose_action(char curr_state)
{
  int random_choice;
  float random_num;
  float max_q;
  int action_choice;

  random_num = fabs((rand()/(float)(RAND_MAX)));	//random num between 0 and 1


  
  if (random_num < epsilon)
  {
    //pick randomly
    random_choice = rand()%ACTIONS;
    return random_choice;
  }
  else
  {
    //pick best
    this->q_row = Q[curr_state];

    max_q = *std::max_element(q_row.begin(), q_row.end());
    for (int i = 0; i < q_row.size(); i++)
      {
        if (max_q == q_row[i])
          {
            max_value_idxs.push_back(i);     
	  }
      }
    if (max_value_idxs.size() == 1)
      {
	action_choice = max_value_idxs[0];
        max_value_idxs.clear();
	return max_value_idxs[0];
      }
	else if (max_value_idxs.size() > 1)
      { 
//	ROS_INFO("random choice for repeated bests");
//	ROS_INFO("size: %d", max_value_idxs.size());
        action_choice = rand()%(max_value_idxs.size());
	max_value_idxs.clear();
//	ROS_INFO("rnadom choice is: %d", action_choice);
        return action_choice;
      }
  }
}



q_learning controller;

namespace gazebo
{

GazeboRsvBalance::GazeboRsvBalance() {}

GazeboRsvBalance::~GazeboRsvBalance() {}

void GazeboRsvBalance::Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf)
{
  this->parent_ = _parent;
  this->sdf_ = _sdf;
  this->gazebo_ros_ = GazeboRosPtr(new GazeboRos(_parent, _sdf, "RsvBalancePlugin"));

  this->gazebo_ros_->isInitialized();

  // Obtain parameters from rosparam
  this->gazebo_ros_->getParameter<bool>(this->publish_odom_tf_, "publishOdomTF", true);
  this->gazebo_ros_->getParameter<bool>(this->publish_wheel_joint_, "publishWheelJointState", true);

  this->gazebo_ros_->getParameter<std::string>(this->command_topic_, "commandTopic", "cmd_vel");
  this->gazebo_ros_->getParameter<std::string>(this->odom_topic_, "odomTopic", "odom");
  this->gazebo_ros_->getParameter<std::string>(this->base_frame_id_, "baseFrameId", "base_link");
  this->gazebo_ros_->getParameter<std::string>(this->odom_frame_id_, "odomFrameId", "odom");

  this->gazebo_ros_->getParameter<bool>(this->publish_state_, "publishState", true);
  this->gazebo_ros_->getParameter<double>(this->update_rate_, "updateRate", 50.0);
  this->gazebo_ros_->getParameter<double>(this->publish_state_rate_, "publishStateRate", 50);
  this->gazebo_ros_->getParameter<double>(this->publish_diagnostics_rate_, "publishDiagnosticsRate", 1);

  std::map<std::string, OdomSource> odom_options;
  odom_options["encoder"] = ENCODER;
  odom_options["world"] = WORLD;
  this->gazebo_ros_->getParameter<OdomSource>(this->odom_source_, "odomSource", odom_options, WORLD);

  this->mode_map_["park"] = PARK;
  this->mode_map_["tractor"] = TRACTOR;
  this->mode_map_["balance"] = BALANCE;
  this->gazebo_ros_->getParameter<Mode>(this->current_mode_, "startMode", this->mode_map_, BALANCE);

  if (!this->sdf_->HasElement("wheelSeparation") || !this->sdf_->HasElement("wheelRadius") )
  {
    ROS_ERROR("RsvBalancePlugin - Missing <wheelSeparation> or <wheelDiameter>, Aborting");
    return;
  }
  else
  {
    this->gazebo_ros_->getParameter<double>(this->wheel_separation_, "wheelSeparation");
    this->gazebo_ros_->getParameter<double>(this->wheel_radius_, "wheelRadius");
  }

  this->joints_.resize(2);
  this->joints_[LEFT] = this->gazebo_ros_->getJoint(this->parent_, "leftJoint", "left_joint");
  this->joints_[RIGHT] = this->gazebo_ros_->getJoint(this->parent_, "rightJoint", "right_joint");

  // Control loop timing
  if (this->update_rate_ > 0.0)
  {
    this->update_period_ = 1.0 / this->update_rate_;	//CHANGED TO 5/10 = 0.5SEC = 2HZ
  }
  else
  {
    ROS_WARN("RsvBalancePlugin - Update rate < 0. Update period set to: 0.1. ");
    this->update_period_ = 0.1;
  }
  this->last_update_time_ = this->parent_->GetWorld()->GetSimTime();
  // Variable that control RL algorithm updates
  //this->rl_update_time = this->parent_->GetWorld()->GetSimTime();
 

  // Command velocity subscriber
  ros::SubscribeOptions so = ros::SubscribeOptions::create<geometry_msgs::Twist>(this->command_topic_, 5,
              boost::bind(&GazeboRsvBalance::cmdVelCallback, this, _1),
              ros::VoidPtr(), &this->queue_);
  this->cmd_vel_subscriber_ = this->gazebo_ros_->node()->subscribe(so);
  ROS_INFO("%s: Subscribed to %s!", this->gazebo_ros_->info(), this->command_topic_.c_str());
  // Tilt equilibrium subscriber
  so = ros::SubscribeOptions::create<std_msgs::Float64>("tilt_equilibrium", 5,
              boost::bind(&GazeboRsvBalance::cmdTiltCallback, this, _1),
              ros::VoidPtr(), &this->queue_);
  this->cmd_tilt_subscriber_ = this->gazebo_ros_->node()->subscribe(so);
  ROS_INFO("%s: Subscribed to %s!", this->gazebo_ros_->info(), "tilt_equilibrium");

  // Odometry publisher
  this->odometry_publisher_ = this->gazebo_ros_->node()->advertise<nav_msgs::Odometry>(this->odom_topic_, 10);
  ROS_INFO("%s: Advertise odom on %s !", this->gazebo_ros_->info(), this->odom_topic_.c_str());
  // State publisher
  this->state_publisher_ = this->gazebo_ros_->node()->advertise<rsv_balance_msgs::State>("state", 10);
  ROS_INFO("%s: Advertise system state on %s !", this->gazebo_ros_->info(), "state");
  // Joint state publisher
  this->joint_state_publisher_ = this->gazebo_ros_->node()->advertise<sensor_msgs::JointState>("joint_states", 10);
  ROS_INFO("%s: Advertise joint_states!", gazebo_ros_->info());

  this->state_publisher_ = this->gazebo_ros_->node()->advertise<rsv_balance_msgs::State>("state", 10);
  


  // Service for changing operating mode
  ros::AdvertiseServiceOptions ao = ros::AdvertiseServiceOptions::create<rsv_balance_msgs::SetMode>("set_mode",
        boost::bind(&GazeboRsvBalance::setMode, this, _1),
        ros::VoidPtr(),
        &this->queue_);
  this->set_mode_server_ = this->gazebo_ros_->node()->advertiseService(ao);
  // Service for chaning input source
  ao = ros::AdvertiseServiceOptions::create<rsv_balance_msgs::SetInput>("set_input",
        boost::bind(&GazeboRsvBalance::setInput, this, _1),
        ros::VoidPtr(),
        &this->queue_);
  this->set_input_server_ = this->gazebo_ros_->node()->advertiseService(ao);
  // Reset override service
  ao = ros::AdvertiseServiceOptions::create<std_srvs::Empty>("reset_override",
        boost::bind(&GazeboRsvBalance::resetOverride, this, _1),
        ros::VoidPtr(),
        &this->queue_);
  this->reset_override_server_ = this->gazebo_ros_->node()->advertiseService(ao);
  // Reset odom service
  ao = ros::AdvertiseServiceOptions::create<std_srvs::Empty>("reset_odom",
        boost::bind(&GazeboRsvBalance::resetOdom, this, _1),
        ros::VoidPtr(),
        &this->queue_);
  this->reset_odom_server_ = this->gazebo_ros_->node()->advertiseService(ao);

  // Broadcaster for TF
  this->transform_broadcaster_ = boost::shared_ptr<tf::TransformBroadcaster>(new tf::TransformBroadcaster());

  this->resetVariables();
  this->state_control_.resetControl();
  this->x_desired_ = 0;
  this->rot_desired_ = 0;
  this->tilt_desired_ = 0;
  this->u_control_ = this->state_control_.getControl();


  this->alive_ = true;
  // start custom queue
  this->callback_queue_thread_ = boost::thread(boost::bind(&GazeboRsvBalance::QueueThread, this));
  // listen to the update event (broadcast every simulation iteration)
 
 this->update_connection_ = event::Events::ConnectWorldUpdateBegin(boost::bind(&GazeboRsvBalance::UpdateChild, this));

}

/*!
* \brief Resets simulation variables.
* 
* Used when starting and when gazebo reset world or model
*/
void GazeboRsvBalance::resetVariables()
{
  this->x_desired_ = 0;
  this->rot_desired_ = 0;
  this->odom_offset_pos_ = math::Vector3(0, 0, 0);
  this->odom_offset_rot_ = math::Vector3(0, 0, 0);
}

/*!
* \brief Sets platform operating mode.
*/
bool GazeboRsvBalance::setMode(rsv_balance_msgs::SetMode::Request  &req)
{
  // Ugly implementation means bad concept
  switch (req.mode)
  {
    case rsv_balance_msgs::SetModeRequest::PARK:
      ROS_INFO("%s: Mode: park", this->gazebo_ros_->info());
      this->current_mode_ = PARK;
      break;
    case rsv_balance_msgs::SetModeRequest::TRACTOR:
      ROS_INFO("%s: Mode: tractor", this->gazebo_ros_->info());
      this->current_mode_ = TRACTOR;
      break;
    case rsv_balance_msgs::SetModeRequest::BALANCE:
      ROS_INFO("%s: Mode: balance", this->gazebo_ros_->info());
      this->current_mode_ = BALANCE;
      break;
    default:
      return false;
  };
  return true;
}

/*!
* \brief Just exposes service. Not used in simulation
*/
bool GazeboRsvBalance::setInput(rsv_balance_msgs::SetInput::Request  &req)
{
  // In simulation input should always be serial communication.
  ROS_INFO("%s: Input: %d", this->gazebo_ros_->info(), req.input);
  return true;
}

/*!
* \brief Just exposes service. Not used in simulation
*/
bool GazeboRsvBalance::resetOverride(std_srvs::Empty::Request  &req)
{
  // In simulation we don't have RC override, nothing to reset
  ROS_INFO("%s: Reset Override", this->gazebo_ros_->info());
  return true;
}

/*!
* \brief Service to reset odometry.
*/
bool GazeboRsvBalance::resetOdom(std_srvs::Empty::Request  &req)
{
  ROS_INFO("%s: Reset Odom", this->gazebo_ros_->info());
  this->resetOdometry();
  return true;
}

/*!
* \brief Callback to cmd_vel
*/
void GazeboRsvBalance::cmdVelCallback(const geometry_msgs::Twist::ConstPtr& cmd_msg)
{
  this->x_desired_ = cmd_msg->linear.x;
  this->rot_desired_ = cmd_msg->angular.z;
}

/*!
* \brief Callback to cmd_tilt
*/
void GazeboRsvBalance::cmdTiltCallback(const std_msgs::Float64::ConstPtr& cmd_tilt)
{
  this->tilt_desired_ = cmd_tilt->data;
}

/*!
* \brief Gets pitch angle values directly from Gazebo world
*/
void GazeboRsvBalance::updateIMU()
{
  // Store pitch and dpitch
  math::Pose pose = this->parent_->GetWorldPose();
  math::Vector3 veul = this->parent_->GetRelativeAngularVel();
  this->imu_pitch_ = pose.rot.GetPitch();
  this->imu_dpitch_ = veul.y;
}

/*!
* \brief Resets odometry by adding offset to WORLD odometry, and resetting odometry values.
*/
/** @todo Actually implement it */
void GazeboRsvBalance::resetOdometry()
{
  math::Pose pose = this->parent_->GetWorldPose();
  this->odom_offset_pos_ = pose.pos;
  this->odom_offset_rot_.z = pose.rot.GetYaw();
}

/*!
* \brief Updates odometry, from Gazebo world or from encoders.
*/
/** @todo Implement encoder odometry */
void GazeboRsvBalance::updateOdometry()
{
  double ang_velocity_left = -this->joints_[LEFT]->GetVelocity(0);
  double ang_velocity_right = this->joints_[RIGHT]->GetVelocity(0);

  this->feedback_v_ = this->wheel_radius_/2.0 * (ang_velocity_right + ang_velocity_left);
  this->feedback_w_ = this->wheel_radius_/this->wheel_separation_ * (ang_velocity_right - ang_velocity_left);

  if (odom_source_ == WORLD)
  {
    math::Pose pose = this->parent_->GetWorldPose();
    math::Vector3 velocity_linear = this->parent_->GetRelativeLinearVel();
    math::Vector3 velocity_angular = this->parent_->GetRelativeAngularVel();
    // TODO(vmatos): reset odometry by setting an offset in world
    // this->odom_.pose.pose.position.x = pose.pos[0] - this->odom_offset_pos_[0];
    // this->odom_.pose.pose.position.y = pose.pos[1] - this->odom_offset_pos_[1];
    // this->odom_.pose.pose.position.z = pose.pos[2] - this->odom_offset_pos_[2];
    this->odom_.pose.pose.position.x = pose.pos[0];
    this->odom_.pose.pose.position.y = pose.pos[1];
    this->odom_.pose.pose.position.z = pose.pos[2];
    // tf::Quaternion qt;
    // qt.setRPY(pose.rot.GetRoll(), pose.rot.GetPitch(), pose.rot.GetYaw() - this->odom_offset_rot_[2]);
    // qt.setRPY(pose.rot.GetRoll(), pose.rot.GetPitch(), pose.rot.GetYaw());
    this->odom_.pose.pose.orientation.x = pose.rot.x;
    this->odom_.pose.pose.orientation.y = pose.rot.y;
    this->odom_.pose.pose.orientation.z = pose.rot.z;
    this->odom_.pose.pose.orientation.w = pose.rot.w;
    this->odom_.twist.twist.linear.x = velocity_linear[0];
    this->odom_.twist.twist.linear.y = velocity_linear[1];
    this->odom_.twist.twist.angular.z = velocity_angular.z;
  }
  else
  {
    ROS_WARN("%s - Odometry from other sources not yet supported.", this->gazebo_ros_->info());
  }
}

/*!
* \brief Publishes odometry and desired tfs
*/
/** @todo User configurable covariance */
void GazeboRsvBalance::publishOdometry()
{
  ros::Time current_time = ros::Time::now();
  std::string odom_frame = this->gazebo_ros_->resolveTF(this->odom_frame_id_);
  std::string base_frame = this->gazebo_ros_->resolveTF(this->base_frame_id_);

  // set odometry covariance
  this->odom_.pose.covariance[0] = 0.00001;
  this->odom_.pose.covariance[7] = 0.00001;
  this->odom_.pose.covariance[14] = 1000000000000.0;
  this->odom_.pose.covariance[21] = 1000000000000.0;
  this->odom_.pose.covariance[28] = 1000000000000.0;
  this->odom_.pose.covariance[35] = 0.001;
  // set header
  this->odom_.header.stamp = current_time;
  this->odom_.header.frame_id = odom_frame;
  this->odom_.child_frame_id = base_frame;
  //  Publish odometry
  this->odometry_publisher_.publish(this->odom_);

  if (this->publish_odom_tf_)
  {
    tf::Vector3 vt;
    tf::Quaternion qt;
    vt = tf::Vector3(this->odom_.pose.pose.position.x,
                     this->odom_.pose.pose.position.y,
                     this->odom_.pose.pose.position.z);
    qt = tf::Quaternion(this->odom_.pose.pose.orientation.x, this->odom_.pose.pose.orientation.y,
                        this->odom_.pose.pose.orientation.z, this->odom_.pose.pose.orientation.w);
    tf::Transform base_to_odom(qt, vt);
    this->transform_broadcaster_->sendTransform(
            tf::StampedTransform(base_to_odom, current_time, odom_frame, base_frame));
  }
}

/*!
* \brief Publishes wheel joint_states
*/
void GazeboRsvBalance::publishWheelJointState()
{
  ros::Time current_time = ros::Time::now();

  sensor_msgs::JointState joint_state;
  joint_state.header.stamp = current_time;
  joint_state.name.resize(joints_.size());
  joint_state.position.resize(joints_.size());

  for (int i = 0; i < 2; i++)
  {
    physics::JointPtr joint = this->joints_[i];
    math::Angle angle = joint->GetAngle(0);
    joint_state.name[i] = joint->GetName();
    joint_state.position[i] = angle.Radian();
  }
  this->joint_state_publisher_.publish(joint_state);
}

/*!
* \brief Called when Gazebo resets world
*/
void GazeboRsvBalance::Reset()
{
  this->resetVariables();
  this->state_control_.resetControl();
  // Reset control
  this->last_update_time_ = this->parent_->GetWorld()->GetSimTime();
  this->current_mode_ = BALANCE;
  this->imu_pitch_ = 0;
  this->imu_dpitch_ = 0;
  this->feedback_v_ = 0;
  this->feedback_w_ = 0;
}

/*!
* \brief Gazebo step update
*/
void GazeboRsvBalance::UpdateChild()
{
  common::Time current_time = this->parent_->GetWorld()->GetSimTime();
  double seconds_since_last_update = (current_time - this->last_update_time_).Double();
  char action_idx;  
  float pitch;
  char curr_state;
  char next_state;
  int reward;
  float next_pitch;
  float next_pitch_dot;

    this->updateIMU();
    this->updateOdometry();
    this->publishOdometry();
    this->publishWheelJointState();

    pitch = this->imu_pitch_*(180/M_PI);
      if (std::abs(pitch) > 35)
      {

	this->restart_delta = parent_->GetWorld()->GetSimTime();
	if (this->restart_delta - this->restart_delta_prev > 0.1)
	{

	  ROS_INFO("RESTART SIM - pitch is: %f!", this->imu_pitch_*(180/M_PI));
	  controller.episode_num++;
          controller.msg.episodes = controller.episode_num;
	  controller.time_steps = 0;
	  controller.prev_pitch = 0;
	  controller.pitch_dot = 0;
	}
	this->restart_delta_prev = this->restart_delta;
	
      }


  // Only execute control loop on specified rate
  if (seconds_since_last_update > RL_DELTA)
  {

    ROS_INFO("seconds since last update %f", seconds_since_last_update);

    this->last_update_time_ += common::Time(RL_DELTA);

   switch (this->current_mode_)
    {
     case BALANCE:


       // apply control if segway is still in pitch range
      if (std::abs(pitch) <=35 && std::abs(pitch) >=0)
      {
	controller.pitch = pitch;
	controller.msg.time_steps = controller.time_steps;
	controller.msg.error = controller.pitch - 0;
	controller.msg.prev_pitch = controller.prev_pitch;
	

	//get state value
	ROS_INFO("pitch: %f", controller.pitch);
	ROS_INFO("pitch dot: %f", controller.pitch_dot);
 	controller.msg.pitch = controller.pitch;
	curr_state = controller.get_state(controller.pitch, controller.pitch_dot);
	controller.msg.current_state = curr_state;
	ROS_INFO("current state: %d", curr_state);

	// select action
	action_idx = controller.choose_action(curr_state);
	ROS_INFO("action idx %d and action: %d", action_idx, controller.actions[action_idx]);	
	controller.msg.action = controller.actions[action_idx];

	//take action
	this->joints_[LEFT]->SetForce(0,-controller.actions[action_idx]);
	this->joints_[RIGHT]->SetForce(0, controller.actions[action_idx]);

	//get next state
	next_state = controller.get_next_state(controller.pitch, controller.pitch_dot, action_idx);
	controller.msg.next_state = next_state;
	//get reward
	reward = controller.get_reward(next_state);
	controller.msg.reward = reward;
	ROS_INFO("reward is %d", reward);	

	//TD update
	controller.TD_update(curr_state, action_idx, next_state, reward);

	// publish Q(s,a) matrix
	


	//cycle n repeat
	if(reward == 1000)
	{
		controller.wins++;
		controller.msg.wins = controller.wins;
	}
	else if (reward == 0)
	{
		controller.loses++;
		controller.msg.loses = controller.loses;
	}

	//pitch dot
	controller.pitch_dot = (controller.pitch - controller.prev_pitch)/RL_DELTA;
	controller.msg.pitch_dot = controller.pitch_dot;

	// publish important data
	this->state_publisher_.publish(controller.msg);

	// prev pitch
	controller.prev_pitch = controller.pitch;

	//increment timestep
	controller.time_steps++;

      }	
	
      break;
    case TRACTOR:
      this->joints_[LEFT]->SetVelocity(0, -( (2.0*this->x_desired_ - this->rot_desired_*this->wheel_separation_)
                                                                                        / (this->wheel_radius_*2.0)));
      this->joints_[RIGHT]->SetVelocity(0, ( (2.0*this->x_desired_ + this->rot_desired_*this->wheel_separation_)
                                                                                        / (this->wheel_radius_*2.0)));
      break;
    case PARK:
      this->joints_[LEFT]->SetVelocity(0, 0);
      this->joints_[RIGHT]->SetVelocity(0, 0);
      this->joints_[LEFT]->SetForce(0, 0);
      this->joints_[RIGHT]->SetForce(0, 0);
      break;
  };
}	//end of if
}

/*!
* \brief Called by gazebo upon exiting
*/
void GazeboRsvBalance::FiniChild()
{
  this->alive_ = false;
  this->queue_.clear();
  this->queue_.disable();
  this->gazebo_ros_->node()->shutdown();
  this->callback_queue_thread_.join();
}

void GazeboRsvBalance::QueueThread()
{
  static const double timeout = 0.01;
  while (this->alive_ && this->gazebo_ros_->node()->ok())
  {
    this->queue_.callAvailable(ros::WallDuration(timeout));
  }
}

GZ_REGISTER_MODEL_PLUGIN(GazeboRsvBalance)
}  // namespace gazebo

