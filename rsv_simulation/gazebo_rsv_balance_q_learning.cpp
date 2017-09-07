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

class reinforcement_learning
{
  public:
    float alpha;
    int episode_num;
    int time_step;
    int wins;
    int loses;
    float discount_factor;
    float epsilon;

    char actions[ACTIONS] = {30,20,10,5,-5,-10,-20,-30};
    int rewards[STATES] = {0,50,100,1000,1000,100,50,0};
    reinforcement_learning();
    ~reinforcement_learning();

    std::vector<std::vector<float> > Q;

    char virtual choose_action(char) = 0;
    void TD_update(char, char, char, int);
    char get_state(float);
    char get_next_state(char, char);
    int get_reward(char);
};

reinforcement_learning::reinforcement_learning()
  :  Q(STATES, std::vector<float>(ACTIONS,0)), 
     episode_num(0), time_step(0), wins(0),
     loses(0), discount_factor(0.3), alpha(0.3),
     epsilon(0.3)
{
}

reinforcement_learning::~reinforcement_learning()
{
}

int reinforcement_learning::get_reward(char next_state)
{
  return rewards[next_state];
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

char reinforcement_learning::get_state(float pitch)
{
  if (pitch < -15)
  {
    return 0;
  }else if (pitch < -10 && pitch > -15)
  {
    return 1;
  }else if (pitch < -5 && pitch > -10)
  {
    return 2;
  }else if (pitch < 0 && pitch > -5)
  {
    return 3;
  }else if (pitch < 5 && pitch > 0)
  {
    return 4;
  }else if (pitch < 10 && pitch > 5)
  {
    return 5;
  }else if (pitch < 15 && pitch > 10)
  {
    return 6;
  }else if (pitch > 15)
  {
    return 7;
  }else
  {
    return 0;
  }
}


char reinforcement_learning::get_next_state(char cur_state, char action)
{


}

class q_learning: public reinforcement_learning
{
  public:
    q_learning();
    ~q_learning();
    std::vector<float> q_row;
    std::vector<int> max_value_idxs;

    int test_counter;

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
 // std::vector<float> q_row;
 // std::vector<int> max_value_idxs;

  random_num = fabs((rand()/(float)(RAND_MAX)));	//random num between 0 and 1
  //ROS_INFO("random num: %f", random_num);

  
  if (random_num < epsilon)
  {
    //pick randomly
    random_choice = rand()%ACTIONS;
    ROS_INFO("random choice: %d", random_choice);
    return random_choice;
  }
  else
  {
    //pick best
    ROS_INFO("picks best");
    this->q_row = Q[curr_state];

    max_q = *std::max_element(q_row.begin(), q_row.end());
//    ROS_INFO("max_q: %f", max_q);    
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
	//std::fill(max_value_idxs.being(), max_value_idxs.end(),0);
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
// Only execute control loop on specified rate
  if (seconds_since_last_update > RL_DELTA)
  {

    ROS_INFO("seconds since last update %f", seconds_since_last_update);

    this->last_update_time_ += common::Time(RL_DELTA);


    this->updateIMU();
    this->updateOdometry();
    this->publishOdometry();
    this->publishWheelJointState();

  switch (this->current_mode_)
  {
    case BALANCE:

	//increment timestep
	controller.time_step++;





	// Check whether robot has fallen. Increase episode num is it has.
	if (std::abs(this->imu_pitch_*(180/M_PI)) > 35)
	{
		this->restart_delta = parent_->GetWorld()->GetSimTime();
		if (this->restart_delta - this->restart_delta_prev > 0.1)
		{

		ROS_INFO("RESTART SIM - pitch is: %f!", this->imu_pitch_*(180/M_PI));
		controller.episode_num++;
		controller.time_step = 0;
		ROS_INFO("EPISODE NUM: %d", controller.episode_num);
		}
		this->restart_delta_prev = this->restart_delta;
	} 

	//get state value
	pitch = this->imu_pitch_*(180/M_PI);
//	ROS_INFO("pitch: %f", pitch);
	curr_state = controller.get_state(pitch);
//	ROS_INFO("current state: %d", curr_state);

	// select action
	action_idx = controller.choose_action(curr_state);	
//	ROS_INFO("action selected: %d", action_idx);	


	//take action
	this->joints_[LEFT]->SetVelocity(0,-controller.actions[action_idx]);
	this->joints_[RIGHT]->SetVelocity(0, controller.actions[action_idx]);

	//get next state
	next_state = 6;	
 	
	//get reward
	reward = controller.get_reward(next_state);

	//TD update
	controller.TD_update(curr_state, action_idx, next_state, reward);

	//cycle n repeat
	if(reward == 1000)
	{
		controller.wins++;
	}
	else if (reward == 0)
	{
		controller.loses++;
	}
	curr_state = next_state;	

	
	
//      this->joints_[LEFT]->SetForce(0, -this->u_control_[balance_control::tauL]);
//      this->joints_[RIGHT]->SetForce(0, this->u_control_[balance_control::tauR]);
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
