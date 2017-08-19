/*********************************************************************
 *  Copyright (c) 2015 Robosavvy Ltd.
 *  Author: Vitor Matos
 *
 *********************************************************************/

#ifndef GAZEBO_RSV_BALANCE_GAZEBO_RSV_BALANCE_H
#define GAZEBO_RSV_BALANCE_GAZEBO_RSV_BALANCE_H

#include <string>
#include <vector>
#include <map>

#include <ros/ros.h>
#include <ros/callback_queue.h>

#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>

#include <gazebo/common/common.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo_plugins/gazebo_ros_utils.h>

#include <std_msgs/Float64.h>
#include <sensor_msgs/JointState.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>

#include <std_srvs/Empty.h>

#include <rsv_balance_msgs/State.h>
#include <rsv_balance_msgs/SetMode.h>
#include <rsv_balance_msgs/SetInput.h>

#include "rsv_balance_gazebo_control/balance_gazebo_control.h"

namespace gazebo
{

class Joint;
class Entity;
/**
* Gazebo Plugin which controls the balance platform just like the real thing.
*/
class GazeboRsvBalance: public ModelPlugin
{
  enum
  {
    RIGHT,
    LEFT
  };
  enum OdomSource
  {
    ENCODER = 0,
    WORLD = 1
  };
  enum Mode
  {
    PARK = rsv_balance_msgs::SetModeRequest::PARK,
    TRACTOR = rsv_balance_msgs::SetModeRequest::TRACTOR,
    BALANCE = rsv_balance_msgs::SetModeRequest::BALANCE
  };
  public:
    GazeboRsvBalance();
    ~GazeboRsvBalance();
    void Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf);
    void Reset();

  protected:
    virtual void UpdateChild();
    virtual void FiniChild();

  private:
    //  Gazebo information
    GazeboRosPtr gazebo_ros_;
    physics::ModelPtr parent_;
    sdf::ElementPtr sdf_;
    event::ConnectionPtr update_connection_;

    //  Plugin
    bool alive_;
    ros::CallbackQueue queue_;
    boost::thread callback_queue_thread_;
    void QueueThread();

    //  Publishers and subscribers
    std::string command_topic_;
    std::string odom_topic_;
    ros::Publisher odometry_publisher_;
    ros::Publisher state_publisher_;
    ros::Publisher joint_state_publisher_;
    ros::Subscriber cmd_vel_subscriber_;
    ros::Subscriber cmd_tilt_subscriber_;
    ros::ServiceServer set_mode_server_;
    ros::ServiceServer set_input_server_;
    ros::ServiceServer reset_override_server_;
    ros::ServiceServer reset_odom_server_;
    boost::shared_ptr<tf::TransformBroadcaster> transform_broadcaster_;

    //  Node callbacks
    void cmdVelCallback(const geometry_msgs::Twist::ConstPtr& cmd_msg);
    void cmdTiltCallback(const std_msgs::Float64::ConstPtr& cmd_tilt);
    bool setMode(rsv_balance_msgs::SetMode::Request  &req);
    bool setInput(rsv_balance_msgs::SetInput::Request  &req);
    bool resetOverride(std_srvs::Empty::Request  &req);
    bool resetOdom(std_srvs::Empty::Request  &req);

    //  Node configurations
    bool publish_odom_tf_;
    bool publish_wheel_joint_;
    std::string base_frame_id_;
    std::string odom_frame_id_;
    OdomSource odom_source_;
    bool publish_state_;
    double publish_state_rate_;
    double publish_diagnostics_rate_;

    double wheel_separation_;
    double wheel_radius_;

    std::map<std::string, Mode> mode_map_;
    Mode current_mode_;

    //  Robot
    std::vector<physics::JointPtr> joints_;
    double x_desired_;
    double rot_desired_;
    double tilt_desired_;
    double imu_pitch_;
    double imu_dpitch_;
    double feedback_v_;
    double feedback_w_;
    math::Vector3 odom_offset_pos_;
    math::Vector3 odom_offset_rot_;

    void resetVariables();
    nav_msgs::Odometry odom_;
    void updateIMU();
    void updateOdometry();
    void resetOdometry();
    void publishOdometry();
    void publishWheelJointState();

    // Control
    double update_rate_;
    double update_period_;
    common::Time last_update_time_;
    balance_control::BalanceControl state_control_;
    double *u_control_;
};

}  // namespace gazebo

#endif  // GAZEBO_RSV_BALANCE_GAZEBO_RSV_BALANCE_H

