/*
 *  Gazebo - Outdoor Multi-Robot Simulator
 *  Copyright (C) 2003  
 *     Nate Koenig & Andrew Howard
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program; if not, write to the Free Software
 *  Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 */

#ifndef SERVO_PLUGIN_H
#define SERVO_PLUGIN_H

#include <gazebo/common/Plugin.hh>
#include <gazebo/common/Time.hh>
#if (GAZEBO_MAJOR_VERSION < 8)
#include <gazebo/math/Quaternion.hh>
#endif

// ROS 
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/QuaternionStamped.h>
#include <sensor_msgs/JointState.h>

// Boost
#include <boost/thread.hpp>
#include <boost/bind.hpp>

namespace gazebo
{

class ServoPlugin : public ModelPlugin
{

public:
  ServoPlugin();
  virtual ~ServoPlugin();

protected:
  virtual void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf);
  virtual void Init();
  virtual void Reset();
  virtual void Update();

private:
  void CalculateVelocities();
  void publish_joint_states();

private:
  /// \brief The parent World
  physics::WorldPtr world;

  // Simulation time of the last update
  common::Time prevUpdateTime;

  bool enableMotors;

  struct Servo {
    std::string name;
#if (GAZEBO_MAJOR_VERSION >= 8)
    ignition::math::Vector3d axis;
#else
    math::Vector3 axis;
#endif
    physics::JointPtr joint;
    float velocity;
    Servo() : velocity() {}
  } servo[3];

  unsigned int countOfServos;
  unsigned int orderOfAxes[3];
  unsigned int rotationConv;
  sensor_msgs::JointState joint_state;

  // parameters
  std::string robotNamespace;
  std::string topicName;
  std::string jointStateName;
  common::Time controlPeriod;

  float proportionalControllerGain;
  float derivativeControllerGain;
  double maximumVelocity;
  float maximumTorque;

  // ROS STUFF
  ros::NodeHandle* rosnode_;
  ros::Publisher jointStatePub_;
  ros::Subscriber sub_;
  tf::TransformListener* transform_listener_;

  // Custom Callback Queue
  ros::CallbackQueue queue_;
//  boost::thread* callback_queue_thread_;
//  void QueueThread();

  // DiffDrive stuff
  void cmdCallback(const geometry_msgs::QuaternionStamped::ConstPtr& cmd_msg);

  boost::mutex mutex;
  geometry_msgs::QuaternionStamped::ConstPtr current_cmd;
#if (GAZEBO_MAJOR_VERSION >= 8)
  ignition::math::Quaterniond rotation_;
#else
  math::Quaternion rotation_;
#endif

  // Pointer to the update event connection
  event::ConnectionPtr updateConnection;
};

}

#endif
