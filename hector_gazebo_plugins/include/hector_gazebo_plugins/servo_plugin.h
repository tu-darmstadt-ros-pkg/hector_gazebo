//=================================================================================================
// Copyright (c) 2012, Johannes Meyer, TU Darmstadt
// All rights reserved.

// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//     * Redistributions of source code must retain the above copyright
//       notice, this list of conditions and the following disclaimer.
//     * Redistributions in binary form must reproduce the above copyright
//       notice, this list of conditions and the following disclaimer in the
//       documentation and/or other materials provided with the distribution.
//     * Neither the name of the Flight Systems and Automatic Control group,
//       TU Darmstadt, nor the names of its contributors may be used to
//       endorse or promote products derived from this software without
//       specific prior written permission.

// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
// ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
// WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
// DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER BE LIABLE FOR ANY
// DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
// (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
// LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
// ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
// SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//=================================================================================================

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
