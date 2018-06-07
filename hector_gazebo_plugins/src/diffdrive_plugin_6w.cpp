//=================================================================================================
// Copyright (c) 2011, Stefan Kohlbrecher, TU Darmstadt
// All rights reserved.

// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//     * Redistributions of source code must retain the above copyright
//       notice, this list of conditions and the following disclaimer.
//     * Redistributions in binary form must reproduce the above copyright
//       notice, this list of conditions and the following disclaimer in the
//       documentation and/or other materials provided with the distribution.
//     * Neither the name of the Simulation, Systems Optimization and Robotics
//       group, TU Darmstadt nor the names of its contributors may be used to
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

/**
 * Based on diffdrive_plugin by Nathan Koenig, Andrew Howard and Daniel Hewlett
 */

#include <algorithm>
#include <assert.h>

#include <hector_gazebo_plugins/diffdrive_plugin_6w.h>
#include <gazebo/common/Events.hh>
#include <gazebo/physics/physics.hh>

#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <boost/bind.hpp>

#include <gazebo/gazebo_config.h>

namespace gazebo {

enum
{
  FRONT_LEFT,
  FRONT_RIGHT,
  MID_LEFT,
  MID_RIGHT,
  REAR_LEFT,
  REAR_RIGHT,
  NUM_WHEELS
};

// Constructor
DiffDrivePlugin6W::DiffDrivePlugin6W()
{
}

// Destructor
DiffDrivePlugin6W::~DiffDrivePlugin6W()
{
#if (GAZEBO_MAJOR_VERSION >= 8)
  updateConnection.reset();
#else
  event::Events::DisconnectWorldUpdateBegin(updateConnection);
#endif
  delete transform_broadcaster_;
  rosnode_->shutdown();
  callback_queue_thread_.join();
  delete rosnode_;
}

// Load the controller
void DiffDrivePlugin6W::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
{
  world = _model->GetWorld();

  // default parameters
  namespace_.clear();
  topic_ = "cmd_vel";
  wheelSep = 0.34;
  wheelDiam = 0.15;
  torque = 10.0;

  // load parameters
  if (_sdf->HasElement("robotNamespace"))
    namespace_ = _sdf->GetElement("robotNamespace")->GetValue()->GetAsString();

  if (_sdf->HasElement("topicName"))
    topic_ = _sdf->GetElement("topicName")->GetValue()->GetAsString();

  if (_sdf->HasElement("bodyName"))
  {
    link_name_ = _sdf->GetElement("bodyName")->GetValue()->GetAsString();
    link = _model->GetLink(link_name_);
  } else {
    link = _model->GetLink();
    link_name_ = link->GetName();
  }

  // assert that the body by link_name_ exists
  if (!link)
  {
    ROS_FATAL("DiffDrivePlugin6W plugin error: bodyName: %s does not exist\n", link_name_.c_str());
    return;
  }

  if (_sdf->HasElement("frontLeftJoint"))  joints[FRONT_LEFT]  = _model->GetJoint(_sdf->GetElement("frontLeftJoint")->GetValue()->GetAsString());
  if (_sdf->HasElement("frontRightJoint")) joints[FRONT_RIGHT] = _model->GetJoint(_sdf->GetElement("frontRightJoint")->GetValue()->GetAsString());
  if (_sdf->HasElement("midLeftJoint"))    joints[MID_LEFT]    = _model->GetJoint(_sdf->GetElement("midLeftJoint")->GetValue()->GetAsString());
  if (_sdf->HasElement("midRightJoint"))   joints[MID_RIGHT]   = _model->GetJoint(_sdf->GetElement("midRightJoint")->GetValue()->GetAsString());
  if (_sdf->HasElement("rearLeftJoint"))   joints[REAR_LEFT]   = _model->GetJoint(_sdf->GetElement("rearLeftJoint")->GetValue()->GetAsString());
  if (_sdf->HasElement("rearRightJoint"))  joints[REAR_RIGHT]  = _model->GetJoint(_sdf->GetElement("rearRightJoint")->GetValue()->GetAsString());

  if (!joints[FRONT_LEFT])  ROS_FATAL("diffdrive_plugin_6w error: The controller couldn't get front left joint");
  if (!joints[FRONT_RIGHT]) ROS_FATAL("diffdrive_plugin_6w error: The controller couldn't get front right joint");
  if (!joints[MID_LEFT])    ROS_FATAL("diffdrive_plugin_6w error: The controller couldn't get mid left joint");
  if (!joints[MID_RIGHT])   ROS_FATAL("diffdrive_plugin_6w error: The controller couldn't get mid right joint");
  if (!joints[REAR_LEFT])   ROS_FATAL("diffdrive_plugin_6w error: The controller couldn't get rear left joint");
  if (!joints[REAR_RIGHT])  ROS_FATAL("diffdrive_plugin_6w error: The controller couldn't get rear right joint");

  if (_sdf->HasElement("wheelSeparation"))
    _sdf->GetElement("wheelSeparation")->GetValue()->Get(wheelSep);

  if (_sdf->HasElement("wheelDiameter"))
    _sdf->GetElement("wheelDiameter")->GetValue()->Get(wheelDiam);

  if (_sdf->HasElement("torque"))
    _sdf->GetElement("torque")->GetValue()->Get(torque);

  // Make sure the ROS node for Gazebo has already been initialized
  if (!ros::isInitialized())
  {
    ROS_FATAL_STREAM("A ROS node for Gazebo has not been initialized, unable to load plugin. "
      << "Load the Gazebo system plugin 'libgazebo_ros_api_plugin.so' in the gazebo_ros package)");
    return;
  }

  rosnode_ = new ros::NodeHandle(namespace_);

  tf_prefix_ = tf::getPrefixParam(*rosnode_);
  transform_broadcaster_ = new tf::TransformBroadcaster();

  // ROS: Subscribe to the velocity command topic (usually "cmd_vel")
  ros::SubscribeOptions so =
      ros::SubscribeOptions::create<geometry_msgs::Twist>(topic_, 1,
                                                          boost::bind(&DiffDrivePlugin6W::cmdVelCallback, this, _1),
                                                          ros::VoidPtr(), &queue_);
  sub_ = rosnode_->subscribe(so);
  pub_ = rosnode_->advertise<nav_msgs::Odometry>("odom", 1);

  callback_queue_thread_ = boost::thread(boost::bind(&DiffDrivePlugin6W::QueueThread, this));

  Reset();

  // New Mechanism for Updating every World Cycle
  // Listen to the update event. This event is broadcast every
  // simulation iteration.
  updateConnection = event::Events::ConnectWorldUpdateBegin(
      boost::bind(&DiffDrivePlugin6W::Update, this));
}

// Initialize the controller
void DiffDrivePlugin6W::Reset()
{
  enableMotors = true;

  for (size_t i = 0; i < 2; ++i){
    wheelSpeed[i] = 0;
  }

#if (GAZEBO_MAJOR_VERSION >= 8)
  prevUpdateTime = world->SimTime();
#else
  prevUpdateTime = world->GetSimTime();
#endif

  x_ = 0;
  rot_ = 0;
  alive_ = true;

  // Reset odometric pose
  odomPose[0] = 0.0;
  odomPose[1] = 0.0;
  odomPose[2] = 0.0;

  odomVel[0] = 0.0;
  odomVel[1] = 0.0;
  odomVel[2] = 0.0;
}

// Update the controller
void DiffDrivePlugin6W::Update()
{
  // TODO: Step should be in a parameter of this function
  double d1, d2;
  double dr, da;
  common::Time stepTime;

  GetPositionCmd();

  //stepTime = World::Instance()->GetPhysicsEngine()->GetStepTime();
#if (GAZEBO_MAJOR_VERSION >= 8)
  stepTime = world->SimTime() - prevUpdateTime;
  prevUpdateTime = world->SimTime();
#else
  stepTime = world->GetSimTime() - prevUpdateTime;
  prevUpdateTime = world->GetSimTime();
#endif

  // Distance travelled by front wheels
  d1 = stepTime.Double() * wheelDiam / 2 * joints[MID_LEFT]->GetVelocity(0);
  d2 = stepTime.Double() * wheelDiam / 2 * joints[MID_RIGHT]->GetVelocity(0);

  dr = (d1 + d2) / 2;
  da = (d1 - d2) / wheelSep;

  // Compute odometric pose
  odomPose[0] += dr * cos(odomPose[2]);
  odomPose[1] += dr * sin(odomPose[2]);
  odomPose[2] += da;

  // Compute odometric instantaneous velocity
  odomVel[0] = dr / stepTime.Double();
  odomVel[1] = 0.0;
  odomVel[2] = da / stepTime.Double();

  if (enableMotors)
  {
    joints[FRONT_LEFT]->SetVelocity(0, wheelSpeed[0] / (wheelDiam / 2.0));
    joints[MID_LEFT]->SetVelocity(0, wheelSpeed[0] / (wheelDiam / 2.0));
    joints[REAR_LEFT]->SetVelocity(0, wheelSpeed[0] / (wheelDiam / 2.0));

    joints[FRONT_RIGHT]->SetVelocity(0, wheelSpeed[1] / (wheelDiam / 2.0));
    joints[MID_RIGHT]->SetVelocity(0, wheelSpeed[1] / (wheelDiam / 2.0));
    joints[REAR_RIGHT]->SetVelocity(0, wheelSpeed[1] / (wheelDiam / 2.0));

#if (GAZEBO_MAJOR_VERSION > 4)
    joints[FRONT_LEFT]->SetEffortLimit(0, torque);
    joints[MID_LEFT]->SetEffortLimit(0, torque);
    joints[REAR_LEFT]->SetEffortLimit(0, torque);

    joints[FRONT_RIGHT]->SetEffortLimit(0, torque);
    joints[MID_RIGHT]->SetEffortLimit(0, torque);
    joints[REAR_RIGHT]->SetEffortLimit(0, torque);
#else
    joints[FRONT_LEFT]->SetMaxForce(0, torque);
    joints[MID_LEFT]->SetMaxForce(0, torque);
    joints[REAR_LEFT]->SetMaxForce(0, torque);

    joints[FRONT_RIGHT]->SetMaxForce(0, torque);
    joints[MID_RIGHT]->SetMaxForce(0, torque);
    joints[REAR_RIGHT]->SetMaxForce(0, torque);
#endif
  }

  //publish_odometry();
}

// NEW: Now uses the target velocities from the ROS message, not the Iface 
void DiffDrivePlugin6W::GetPositionCmd()
{
  lock.lock();

  double vr, va;

  vr = x_; //myIface->data->cmdVelocity.pos.x;
  va = -rot_; //myIface->data->cmdVelocity.yaw;

  //std::cout << "X: [" << x_ << "] ROT: [" << rot_ << "]" << std::endl;

  // Changed motors to be always on, which is probably what we want anyway
  enableMotors = true; //myIface->data->cmdEnableMotors > 0;

  //std::cout << enableMotors << std::endl;

  wheelSpeed[0] = vr + va * wheelSep / 2;
  wheelSpeed[1] = vr - va * wheelSep / 2;

  lock.unlock();
}

// NEW: Store the velocities from the ROS message
void DiffDrivePlugin6W::cmdVelCallback(const geometry_msgs::Twist::ConstPtr& cmd_msg)
{
  //std::cout << "BEGIN CALLBACK\n";

  lock.lock();

  x_ = cmd_msg->linear.x;
  rot_ = cmd_msg->angular.z;

  lock.unlock();

  //std::cout << "END CALLBACK\n";
}

// NEW: custom callback queue thread
void DiffDrivePlugin6W::QueueThread()
{
  static const double timeout = 0.01;

  while (alive_ && rosnode_->ok())
  {
    //    std::cout << "CALLING STUFF\n";
    queue_.callAvailable(ros::WallDuration(timeout));
  }
}

// NEW: Update this to publish odometry topic
void DiffDrivePlugin6W::publish_odometry()
{
  // get current time
#if (GAZEBO_MAJOR_VERSION >= 8)
  ros::Time current_time_((world->SimTime()).sec, (world->SimTime()).nsec); 
#else
  ros::Time current_time_((world->GetSimTime()).sec, (world->GetSimTime()).nsec); 
#endif

  // getting data for base_footprint to odom transform
#if (GAZEBO_MAJOR_VERSION >= 8)
  ignition::math::Pose3d pose = link->WorldPose();
  ignition::math::Vector3d velocity = link->WorldLinearVel();
  ignition::math::Vector3d angular_velocity = link->WorldAngularVel();

  tf::Quaternion qt(pose.Rot().X(), pose.Rot().Y(), pose.Rot().Z(), pose.Rot().W());
  tf::Vector3 vt(pose.Pos().X(), pose.Pos().Y(), pose.Pos().Z());
#else
  math::Pose pose = link->GetWorldPose();
  math::Vector3 velocity = link->GetWorldLinearVel();
  math::Vector3 angular_velocity = link->GetWorldAngularVel();

  tf::Quaternion qt(pose.rot.x, pose.rot.y, pose.rot.z, pose.rot.w);
  tf::Vector3 vt(pose.pos.x, pose.pos.y, pose.pos.z);
#endif
  tf::Transform base_footprint_to_odom(qt, vt);

  transform_broadcaster_->sendTransform(tf::StampedTransform(base_footprint_to_odom,
                                                            current_time_,
                                                            "odom",
                                                            "base_footprint"));

  // publish odom topic
#if (GAZEBO_MAJOR_VERSION >= 8)
  odom_.pose.pose.position.x = pose.Pos().X();
  odom_.pose.pose.position.y = pose.Pos().Y();

  odom_.pose.pose.orientation.x = pose.Rot().X();
  odom_.pose.pose.orientation.y = pose.Rot().Y();
  odom_.pose.pose.orientation.z = pose.Rot().Z();
  odom_.pose.pose.orientation.w = pose.Rot().W();

  odom_.twist.twist.linear.x = velocity.X();
  odom_.twist.twist.linear.y = velocity.Y();
  odom_.twist.twist.angular.z = angular_velocity.Z();
#else
  odom_.pose.pose.position.x = pose.pos.x;
  odom_.pose.pose.position.y = pose.pos.y;

  odom_.pose.pose.orientation.x = pose.rot.x;
  odom_.pose.pose.orientation.y = pose.rot.y;
  odom_.pose.pose.orientation.z = pose.rot.z;
  odom_.pose.pose.orientation.w = pose.rot.w;

  odom_.twist.twist.linear.x = velocity.x;
  odom_.twist.twist.linear.y = velocity.y;
  odom_.twist.twist.angular.z = angular_velocity.z;
#endif

  odom_.header.frame_id = tf::resolve(tf_prefix_, "odom");
  odom_.child_frame_id = "base_footprint";
  odom_.header.stamp = current_time_;

  pub_.publish(odom_);
}

GZ_REGISTER_MODEL_PLUGIN(DiffDrivePlugin6W)

} // namespace gazebo
