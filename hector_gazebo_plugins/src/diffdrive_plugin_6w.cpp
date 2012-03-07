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

#include <gazebo/gazebo.h>
#include <gazebo/GazeboError.hh>
#include <gazebo/Quatern.hh>
#include <gazebo/Controller.hh>
#include <gazebo/Body.hh>
#include <gazebo/World.hh>
#include <gazebo/Simulator.hh>
#include <gazebo/Global.hh>
#include <gazebo/XMLConfig.hh>
#include <gazebo/ControllerFactory.hh>
#include <gazebo/PhysicsEngine.hh>

#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <boost/bind.hpp>

using namespace gazebo;

GZ_REGISTER_DYNAMIC_CONTROLLER("diffdrive_plugin_6w", DiffDrivePlugin6W);

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
DiffDrivePlugin6W::DiffDrivePlugin6W(Entity *parent) :
  Controller(parent)
{
  parent_ = dynamic_cast<Model*> (parent);

  if (!parent_)
    gzthrow("Differential_Position2d controller requires a Model as its parent");

  enableMotors = true;

  for (size_t i = 0; i < 2; ++i){
    wheelSpeed[i] = 0;
  }

  prevUpdateTime = Simulator::Instance()->GetSimTime();

  Param::Begin(&parameters);
  frontLeftJointNameP = new ParamT<std::string> ("frontLeftJoint", "", 1);
  frontRightJointNameP = new ParamT<std::string> ("frontRightJoint", "", 1);
  midLeftJointNameP = new ParamT<std::string> ("midLeftJoint", "", 1);
  midRightJointNameP = new ParamT<std::string> ("midRightJoint", "", 1);
  rearLeftJointNameP = new ParamT<std::string> ("rearLeftJoint", "", 1);
  rearRightJointNameP = new ParamT<std::string> ("rearRightJoint", "", 1);

  wheelSepP = new ParamT<float> ("wheelSeparation", 0.34, 1);
  wheelDiamP = new ParamT<float> ("wheelDiameter", 0.15, 1);
  torqueP = new ParamT<float> ("torque", 10.0, 1);
  robotNamespaceP = new ParamT<std::string> ("robotNamespace", "", 0);
  topicNameP = new ParamT<std::string> ("topicName", "", 1);
  Param::End();

  x_ = 0;
  rot_ = 0;
  alive_ = true;
}

// Destructor
DiffDrivePlugin6W::~DiffDrivePlugin6W()
{

  delete frontLeftJointNameP;
  delete frontRightJointNameP;
  delete midLeftJointNameP;
  delete midRightJointNameP;
  delete rearLeftJointNameP;
  delete rearRightJointNameP;

  delete wheelSepP;
  delete wheelDiamP;
  delete torqueP;
  delete robotNamespaceP;
  delete topicNameP;
  delete callback_queue_thread_;
  delete rosnode_;
  delete transform_broadcaster_;
}

// Load the controller
void DiffDrivePlugin6W::LoadChild(XMLConfigNode *node)
{
  pos_iface_ = dynamic_cast<libgazebo::PositionIface*> (GetIface("position"));

  // the defaults are from pioneer2dx
  wheelSepP->Load(node);
  wheelDiamP->Load(node);
  torqueP->Load(node);

  //leftJointNameP->Load(node);
  //rightJointNameP->Load(node);

  frontLeftJointNameP->Load(node);
  frontRightJointNameP->Load(node);
  midLeftJointNameP->Load(node);
  midRightJointNameP->Load(node);
  rearLeftJointNameP->Load(node);
  rearRightJointNameP->Load(node);

  joints[FRONT_LEFT] = parent_->GetJoint(**frontLeftJointNameP);
  joints[FRONT_RIGHT] = parent_->GetJoint(**frontRightJointNameP);
  joints[MID_LEFT] = parent_->GetJoint(**midLeftJointNameP);
  joints[MID_RIGHT] = parent_->GetJoint(**midRightJointNameP);
  joints[REAR_LEFT] = parent_->GetJoint(**rearLeftJointNameP);
  joints[REAR_RIGHT] = parent_->GetJoint(**rearRightJointNameP);

  //if (!joints[LEFT])
  //  gzthrow("The controller couldn't get left hinge joint");

  //if (!joints[RIGHT])
  //  gzthrow("The controller couldn't get right hinge joint");

  // Initialize the ROS node and subscribe to cmd_vel

  robotNamespaceP->Load(node);
  robotNamespace = robotNamespaceP->GetValue();

  int argc = 0;
  char** argv = NULL;
  ros::init(argc, argv, "diff_drive_plugin", ros::init_options::NoSigintHandler | ros::init_options::AnonymousName);
  rosnode_ = new ros::NodeHandle(robotNamespace);

  tf_prefix_ = tf::getPrefixParam(*rosnode_);
  transform_broadcaster_ = new tf::TransformBroadcaster();

  topicNameP->Load(node);
  topicName = topicNameP->GetValue();

  // ROS: Subscribe to the velocity command topic (usually "cmd_vel")
  ros::SubscribeOptions so =
      ros::SubscribeOptions::create<geometry_msgs::Twist>(topicName, 1,
                                                          boost::bind(&DiffDrivePlugin6W::cmdVelCallback, this, _1),
                                                          ros::VoidPtr(), &queue_);
  sub_ = rosnode_->subscribe(so);
  pub_ = rosnode_->advertise<nav_msgs::Odometry>("odom", 1);
}

// Initialize the controller
void DiffDrivePlugin6W::InitChild()
{
  // Reset odometric pose
  odomPose[0] = 0.0;
  odomPose[1] = 0.0;
  odomPose[2] = 0.0;

  odomVel[0] = 0.0;
  odomVel[1] = 0.0;
  odomVel[2] = 0.0;

  callback_queue_thread_ = new boost::thread(boost::bind(&DiffDrivePlugin6W::QueueThread, this));
}

// Load the controller
void DiffDrivePlugin6W::SaveChild(std::string &prefix, std::ostream &stream)
{
  stream << prefix << *(frontLeftJointNameP) << "\n";
  stream << prefix << *(frontRightJointNameP) << "\n";
  stream << prefix << *(midLeftJointNameP) << "\n";
  stream << prefix << *(midRightJointNameP) << "\n";
  stream << prefix << *(rearLeftJointNameP) << "\n";
  stream << prefix << *(rearRightJointNameP) << "\n";

  stream << prefix << *(torqueP) << "\n";
  stream << prefix << *(wheelDiamP) << "\n";
  stream << prefix << *(wheelSepP) << "\n";
}

// Reset
void DiffDrivePlugin6W::ResetChild()
{
  // Reset odometric pose
  odomPose[0] = 0.0;
  odomPose[1] = 0.0;
  odomPose[2] = 0.0;

  odomVel[0] = 0.0;
  odomVel[1] = 0.0;
  odomVel[2] = 0.0;
}

// Update the controller
void DiffDrivePlugin6W::UpdateChild()
{
  // TODO: Step should be in a parameter of this function
  double wd, ws;
  double d1, d2;
  double dr, da;
  Time stepTime;

  //myIface->Lock(1);

  GetPositionCmd();

  wd = **(wheelDiamP);
  ws = **(wheelSepP);

  //stepTime = World::Instance()->GetPhysicsEngine()->GetStepTime();
  stepTime = Simulator::Instance()->GetSimTime() - prevUpdateTime;
  prevUpdateTime = Simulator::Instance()->GetSimTime();

  // Distance travelled by front wheels
  d1 = stepTime.Double() * wd / 2 * joints[MID_LEFT]->GetVelocity(0);
  d2 = stepTime.Double() * wd / 2 * joints[MID_RIGHT]->GetVelocity(0);

  dr = (d1 + d2) / 2;
  da = (d1 - d2) / ws;

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
    joints[FRONT_LEFT]->SetVelocity(0, wheelSpeed[0] / (**(wheelDiamP) / 2.0));
    joints[MID_LEFT]->SetVelocity(0, wheelSpeed[0] / (**(wheelDiamP) / 2.0));
    joints[REAR_LEFT]->SetVelocity(0, wheelSpeed[0] / (**(wheelDiamP) / 2.0));

    joints[FRONT_RIGHT]->SetVelocity(0, wheelSpeed[1] / (**(wheelDiamP) / 2.0));
    joints[MID_RIGHT]->SetVelocity(0, wheelSpeed[1] / (**(wheelDiamP) / 2.0));
    joints[REAR_RIGHT]->SetVelocity(0, wheelSpeed[1] / (**(wheelDiamP) / 2.0));

    joints[FRONT_LEFT]->SetMaxForce(0, **(torqueP));
    joints[MID_LEFT]->SetMaxForce(0, **(torqueP));
    joints[REAR_LEFT]->SetMaxForce(0, **(torqueP));

    joints[FRONT_RIGHT]->SetMaxForce(0, **(torqueP));
    joints[MID_RIGHT]->SetMaxForce(0, **(torqueP));
    joints[REAR_RIGHT]->SetMaxForce(0, **(torqueP));
  }

  write_position_data();
  //publish_odometry();

  //myIface->Unlock();
}

// Finalize the controller
void DiffDrivePlugin6W::FiniChild()
{
  //std::cout << "ENTERING FINALIZE\n";
  alive_ = false;
  // Custom Callback Queue
  queue_.clear();
  queue_.disable();
  rosnode_->shutdown();
  callback_queue_thread_->join();
  //std::cout << "EXITING FINALIZE\n";
}

// NEW: Now uses the target velocities from the ROS message, not the Iface 
void DiffDrivePlugin6W::GetPositionCmd()
{
  lock.lock();

  double vr, va;

  vr = x_; //myIface->data->cmdVelocity.pos.x;
  va = rot_; //myIface->data->cmdVelocity.yaw;

  //std::cout << "X: [" << x_ << "] ROT: [" << rot_ << "]" << std::endl;

  // Changed motors to be always on, which is probably what we want anyway
  enableMotors = true; //myIface->data->cmdEnableMotors > 0;

  //std::cout << enableMotors << std::endl;

  wheelSpeed[0] = vr + va * **(wheelSepP) / 2;
  wheelSpeed[1] = vr - va * **(wheelSepP) / 2;

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
  ros::Time current_time_((Simulator::Instance()->GetSimTime()).sec, (Simulator::Instance()->GetSimTime()).nsec); 

  // getting data for base_footprint to odom transform
  btQuaternion qt;
  // TODO: Is there something wrong here? RVIZ has a problem?
  qt.setEulerZYX(pos_iface_->data->pose.yaw, pos_iface_->data->pose.pitch, pos_iface_->data->pose.roll);
  btVector3 vt(pos_iface_->data->pose.pos.x, pos_iface_->data->pose.pos.y, pos_iface_->data->pose.pos.z);
  tf::Transform base_footprint_to_odom(qt, vt);

  transform_broadcaster_->sendTransform(tf::StampedTransform(base_footprint_to_odom,
                                                            current_time_,
                                                            "odom",
                                                            "base_footprint"));

  // publish odom topic
  odom_.pose.pose.position.x = pos_iface_->data->pose.pos.x;
  odom_.pose.pose.position.y = pos_iface_->data->pose.pos.y;

  gazebo::Quatern rot;
  rot.SetFromEuler(gazebo::Vector3(pos_iface_->data->pose.roll, pos_iface_->data->pose.pitch, pos_iface_->data->pose.yaw));

  odom_.pose.pose.orientation.x = rot.x;
  odom_.pose.pose.orientation.y = rot.y;
  odom_.pose.pose.orientation.z = rot.z;
  odom_.pose.pose.orientation.w = rot.u;

  odom_.twist.twist.linear.x = pos_iface_->data->velocity.pos.x;
  odom_.twist.twist.linear.y = pos_iface_->data->velocity.pos.y;
  odom_.twist.twist.angular.z = pos_iface_->data->velocity.yaw;

  odom_.header.frame_id = tf::resolve(tf_prefix_, "odom");
  odom_.child_frame_id = "base_footprint";

  //odom_.header.stamp = current_time_;
  odom_.header.stamp.sec = (Simulator::Instance()->GetSimTime()).sec;
  odom_.header.stamp.nsec = (Simulator::Instance()->GetSimTime()).nsec;

  pub_.publish(odom_);
}

// Update the data in the interface
void DiffDrivePlugin6W::write_position_data()
{
  // TODO: Data timestamp
  pos_iface_->data->head.time = Simulator::Instance()->GetSimTime().Double();

  pos_iface_->data->pose.pos.x = odomPose[0];
  pos_iface_->data->pose.pos.y = odomPose[1];
  pos_iface_->data->pose.yaw = NORMALIZE(odomPose[2]);

  pos_iface_->data->velocity.pos.x = odomVel[0];
  pos_iface_->data->velocity.yaw = odomVel[2];

  // TODO
  pos_iface_->data->stall = 0;
}

