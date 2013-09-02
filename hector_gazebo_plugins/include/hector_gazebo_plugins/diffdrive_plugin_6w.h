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
/*
 * Desc: ROS interface to a Position2d controller for a Differential drive.
 * Author: Daniel Hewlett (adapted from Nathan Koenig)
 */
#ifndef DIFFDRIVE_PLUGIN_HH
#define DIFFDRIVE_PLUGIN_HH

#include <map>

#include <gazebo/common/Plugin.hh>
#include <gazebo/common/Time.hh>

// ROS 
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>

// Custom Callback Queue
#include <ros/callback_queue.h>
#include <ros/advertise_options.h>

// Boost
#include <boost/thread.hpp>
#include <boost/bind.hpp>

namespace gazebo
{

class DiffDrivePlugin6W : public ModelPlugin
{

public:
  DiffDrivePlugin6W();
  virtual ~DiffDrivePlugin6W();

protected:
  virtual void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf);
  virtual void Reset();
  virtual void Update();

private:
  void publish_odometry();
  void GetPositionCmd();

  physics::LinkPtr link;
  physics::WorldPtr world;
  physics::JointPtr joints[6];

  float wheelSep;
  float wheelDiam;
  float torque;
  float wheelSpeed[2];

  // Simulation time of the last update
  common::Time prevUpdateTime;

  bool enableMotors;
  float odomPose[3];
  float odomVel[3];

  // ROS STUFF
  ros::NodeHandle* rosnode_;
  ros::Publisher pub_;
  ros::Subscriber sub_;
  tf::TransformBroadcaster *transform_broadcaster_;
  nav_msgs::Odometry odom_;
  std::string tf_prefix_;

  boost::mutex lock;

  std::string namespace_;
  std::string topic_;
  std::string link_name_;

  // Custom Callback Queue
  ros::CallbackQueue queue_;
  boost::thread callback_queue_thread_;
  void QueueThread();

  // DiffDrive stuff
  void cmdVelCallback(const geometry_msgs::Twist::ConstPtr& cmd_msg);

  float x_;
  float rot_;
  bool alive_;

  // Pointer to the update event connection
  event::ConnectionPtr updateConnection;
};

}

#endif

