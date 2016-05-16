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

#include <hector_gazebo_plugins/gazebo_ros_wireless_receiver.h>
#include <gazebo/common/Events.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/sensors/Sensor.hh>

#include <limits>

#include "gazebo/physics/Base.hh"
#include "gazebo/physics/PhysicsTypes.hh"
#include "gazebo/physics/WorldState.hh"
#include "gazebo/gazebo.hh"


using namespace std;

namespace gazebo {

GazeboRosWirelessReceiver::GazeboRosWirelessReceiver()
{
}

////////////////////////////////////////////////////////////////////////////////
// Destructor
GazeboRosWirelessReceiver::~GazeboRosWirelessReceiver()
{
  updateTimer.Disconnect(updateConnection);
  sensor_->SetActive(false);

  dynamic_reconfigure_server_.reset();

  node_handle_->shutdown();
  delete node_handle_;
}

////////////////////////////////////////////////////////////////////////////////
// Load the controller
void GazeboRosWirelessReceiver::Load(sensors::SensorPtr _sensor, sdf::ElementPtr _sdf)
{
  // Get then name of the parent sensor
  sensor_ = boost::dynamic_pointer_cast<sensors::Sensor>(_sensor);
  if (!sensor_)
  {
    gzthrow("GazeboRosWirelessReceiver requires a Sensor as its parent");
    return;
  }

  // Get the world name.
  std::string worldName = sensor_->GetWorldName();
  world = physics::get_world(worldName);
  std::string parentName = sensor_->GetParentName();
  parentEntity = world->GetEntity(parentName);

  // default parameters
  namespace_.clear();
  topic_ = "wireless_receiver";
  frame_id_ = "/wireless_receiver";

  // load parameters
  if (_sdf->HasElement("robotNamespace"))
    namespace_ = _sdf->GetElement("robotNamespace")->GetValue()->GetAsString();

  if (_sdf->HasElement("frameId"))
    frame_id_ = _sdf->GetElement("frameId")->GetValue()->GetAsString();

  if (_sdf->HasElement("topicName"))
    topic_ = _sdf->GetElement("topicName")->GetValue()->GetAsString();



  rss_sensor_model_.Load(_sdf, "rss");
  AoA_sensor_model_.Load(_sdf, "AoA");


  // Make sure the ROS node for Gazebo has already been initialized
  if (!ros::isInitialized())
  {
    ROS_FATAL_STREAM("A ROS node for Gazebo has not been initialized, unable to load plugin. "
      << "Load the Gazebo system plugin 'libgazebo_ros_api_plugin.so' in the gazebo_ros package)");
    return;
  }

  node_handle_ = new ros::NodeHandle(namespace_);

  physics::Model_V models = this->world->GetModels();
  transmitter_count_ = 0;
  for(physics::Model_V::iterator it = models.begin(); it != models.end(); it++)
  {
    std::string name = (*it)->GetName();
    
    if( name.compare(0, 15, "wireless_router") == 0 )
    {
      transmitter_count_++;
    }
  }
  if(transmitter_count_ > 0)
  {
    transmitter_pub_ = node_handle_->advertise<sensor_msgs::PointCloud>(topic_+"/transmitter", 1);
    rss_pub_ = node_handle_->advertise<sensor_msgs::PointCloud>(topic_+"/rss", 1);
    AoA_pub_ = node_handle_->advertise<sensor_msgs::PointCloud>(topic_+"/AoA", 1);
    receiver_pub_ = node_handle_->advertise<geometry_msgs::PoseStamped>(topic_ + "/receiver", 1);
  }


  // setup dynamic_reconfigure server
  dynamic_reconfigure_server_.reset(new dynamic_reconfigure::Server<SensorModelConfig>(ros::NodeHandle(*node_handle_, topic_)));
  dynamic_reconfigure_server_->setCallback(boost::bind(&SensorModel::dynamicReconfigureCallback, &rss_sensor_model_, _1, _2));
  dynamic_reconfigure_server_->setCallback(boost::bind(&SensorModel3::dynamicReconfigureCallback, &AoA_sensor_model_, _1, _2));

  Reset();

  // connect Update function
  updateTimer.setUpdateRate(10.0);
  updateTimer.Load(world, _sdf);
  updateConnection = updateTimer.Connect(boost::bind(&GazeboRosWirelessReceiver::Update, this));

  // activate RaySensor
  sensor_->SetActive(true);
}

void GazeboRosWirelessReceiver::Reset()
{
  updateTimer.Reset();
  rss_sensor_model_.reset();
  AoA_sensor_model_.reset();
}

////////////////////////////////////////////////////////////////////////////////
// Update the controller
void GazeboRosWirelessReceiver::Update()
{
  common::Time sim_time = world->GetSimTime();
  double dt = updateTimer.getTimeSinceLastUpdate().Double();

  // activate RaySensor if it is not yet active
  if (!sensor_->IsActive()) sensor_->SetActive(true);

  if(transmitter_count_  == 0)
  {
    return;
  }
  
  sensor_msgs::PointCloud transmitters_;//positon of transmitter
  sensor_msgs::PointCloud rss_; //rss value of transmitter, here is the distance 
  sensor_msgs::PointCloud AoA_;//angle of arrival 
  geometry_msgs::PoseStamped receiver_pose_;

  //receiver's pose in the world
  math::Pose referencePose = 
      sensor_->GetPose() + this->parentEntity->GetWorldPose();

  receiver_pose_.header.frame_id = "/world";
  receiver_pose_.header.stamp.sec =  (world->GetSimTime()).sec;
  receiver_pose_.header.stamp.nsec =  (world->GetSimTime()).nsec;
  receiver_pose_.pose.position.x = referencePose.pos.x;
  receiver_pose_.pose.position.y = referencePose.pos.y;
  receiver_pose_.pose.position.z = referencePose.pos.z;
  receiver_pose_.pose.orientation.w = referencePose.rot.w;
  receiver_pose_.pose.orientation.x = referencePose.rot.x;
  receiver_pose_.pose.orientation.y = referencePose.rot.y;
  receiver_pose_.pose.orientation.z = referencePose.rot.z;
  


  transmitters_.header.frame_id = "/world";
  transmitters_.header.stamp.sec =  (world->GetSimTime()).sec;
  transmitters_.header.stamp.nsec =  (world->GetSimTime()).nsec;

  AoA_.header.frame_id = "/wireless_receiver"; //todo
  AoA_.header.stamp.sec =  (world->GetSimTime()).sec;
  AoA_.header.stamp.nsec =  (world->GetSimTime()).nsec;

  rss_.header.frame_id = "/wireless_receiver";
  rss_.header.stamp.sec =  (world->GetSimTime()).sec;
  rss_.header.stamp.nsec =  (world->GetSimTime()).nsec;

 
  transmitters_.channels.resize(1);
  rss_.channels.resize(1);
  AoA_.channels.resize(1);


  int count = 0;//id of the transmitter
  physics::Model_V models = this->world->GetModels();
  for(physics::Model_V::iterator it = models.begin(); it != models.end(); it++)
  {
    std::string name = (*it)->GetName();
    
    if( name.compare(0, 15, "wireless_router") == 0 )
    {                                                                              
      //std::cout << "name: " << name << std::endl;
      math::Pose model_pose = (*it)->GetWorldPose();
      math::Pose relative_pose = -(referencePose - model_pose);//router coordinator to receiver coordinator

      geometry_msgs::Point32 p;
      p.x = model_pose.pos.x;
      p.y = model_pose.pos.y;
      p.z = model_pose.pos.z;
      transmitters_.points.push_back(p);
      transmitters_.channels[0].values.push_back(count);

      double dist = relative_pose.pos.GetLength();
      dist = rss_sensor_model_(dist, dt);
      geometry_msgs::Point32 dist_p;
      dist_p.x = dist;//dist; //todo
      rss_.points.push_back(dist_p);
      rss_.channels[0].values.push_back(count);

      relative_pose.pos.Normalize();//only direction
      math::Vector3 dir  = AoA_sensor_model_(relative_pose.pos, dt);
      geometry_msgs::Point32 relative_p;
      relative_p.x = dir.x;
      relative_p.y = dir.y;
      relative_p.z = dir.z;

      AoA_.points.push_back(relative_p);
      AoA_.channels[0].values.push_back(count);
      count++;
    }
    
  }   
  if(count > 0)
  {
    AoA_pub_.publish(AoA_);
    transmitter_pub_.publish(transmitters_);
    rss_pub_.publish(rss_);
    receiver_pub_.publish(receiver_pose_);
  }

  
}

// Register this plugin with the simulator
GZ_REGISTER_SENSOR_PLUGIN(GazeboRosWirelessReceiver)

} // namespace gazebo
