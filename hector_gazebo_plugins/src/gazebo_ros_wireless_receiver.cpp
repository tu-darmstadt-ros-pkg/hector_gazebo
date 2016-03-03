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

#include "gazebo/math/Rand.hh"
#include "gazebo/msgs/msgs.hh"
#include "gazebo/sensors/SensorFactory.hh"
#include "gazebo/sensors/SensorManager.hh"
#include "gazebo/sensors/WirelessReceiver.hh"
#include "gazebo/sensors/WirelessTransmitter.hh"
#include "gazebo/transport/Node.hh"
#include "gazebo/transport/Publisher.hh"

#include <sensor_msgs/Range.h>

#include <limits>

using namespace gazebo;
using namespace sensors;


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
  sensor_ = boost::dynamic_pointer_cast<sensors::WirelessReceiver>(_sensor);
  if (!sensor_)
  {
    gzthrow("GazeboRosWirelessReceiver requires a WirelessReceiver Sensor as its parent");
    return;
  }

  // Get the world name.
  // std::string worldName = sensor_->GetWorldName();
  // world = physics::get_world(worldName);
  

  // default parameters
  namespace_.clear();
  topic_ = "WirelessReceiver";
  frame_id_ = "/WirelessReceiver_link";

  // load parameters
  if (_sdf->HasElement("robotNamespace"))
    namespace_ = _sdf->GetElement("robotNamespace")->GetValue()->GetAsString();

  if (_sdf->HasElement("frameId"))
    frame_id_ = _sdf->GetElement("frameId")->GetValue()->GetAsString();

  if (_sdf->HasElement("topicName"))
    topic_ = _sdf->GetElement("topicName")->GetValue()->GetAsString();

  sensor_model_.Load(_sdf);



  // Make sure the ROS node for Gazebo has already been initialized
  if (!ros::isInitialized())
  {
    ROS_FATAL_STREAM("A ROS node for Gazebo has not been initialized, unable to load plugin. "
      << "Load the Gazebo system plugin 'libgazebo_ros_api_plugin.so' in the gazebo_ros package)");
    return;
  }

  node_handle_ = new ros::NodeHandle(namespace_);
  //publisher_ = node_handle_->advertise<msgs::WirelessNodes>(topic_, 1);
  publisher_ = node_handle_->advertise<sensor_msgs::Range>(topic_, 1);

  // setup dynamic_reconfigure server
  dynamic_reconfigure_server_.reset(new dynamic_reconfigure::Server<SensorModelConfig>(ros::NodeHandle(*node_handle_, topic_)));
  dynamic_reconfigure_server_->setCallback(boost::bind(&SensorModel::dynamicReconfigureCallback, &sensor_model_, _1, _2));

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
  sensor_model_.reset();
}

////////////////////////////////////////////////////////////////////////////////
// Update the controller
void GazeboRosWirelessReceiver::Update()
{
  
  // activate RaySensor if it is not yet active
  if (!sensor_->IsActive()) sensor_->SetActive(true);

  std::string txEssid;
  //msgs::WirelessNodes msg;

  double rxPower;
  double txFreq;

  this->referencePose =
      this->pose + this->parentEntity.lock()->GetWorldPose();

  math::Pose myPos = this->referencePose;
  Sensor_V sensors = SensorManager::Instance()->GetSensors();
  for (Sensor_V::iterator it = sensors.begin(); it != sensors.end(); ++it)
  {
    if ((*it)->GetType() == "wireless_transmitter")
    {
      boost::shared_ptr<gazebo::sensors::WirelessTransmitter> transmitter =
          boost::static_pointer_cast<WirelessTransmitter>(*it);

      txFreq = transmitter->GetFreq();
      rxPower = transmitter->GetSignalStrength(myPos, this->GetGain());

      // Discard if the frequency received is out of our frequency range,
      // or if the received signal strengh is lower than the sensivity
      if ((txFreq < this->GetMinFreqFiltered()) ||
          (txFreq > this->GetMaxFreqFiltered()) ||
          (rxPower < this->GetSensitivity()))
      {
        continue;
      }

      txEssid = transmitter->GetESSID();

      // msgs::WirelessNode *wirelessNode = msg.add_node();
      // wirelessNode->set_essid(txEssid);
      // wirelessNode->set_frequency(txFreq);
      // wirelessNode->set_signal_level(rxPower);
    }
  }
  // if (msg.node_size() > 0)
  // {
  //   this->pub->Publish(msg);
  // }

  sensor_msgs::Range range_msg;
  range_msg.header.frame_id = "/base_link";
  range_msg.header.stamp = ros::Time::now();
  double dist = 3.0;
  range_msg.radiation_type = 0;
  range_msg.field_of_view = 2.0/dist; 
  range_msg.min_range = 0.4;
  range_msg.max_range = 10;
  range_msg.range = dist;
  publisher_.publish(range_msg);

}

// Register this plugin with the simulator
GZ_REGISTER_SENSOR_PLUGIN(GazeboRosWirelessReceiver)

} // namespace gazebo
