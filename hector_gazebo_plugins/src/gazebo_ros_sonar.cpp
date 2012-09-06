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

#include <hector_gazebo_plugins/gazebo_ros_sonar.h>
#include "common/Events.hh"
#include "physics/physics.h"
#include "sensors/RaySensor.hh"

#include <limits>

namespace gazebo {

GazeboRosSonar::GazeboRosSonar()
{
}

////////////////////////////////////////////////////////////////////////////////
// Destructor
GazeboRosSonar::~GazeboRosSonar()
{
  sensor_->SetActive(false);
  event::Events::DisconnectWorldUpdateStart(updateConnection);
  node_handle_->shutdown();
  delete node_handle_;
}

////////////////////////////////////////////////////////////////////////////////
// Load the controller
void GazeboRosSonar::Load(sensors::SensorPtr _sensor, sdf::ElementPtr _sdf)
{
  // Get then name of the parent sensor
  sensor_ = boost::shared_dynamic_cast<sensors::RaySensor>(_sensor);
  if (!sensor_)
  {
    gzthrow("GazeboRosSonar requires a Ray Sensor as its parent");
    return;
  }

  // Get the world name.
  std::string worldName = sensor_->GetWorldName();
  world = physics::get_world(worldName);

  // load parameters
  if (!_sdf->HasElement("robotNamespace"))
    namespace_.clear();
  else
    namespace_ = _sdf->GetElement("robotNamespace")->GetValueString() + "/";

  if (!_sdf->HasElement("frameId"))
    frame_id_ = "";
  else
    frame_id_ = _sdf->GetElement("frameId")->GetValueString();

  if (!_sdf->HasElement("topicName"))
    topic_ = "sonar";
  else
    topic_ = _sdf->GetElement("topicName")->GetValueString();

  sensor_model_.Load(_sdf);

  range_.header.frame_id = frame_id_;
  range_.radiation_type = sensor_msgs::Range::ULTRASOUND;
  range_.field_of_view = std::min(fabs((sensor_->GetAngleMax() - sensor_->GetAngleMin()).GetAsRadian()), fabs((sensor_->GetVerticalAngleMax() - sensor_->GetVerticalAngleMin()).GetAsRadian()));
  range_.max_range = sensor_->GetRangeMax();
  range_.min_range = sensor_->GetRangeMin();

  // start ros node
  if (!ros::isInitialized())
  {
    int argc = 0;
    char** argv = NULL;
    ros::init(argc,argv,"gazebo",ros::init_options::NoSigintHandler|ros::init_options::AnonymousName);
  }

  node_handle_ = new ros::NodeHandle(namespace_);
  publisher_ = node_handle_->advertise<sensor_msgs::Range>(topic_, 1);

  Reset();
  updateConnection = sensor_->GetLaserShape()->ConnectNewLaserScans(
        boost::bind(&GazeboRosSonar::Update, this));

  // activate RaySensor
  sensor_->SetActive(true);
}

void GazeboRosSonar::Reset()
{
  sensor_model_.reset();
}

////////////////////////////////////////////////////////////////////////////////
// Update the controller
void GazeboRosSonar::Update()
{
  common::Time sim_time = world->GetSimTime();
  double dt = (sim_time - last_time).Double();
//  if (last_time + updatePeriod > sim_time) return;

  // activate RaySensor if it is not yet active
  if (!sensor_->IsActive()) sensor_->SetActive(true);

  range_.header.stamp.sec  = (world->GetSimTime()).sec;
  range_.header.stamp.nsec = (world->GetSimTime()).nsec;

  // find ray with minimal range
  range_.range = std::numeric_limits<sensor_msgs::Range::_range_type>::max();
  for(int i = 0; i < sensor_->GetRangeCount(); ++i) {
    double ray = sensor_->GetLaserShape()->GetRange(i);
    if (ray < range_.range) range_.range = ray;
  }

  // add Gaussian noise (and limit to min/max range)
  if (range_.range < range_.max_range) {
    range_.range += sensor_model_.update(dt);
    if (range_.range < range_.min_range) range_.range = range_.min_range;
    if (range_.range > range_.max_range) range_.range = range_.max_range;
  }

  publisher_.publish(range_);

  // save last time stamp
  last_time = sim_time;
}

// Register this plugin with the simulator
GZ_REGISTER_SENSOR_PLUGIN(GazeboRosSonar)

} // namespace gazebo
