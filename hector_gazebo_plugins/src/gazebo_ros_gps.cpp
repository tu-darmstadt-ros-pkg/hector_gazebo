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

#include <hector_gazebo_plugins/gazebo_ros_gps.h>
#include "common/Events.hh"
#include "physics/physics.h"

// WGS84 constants
static const double equatorial_radius = 6378137.0;
static const double flattening = 1.0/298.257223563;
static const double excentrity2 = 2*flattening - flattening*flattening;

// default reference position
static const double DEFAULT_REFERENCE_LATITUDE  = 49.9;
static const double DEFAULT_REFERENCE_LONGITUDE = 8.9;
static const double DEFAULT_REFERENCE_HEADING   = 0.0;
static const double DEFAULT_REFERENCE_ALTITUDE  = 0.0;

namespace gazebo {

GazeboRosGps::GazeboRosGps()
{
}

////////////////////////////////////////////////////////////////////////////////
// Destructor
GazeboRosGps::~GazeboRosGps()
{
  event::Events::DisconnectWorldUpdateStart(updateConnection);
  node_handle_->shutdown();
  delete node_handle_;
}

////////////////////////////////////////////////////////////////////////////////
// Load the controller
void GazeboRosGps::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
{
  world = _model->GetWorld();

  // load parameters
  if (!_sdf->HasElement("robotNamespace"))
    namespace_.clear();
  else
    namespace_ = _sdf->GetElement("robotNamespace")->GetValueString() + "/";

  if (!_sdf->HasElement("bodyName"))
  {
    link = _model->GetLink();
    link_name_ = link->GetName();
  }
  else {
    link_name_ = _sdf->GetElement("bodyName")->GetValueString();
    link = _model->GetLink(link_name_);
  }

  if (!link)
  {
    ROS_FATAL("GazeboRosGps plugin error: bodyName: %s does not exist\n", link_name_.c_str());
    return;
  }

  double update_rate = 4.0;
  if (_sdf->HasElement("updateRate")) update_rate = _sdf->GetElement("updateRate")->GetValueDouble();
  update_period = update_rate > 0.0 ? 1.0/update_rate : 0.0;

  if (!_sdf->HasElement("frameId"))
    frame_id_ = "/world";
  else
    frame_id_ = _sdf->GetElement("frameId")->GetValueString();

  if (!_sdf->HasElement("topicName"))
    fix_topic_ = "fix";
  else
    fix_topic_ = _sdf->GetElement("topicName")->GetValueString();

  if (!_sdf->HasElement("velocityTopicName"))
    velocity_topic_ = "fix_velocity";
  else
    velocity_topic_ = _sdf->GetElement("velocityTopicName")->GetValueString();

  if (!_sdf->HasElement("referenceLatitude"))
    reference_latitude_ = DEFAULT_REFERENCE_LATITUDE;
  else
    reference_latitude_ = _sdf->GetElement("referenceLatitude")->GetValueDouble();

  if (!_sdf->HasElement("referenceLongitude"))
    reference_longitude_ = DEFAULT_REFERENCE_LONGITUDE;
  else
    reference_longitude_ = _sdf->GetElement("referenceLongitude")->GetValueDouble();

  if (!_sdf->HasElement("referenceHeading"))
    reference_heading_ = DEFAULT_REFERENCE_HEADING * M_PI/180.0;
  else
    reference_heading_ = _sdf->GetElement("referenceLatitude")->GetValueDouble() * M_PI/180.0;

  if (!_sdf->HasElement("referenceAltitude"))
    reference_altitude_ = DEFAULT_REFERENCE_ALTITUDE;
  else
    reference_altitude_ = _sdf->GetElement("referenceLatitude")->GetValueDouble();

  if (!_sdf->HasElement("status"))
    status_ = sensor_msgs::NavSatStatus::STATUS_FIX;
  else
    status_ = _sdf->GetElement("status")->GetValueUInt();

  if (!_sdf->HasElement("service"))
    service_ = sensor_msgs::NavSatStatus::SERVICE_GPS;
  else
    service_ = _sdf->GetElement("service")->GetValueUInt();

  fix_.header.frame_id = frame_id_;
  fix_.status.status  = status_;
  fix_.status.service = service_;
  velocity_.header.frame_id = frame_id_;

  position_error_model_.Load(_sdf);
  velocity_error_model_.Load(_sdf, "velocity");

  // calculate earth radii
  double temp = 1.0 / (1.0 - excentrity2 * sin(reference_latitude_ * M_PI/180.0) * sin(reference_latitude_ * M_PI/180.0));
  double prime_vertical_radius = equatorial_radius * sqrt(temp);
  radius_north_ = prime_vertical_radius * (1 - excentrity2) * temp;
  radius_east_  = prime_vertical_radius * cos(reference_latitude_ * M_PI/180.0);

  // start ros node
  if (!ros::isInitialized())
  {
    int argc = 0;
    char** argv = NULL;
    ros::init(argc,argv,"gazebo",ros::init_options::NoSigintHandler|ros::init_options::AnonymousName);
  }

  node_handle_ = new ros::NodeHandle(namespace_);
  fix_publisher_ = node_handle_->advertise<sensor_msgs::NavSatFix>(fix_topic_, 10);
  velocity_publisher_ = node_handle_->advertise<geometry_msgs::Vector3Stamped>(velocity_topic_, 10);

  Reset();

  // New Mechanism for Updating every World Cycle
  // Listen to the update event. This event is broadcast every
  // simulation iteration.
  updateConnection = event::Events::ConnectWorldUpdateStart(
      boost::bind(&GazeboRosGps::Update, this));
}

void GazeboRosGps::Reset()
{
  last_time = world->GetSimTime();

  position_error_model_.reset();
  velocity_error_model_.reset();
}

////////////////////////////////////////////////////////////////////////////////
// Update the controller
void GazeboRosGps::Update()
{
  common::Time sim_time = world->GetSimTime();
  double dt = (sim_time - last_time).Double();
  if (last_time + update_period > sim_time) return;

  math::Pose pose = link->GetWorldPose();

  gazebo::math::Vector3 velocity = velocity_error_model_(link->GetWorldLinearVel(), dt);
  position_error_model_.setCurrentDrift(position_error_model_.getCurrentDrift() + velocity_error_model_.getCurrentError() * dt);
  gazebo::math::Vector3 position = position_error_model_(pose.pos, dt);

  fix_.header.stamp = ros::Time(sim_time.sec, sim_time.nsec);
  velocity_.header.stamp = fix_.header.stamp;

  fix_.latitude  = reference_latitude_  + ( cos(reference_heading_) * position.x + sin(reference_heading_) * position.y) / radius_north_ * 180.0/M_PI;
  fix_.longitude = reference_longitude_ - (-sin(reference_heading_) * position.x + cos(reference_heading_) * position.y) / radius_east_  * 180.0/M_PI;
  fix_.altitude  = reference_altitude_  + position.z;
  fix_.position_covariance_type = sensor_msgs::NavSatFix::COVARIANCE_TYPE_UNKNOWN;
  velocity_.vector.x =  cos(reference_heading_) * velocity.x + sin(reference_heading_) * velocity.y;
  velocity_.vector.y = -sin(reference_heading_) * velocity.x + cos(reference_heading_) * velocity.y;
  velocity_.vector.z = velocity.z;

  fix_publisher_.publish(fix_);
  velocity_publisher_.publish(velocity_);

  // save last time stamp
  last_time = sim_time;
}

// Register this plugin with the simulator
GZ_REGISTER_MODEL_PLUGIN(GazeboRosGps)

} // namespace gazebo
