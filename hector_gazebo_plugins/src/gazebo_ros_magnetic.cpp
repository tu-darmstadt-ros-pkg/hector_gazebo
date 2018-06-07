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

#include <hector_gazebo_plugins/gazebo_ros_magnetic.h>
#include <gazebo/common/Events.hh>
#include <gazebo/physics/physics.hh>

static const double DEFAULT_MAGNITUDE           = 1.0;
static const double DEFAULT_REFERENCE_HEADING   = 0.0;
static const double DEFAULT_DECLINATION         = 0.0;
static const double DEFAULT_INCLINATION         = 60.0;

namespace gazebo {

GazeboRosMagnetic::GazeboRosMagnetic()
{
}

////////////////////////////////////////////////////////////////////////////////
// Destructor
GazeboRosMagnetic::~GazeboRosMagnetic()
{
  updateTimer.Disconnect(updateConnection);

  dynamic_reconfigure_server_.reset();

  node_handle_->shutdown();
  delete node_handle_;
}

////////////////////////////////////////////////////////////////////////////////
// Load the controller
void GazeboRosMagnetic::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
{
  world = _model->GetWorld();

  // load parameters
  if (_sdf->HasElement("robotNamespace"))
    namespace_ = _sdf->GetElement("robotNamespace")->GetValue()->GetAsString();
  else
    namespace_.clear();

  if (!_sdf->HasElement("topicName"))
    topic_ = "magnetic";
  else
    topic_ = _sdf->GetElement("topicName")->Get<std::string>();


  if (_sdf->HasElement("bodyName"))
  {
    link_name_ = _sdf->GetElement("bodyName")->GetValue()->GetAsString();
    link = _model->GetLink(link_name_);
  }
  else
  {
    link = _model->GetLink();
    link_name_ = link->GetName();
  }

  if (!link)
  {
    ROS_FATAL("GazeboRosMagnetic plugin error: bodyName: %s does not exist\n", link_name_.c_str());
    return;
  }

  // default parameters
  frame_id_ = link_name_;
  magnitude_ = DEFAULT_MAGNITUDE;
  reference_heading_ = DEFAULT_REFERENCE_HEADING * M_PI/180.0;
  declination_ = DEFAULT_DECLINATION * M_PI/180.0;
  inclination_ = DEFAULT_INCLINATION * M_PI/180.0;

  if (_sdf->HasElement("frameId"))
    frame_id_ = _sdf->GetElement("frameId")->GetValue()->GetAsString();

  if (_sdf->HasElement("magnitude"))
    _sdf->GetElement("magnitude")->GetValue()->Get(magnitude_);

  if (_sdf->HasElement("referenceHeading"))
    if (_sdf->GetElement("referenceHeading")->GetValue()->Get(reference_heading_))
      reference_heading_ *= M_PI/180.0;

  if (_sdf->HasElement("declination"))
    if (_sdf->GetElement("declination")->GetValue()->Get(declination_))
      declination_ *= M_PI/180.0;

  if (_sdf->HasElement("inclination"))
    if (_sdf->GetElement("inclination")->GetValue()->Get(inclination_))
      inclination_ *= M_PI/180.0;

  // Note: Gazebo uses NorthWestUp coordinate system, heading and declination are compass headings
  magnetic_field_.header.frame_id = frame_id_;
#if (GAZEBO_MAJOR_VERSION >= 8)
  magnetic_field_world_.X() = magnitude_ *  cos(inclination_) * cos(reference_heading_ - declination_);
  magnetic_field_world_.Y() = magnitude_ *  cos(inclination_) * sin(reference_heading_ - declination_);
  magnetic_field_world_.Z() = magnitude_ * -sin(inclination_);
#else
  magnetic_field_world_.x = magnitude_ *  cos(inclination_) * cos(reference_heading_ - declination_);
  magnetic_field_world_.y = magnitude_ *  cos(inclination_) * sin(reference_heading_ - declination_);
  magnetic_field_world_.z = magnitude_ * -sin(inclination_);
#endif

  sensor_model_.Load(_sdf);

  // start ros node
  if (!ros::isInitialized())
  {
    int argc = 0;
    char** argv = NULL;
    ros::init(argc,argv,"gazebo",ros::init_options::NoSigintHandler|ros::init_options::AnonymousName);
  }

  node_handle_ = new ros::NodeHandle(namespace_);
  publisher_ = node_handle_->advertise<geometry_msgs::Vector3Stamped>(topic_, 1);

  // setup dynamic_reconfigure server
  dynamic_reconfigure_server_.reset(new dynamic_reconfigure::Server<SensorModelConfig>(ros::NodeHandle(*node_handle_, topic_)));
  dynamic_reconfigure_server_->setCallback(boost::bind(&SensorModel3::dynamicReconfigureCallback, &sensor_model_, _1, _2));

  Reset();

  // connect Update function
  updateTimer.Load(world, _sdf);
  updateConnection = updateTimer.Connect(boost::bind(&GazeboRosMagnetic::Update, this));
}

void GazeboRosMagnetic::Reset()
{
  updateTimer.Reset();
  sensor_model_.reset();
}

////////////////////////////////////////////////////////////////////////////////
// Update the controller
void GazeboRosMagnetic::Update()
{
#if (GAZEBO_MAJOR_VERSION >= 8)
  common::Time sim_time = world->SimTime();
  double dt = updateTimer.getTimeSinceLastUpdate().Double();

  ignition::math::Pose3d pose = link->WorldPose();
  ignition::math::Vector3d field = sensor_model_(pose.Rot().RotateVectorReverse(magnetic_field_world_), dt);

  magnetic_field_.header.stamp = ros::Time(sim_time.sec, sim_time.nsec);
  magnetic_field_.vector.x = field.X();
  magnetic_field_.vector.y = field.Y();
  magnetic_field_.vector.z = field.Z();
#else
  common::Time sim_time = world->GetSimTime();
  double dt = updateTimer.getTimeSinceLastUpdate().Double();

  math::Pose pose = link->GetWorldPose();
  math::Vector3 field = sensor_model_(pose.rot.RotateVectorReverse(magnetic_field_world_), dt);

  magnetic_field_.header.stamp = ros::Time(sim_time.sec, sim_time.nsec);
  magnetic_field_.vector.x = field.x;
  magnetic_field_.vector.y = field.y;
  magnetic_field_.vector.z = field.z;
#endif

  publisher_.publish(magnetic_field_);
}

// Register this plugin with the simulator
GZ_REGISTER_MODEL_PLUGIN(GazeboRosMagnetic)

} // namespace gazebo
