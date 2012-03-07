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

#include <gazebo/Sensor.hh>
#include <gazebo/Global.hh>
#include <gazebo/XMLConfig.hh>
#include <gazebo/Simulator.hh>
#include <gazebo/gazebo.h>
#include <gazebo/World.hh>
#include <gazebo/PhysicsEngine.hh>
#include <gazebo/GazeboError.hh>
#include <gazebo/ControllerFactory.hh>

using namespace gazebo;

GZ_REGISTER_DYNAMIC_CONTROLLER("hector_gazebo_ros_magnetic", GazeboRosMagnetic)

GazeboRosMagnetic::GazeboRosMagnetic(Entity *parent)
   : Controller(parent)
   , sensor_model_(parameters)
{
  parent_ = dynamic_cast<Model*>(parent);
  if (!parent_) gzthrow("GazeboRosMagnetic controller requires a Model as its parent");

  if (!ros::isInitialized())
  {
    int argc = 0;
    char** argv = NULL;
    ros::init(argc,argv, "gazebo", ros::init_options::NoSigintHandler|ros::init_options::AnonymousName);
  }

  Param::Begin(&parameters);
  namespace_ = new ParamT<std::string>("robotNamespace", "", false);
  body_name_ = new ParamT<std::string>("bodyName", "", true);
  topic_ = new ParamT<std::string>("topicName", "magnetic", false);

  magnitude_ = new ParamT<double>("magnitude", 1.0, false);
  reference_heading_ = new ParamT<double>("referenceHeading", 0.0, false);
  declination_ = new ParamT<double>("declination", 0.0, false);
  inclination_ = new ParamT<double>("inclination", 60.0, false);
  Param::End();
}

////////////////////////////////////////////////////////////////////////////////
// Destructor
GazeboRosMagnetic::~GazeboRosMagnetic()
{
  delete namespace_;
  delete body_name_;
  delete topic_;
  delete magnitude_;
  delete reference_heading_;
  delete declination_;
  delete inclination_;
}

////////////////////////////////////////////////////////////////////////////////
// Load the controller
void GazeboRosMagnetic::LoadChild(XMLConfigNode *node)
{
  namespace_->Load(node);
  body_name_->Load(node);

  // assert that the body by body_name_ exists
  body_ = dynamic_cast<Body*>(parent_->GetBody(**body_name_));
  if (!body_) gzthrow("gazebo_quadrotor_simple_controller plugin error: body_name_: " << **body_name_ << "does not exist\n");

  magnetic_field_.header.frame_id = body_->GetName();

  topic_->Load(node);

  magnitude_->Load(node);
  reference_heading_->Load(node);
  declination_->Load(node);
  inclination_->Load(node);

  reference_heading_->SetValue(**reference_heading_ * M_PI/180.0); // convert to radians
  declination_->SetValue(**declination_ * M_PI/180.0); // convert to radians
  inclination_->SetValue(**inclination_ * M_PI/180.0); // convert to radians

  // Note: Gazebo uses NorthWestUp coordinate system, heading and declination are compass headings
  magnetic_field_world_.x = **magnitude_ *  cos(**inclination_) * cos(**reference_heading_ - **declination_);
  magnetic_field_world_.y = **magnitude_ *  sin(**reference_heading_ - **declination_);
  magnetic_field_world_.z = **magnitude_ * -sin(**inclination_) * cos(**reference_heading_ - **declination_);

  sensor_model_.Load(node);
}

///////////////////////////////////////////////////////
// Initialize the controller
void GazeboRosMagnetic::InitChild()
{
  node_handle_ = new ros::NodeHandle(**namespace_);
  publisher_ = node_handle_->advertise<geometry_msgs::Vector3Stamped>(**topic_, 10);
}

////////////////////////////////////////////////////////////////////////////////
// Update the controller
void GazeboRosMagnetic::UpdateChild()
{
  Time sim_time = Simulator::Instance()->GetSimTime();
  double dt = (sim_time - lastUpdate).Double();

  Pose3d pose = body_->GetWorldPose();
  Vector3 field = sensor_model_(pose.rot.RotateVectorReverse(magnetic_field_world_), dt);

  magnetic_field_.header.stamp = ros::Time(sim_time.sec, sim_time.nsec);
  magnetic_field_.vector.x = field.x;
  magnetic_field_.vector.y = field.y;
  magnetic_field_.vector.z = field.z;

  publisher_.publish(magnetic_field_);
}

////////////////////////////////////////////////////////////////////////////////
// Finalize the controller
void GazeboRosMagnetic::FiniChild()
{
  node_handle_->shutdown();
  delete node_handle_;
}

