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

#include <gazebo/Sensor.hh>
#include <gazebo/Global.hh>
#include <gazebo/XMLConfig.hh>
#include <gazebo/Simulator.hh>
#include <gazebo/gazebo.h>
#include <gazebo/World.hh>
#include <gazebo/PhysicsEngine.hh>
#include <gazebo/GazeboError.hh>
#include <gazebo/ControllerFactory.hh>

static const double EARTH_RADIUS = 6371000.0;

using namespace gazebo;

GZ_REGISTER_DYNAMIC_CONTROLLER("hector_gazebo_ros_gps", GazeboRosGps)

GazeboRosGps::GazeboRosGps(Entity *parent)
   : Controller(parent)
   , position_error_model_(parameters)
   , velocity_error_model_(parameters, "velocity")
{
  parent_ = dynamic_cast<Model*>(parent);
  if (!parent_) gzthrow("GazeboRosGps controller requires a Model as its parent");

  if (!ros::isInitialized())
  {
    int argc = 0;
    char** argv = NULL;
    ros::init(argc,argv, "gazebo", ros::init_options::NoSigintHandler|ros::init_options::AnonymousName);
  }

  Param::Begin(&parameters);
  namespace_ = new ParamT<std::string>("robotNamespace", "", false);
  body_name_ = new ParamT<std::string>("bodyName", "", true);
  frame_id_ = new ParamT<std::string>("frameId", "", false);
  fix_topic_ = new ParamT<std::string>("topicName", "fix", false);
  velocity_topic_ = new ParamT<std::string>("velocityTopicName", "fix_velocity", false);

  reference_latitude_ = new ParamT<double>("referenceLatitude", 49.9, false);
  reference_longitude_ = new ParamT<double>("referenceLongitude", 8.9, false);
  reference_heading_ = new ParamT<double>("referenceHeading", 0.0, false);
  reference_altitude_ = new ParamT<double>("referenceAltitude", 0.0, false);
  status_ = new ParamT<sensor_msgs::NavSatStatus::_status_type>("status", sensor_msgs::NavSatStatus::STATUS_FIX, false);
  service_ = new ParamT<sensor_msgs::NavSatStatus::_service_type>("service", sensor_msgs::NavSatStatus::SERVICE_GPS, false);
  Param::End();
}

////////////////////////////////////////////////////////////////////////////////
// Destructor
GazeboRosGps::~GazeboRosGps()
{
  delete namespace_;
  delete body_name_;
  delete frame_id_;
  delete fix_topic_;
  delete velocity_topic_;
  delete reference_latitude_;
  delete reference_longitude_;
  delete reference_heading_;
  delete reference_altitude_;
  delete status_;
  delete service_;
}

////////////////////////////////////////////////////////////////////////////////
// Load the controller
void GazeboRosGps::LoadChild(XMLConfigNode *node)
{
  namespace_->Load(node);
  body_name_->Load(node);
  frame_id_->Load(node);
  fix_topic_->Load(node);
  velocity_topic_->Load(node);

  // assert that the body by body_name_ exists
  body_ = dynamic_cast<Body*>(parent_->GetBody(**body_name_));
  if (!body_) gzthrow("gazebo_quadrotor_simple_controller plugin error: body_name_: " << **body_name_ << "does not exist\n");

  reference_latitude_->Load(node);
  reference_longitude_->Load(node);
  reference_heading_->Load(node);
  reference_altitude_->Load(node);
  status_->Load(node);
  service_->Load(node);

  position_error_model_.Load(node);
  velocity_error_model_.Load(node);

  reference_heading_->SetValue(**reference_heading_ * M_PI/180.0); // convert to radians

  fix_.header.frame_id = **frame_id_;
  fix_.status.status  = **status_;
  fix_.status.service = **service_;
  velocity_.header.frame_id = **frame_id_;
}

///////////////////////////////////////////////////////
// Initialize the controller
void GazeboRosGps::InitChild()
{
  node_handle_ = new ros::NodeHandle(**namespace_);
  fix_publisher_ = node_handle_->advertise<sensor_msgs::NavSatFix>(**fix_topic_, 10);
  velocity_publisher_ = node_handle_->advertise<geometry_msgs::Vector3Stamped>(**velocity_topic_, 10);
}

////////////////////////////////////////////////////////////////////////////////
// Update the controller
void GazeboRosGps::UpdateChild()
{
  Time sim_time = Simulator::Instance()->GetSimTime();
  double dt = (sim_time - lastUpdate).Double();

  Pose3d pose = body_->GetWorldPose();

  gazebo::Vector3 velocity = velocity_error_model_(body_->GetWorldLinearVel(), dt);
  position_error_model_.setCurrentDrift(position_error_model_.getCurrentDrift() + velocity_error_model_.getCurrentError() * dt);
  gazebo::Vector3 position = position_error_model_(pose.pos, dt);

  fix_.header.stamp = ros::Time(sim_time.sec, sim_time.nsec);
  velocity_.header.stamp = fix_.header.stamp;

  fix_.latitude  = **reference_latitude_  + ( cos(**reference_heading_) * position.x + sin(**reference_heading_) * position.y) / EARTH_RADIUS * 180.0/M_PI;
  fix_.longitude = **reference_longitude_ - (-sin(**reference_heading_) * position.x + cos(**reference_heading_) * position.y) / EARTH_RADIUS * 180.0/M_PI * cos(fix_.latitude);
  fix_.altitude  = **reference_altitude_  + position.z;
  fix_.position_covariance_type = sensor_msgs::NavSatFix::COVARIANCE_TYPE_UNKNOWN;
  velocity_.vector.x =  cos(**reference_heading_) * velocity.x + sin(**reference_heading_) * velocity.y;
  velocity_.vector.y = -sin(**reference_heading_) * velocity.x + cos(**reference_heading_) * velocity.y;
  velocity_.vector.z = velocity.z;

  fix_publisher_.publish(fix_);
  velocity_publisher_.publish(velocity_);
}

////////////////////////////////////////////////////////////////////////////////
// Finalize the controller
void GazeboRosGps::FiniChild()
{
  node_handle_->shutdown();
  delete node_handle_;
}

