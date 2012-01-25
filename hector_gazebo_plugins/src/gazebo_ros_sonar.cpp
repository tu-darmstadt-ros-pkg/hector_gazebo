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

#include <gazebo/Sensor.hh>
#include <gazebo/Global.hh>
#include <gazebo/XMLConfig.hh>
#include <gazebo/Simulator.hh>
#include <gazebo/gazebo.h>
#include <gazebo/GazeboError.hh>
#include <gazebo/ControllerFactory.hh>

#include <limits>

using namespace gazebo;

GZ_REGISTER_DYNAMIC_CONTROLLER("hector_gazebo_ros_sonar", GazeboRosSonar)

GazeboRosSonar::GazeboRosSonar(Entity *parent)
   : Controller(parent)
   , sensor_model_(parameters)
{
  sensor_ = dynamic_cast<RaySensor*>(parent);
  if (!sensor_) gzthrow("GazeboRosSonar controller requires a RaySensor as its parent");

  if (!ros::isInitialized())
  {
    int argc = 0;
    char** argv = NULL;
    ros::init(argc,argv, "gazebo", ros::init_options::NoSigintHandler|ros::init_options::AnonymousName);
  }

  Param::Begin(&parameters);
  namespace_param_ = new ParamT<std::string>("robotNamespace", "", false);
  topic_param_ = new ParamT<std::string>("topicName", "", true);
  frame_id_param_ = new ParamT<std::string>("frameName", "", true);
  Param::End();
}

////////////////////////////////////////////////////////////////////////////////
// Destructor
GazeboRosSonar::~GazeboRosSonar()
{
  delete namespace_param_;
  delete topic_param_;
  delete frame_id_param_;
}

////////////////////////////////////////////////////////////////////////////////
// Load the controller
void GazeboRosSonar::LoadChild(XMLConfigNode *node)
{
  namespace_param_->Load(node);
  topic_param_->Load(node);
  frame_id_param_->Load(node);

  sensor_model_.Load(node);
}

///////////////////////////////////////////////////////
// Initialize the controller
void GazeboRosSonar::InitChild()
{
  range_.header.frame_id = **frame_id_param_;
  range_.radiation_type = sensor_msgs::Range::ULTRASOUND;
  range_.field_of_view = std::min(fabs((sensor_->GetMaxAngle() - sensor_->GetMinAngle()).GetAsRadian()), fabs((sensor_->GetVerticalMaxAngle() - sensor_->GetVerticalMinAngle().GetAsRadian()).GetAsRadian()));
  range_.max_range = sensor_->GetMaxRange();
  range_.min_range = sensor_->GetMinRange();

  sensor_->SetActive(false);
  node_handle_ = new ros::NodeHandle(**namespace_param_);
  publisher_ = node_handle_->advertise<sensor_msgs::Range>(**topic_param_, 10);
}

////////////////////////////////////////////////////////////////////////////////
// Update the controller
void GazeboRosSonar::UpdateChild()
{
  Time sim_time = Simulator::Instance()->GetSimTime();
  double dt = (sim_time - lastUpdate).Double();

  if (!sensor_->IsActive()) sensor_->SetActive(true);

  range_.header.stamp.sec  = (Simulator::Instance()->GetSimTime()).sec;
  range_.header.stamp.nsec = (Simulator::Instance()->GetSimTime()).nsec;

  range_.range = std::numeric_limits<sensor_msgs::Range::_range_type>::max();
  for(int i = 0; i < sensor_->GetRangeCount(); ++i) {
    double ray = sensor_->GetRange(i);
    if (ray < range_.range) range_.range = ray;
  }
  range_.range += sensor_model_.update(dt);

  publisher_.publish(range_);
}

////////////////////////////////////////////////////////////////////////////////
// Finalize the controller
void GazeboRosSonar::FiniChild()
{
  sensor_->SetActive(false);
  node_handle_->shutdown();
  delete node_handle_;
}

