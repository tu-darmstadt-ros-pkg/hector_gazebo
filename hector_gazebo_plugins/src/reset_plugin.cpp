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

#include <hector_gazebo_plugins/reset_plugin.h>

#include <gazebo/Global.hh>
#include <gazebo/XMLConfig.hh>
#include <gazebo/Simulator.hh>
#include <gazebo/gazebo.h>
#include <gazebo/World.hh>
#include <gazebo/PhysicsEngine.hh>
#include <gazebo/GazeboError.hh>
#include <gazebo/ControllerFactory.hh>

using namespace gazebo;

GZ_REGISTER_DYNAMIC_CONTROLLER("reset_plugin", GazeboResetPlugin)

GazeboResetPlugin::GazeboResetPlugin(Entity *parent)
   : Controller(parent)
{
  if (!ros::isInitialized())
  {
    int argc = 0;
    char** argv = NULL;
    ros::init(argc,argv, "gazebo", ros::init_options::NoSigintHandler|ros::init_options::AnonymousName);
  }

  Param::Begin(&parameters);
  namespace_param_ = new ParamT<std::string>("robotNamespace", "", false);
  topic_param_ = new ParamT<std::string>("topicName", "/syscommand", false);
  Param::End();
}

////////////////////////////////////////////////////////////////////////////////
// Destructor
GazeboResetPlugin::~GazeboResetPlugin()
{
  delete namespace_param_;
  delete topic_param_;
}

////////////////////////////////////////////////////////////////////////////////
// Load the controller
void GazeboResetPlugin::LoadChild(XMLConfigNode *node)
{
  namespace_param_->Load(node);
  namespace_ = namespace_param_->GetValue();
  topic_param_->Load(node);
  topic_ = topic_param_->GetValue();
}

///////////////////////////////////////////////////////
// Initialize the controller
void GazeboResetPlugin::InitChild()
{
  node_handle_ = new ros::NodeHandle(namespace_);

  ros::AdvertiseOptions aso = ros::AdvertiseOptions::create<std_msgs::String>(
    topic_, 1,
    ros::SubscriberStatusCallback(), ros::SubscriberStatusCallback(),
    ros::VoidPtr(), &this->callback_queue_);
  publisher_ = node_handle_->advertise(aso);

}

////////////////////////////////////////////////////////////////////////////////
// Update the controller
void GazeboResetPlugin::UpdateChild()
{
}

////////////////////////////////////////////////////////////////////////////////
// Reset the controller
void GazeboResetPlugin::ResetChild()
{
  std_msgs::String command;
  command.data = "reset";
  publisher_.publish(command);
}

////////////////////////////////////////////////////////////////////////////////
// Finalize the controller
void GazeboResetPlugin::FiniChild()
{
  node_handle_->shutdown();
  delete node_handle_;
}
