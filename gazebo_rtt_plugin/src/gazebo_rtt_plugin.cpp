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

#include <gazebo_rtt_plugin/gazebo_rtt_plugin.h>
#include <gazebo_rtt_plugin/gazebo_activity.h>

#include <rtt/deployment/ComponentLoader.hpp>
#include <rtt/os/main.h>

#include <rtt/os/targets/gnulinux-config.h>
#ifdef PLUGINS_ENABLE_MARSHALLING
  #include <rtt/marsh/Marshalling.hpp>
#endif

#ifndef ORO_ROS_PROTOCOL_ID
  #define ORO_ROS_PROTOCOL_ID 3
#endif

namespace gazebo
{

class RTTPlugin::RTTOSInitializer
{
public:
  RTTOSInitializer() {
    __os_init(0, 0);
    RTT::Logger::Instance()->setLogLevel(RTT::Logger::Info);
    RTT::os::TimeService::Instance()->enableSystemClock(false);
  }

  ~RTTOSInitializer() {
    __os_exit();
  }

  static boost::shared_ptr<RTTOSInitializer> Instance() {
    boost::shared_ptr<RTTOSInitializer> ptr;
    if (weak_ptr.expired()) {
      ptr.reset(new RTTOSInitializer());
      weak_ptr = ptr;
    } else {
      ptr = weak_ptr.lock();
    }
    return ptr;
  }

private:
  static boost::weak_ptr<RTTPlugin::RTTOSInitializer> weak_ptr;
};
boost::weak_ptr<RTTPlugin::RTTOSInitializer> RTTPlugin::RTTOSInitializer::weak_ptr;

RTTPlugin::RTTPlugin()
  : rttOsInitializer(RTTOSInitializer::Instance())
{
}

RTTPlugin::~RTTPlugin()
{
  if (taskContext) {
    taskContext->stop();
    taskContext->cleanup();
  }
}

void RTTPlugin::Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf)
{
  // Store the pointer to the model
  this->model = _parent;

  // Store the pointer to the world
  this->world = _parent->GetWorld();

  // Load RTT component packages and libraries
  RTT::ComponentLoader::Instance()->import("rtt_rosnode", "");
  if (_sdf->HasElement("import")) {
    RTT::ComponentLoader::Instance()->import(_sdf->GetValueString("import"), "");
  }
  if (_sdf->HasElement("library")) {
    RTT::ComponentLoader::Instance()->loadLibrary(_sdf->GetValueString("library"));
  }

  // Create an instance of the component
  std::string component = _sdf->GetValueString("component");
  std::string name = _sdf->GetValueString("name");
  if (name.empty()) name = component;

  taskContext.reset(RTT::ComponentLoader::Instance()->loadComponent(name, component));
  if (!taskContext) {
    gzerr << "Could not create a TaskContext for component " << component << std::endl;
    return;
  }

  // set Activity
  GazeboActivity *activity = new GazeboActivity(name);
  activity->Load(world, _sdf);
  taskContext->setActivity(activity);

  // load configuration settings from marshalling
#ifdef PLUGINS_ENABLE_MARSHALLING
  if (_sdf->HasElement("configuration")) {
    taskContext->getProvider<RTT::Marshalling>("marshalling")->loadProperties(_sdf->GetValueString("configuration"));
  }
#endif

  // set TaskContext's properties
  sdf::ElementPtr property = _sdf->GetElement("property");
  while(property) {
    std::string name = property->GetValueString("name");
    RTT::base::PropertyBase *prop = taskContext->getProperty(name);
    if (!prop) continue;

    if (prop->getType() == "string")
      RTT::Property<std::string>(prop).set(property->GetValueString());
    else if (prop->getType() == "double")
      RTT::Property<double>(prop).set(property->GetValueDouble());
    else if (prop->getType() == "float")
      RTT::Property<float>(prop).set(property->GetValueFloat());
    else if (prop->getType() == "int")
      RTT::Property<int>(prop).set(property->GetValueInt());
    else if (prop->getType() == "uint")
      RTT::Property<uint>(prop).set(property->GetValueUInt());
    else if (prop->getType() == "char")
      RTT::Property<char>(prop).set(property->GetValueChar());
    else if (prop->getType() == "bool")
      RTT::Property<bool>(prop).set(property->GetValueBool());
    else
      gzerr << "Property " << name << " has an unknown type. Will not use it." << std::endl;

    property = property->GetNextElement("property");
  }

  // configure the TaskContext
  if (!taskContext->configure()) {
    gzerr << "Failed to configure TaskContext " << taskContext->getName() << std::endl;
    return;
  }

  // create Streams
  sdf::ElementPtr port = _sdf->GetElement("port");
  while(port) {
    std::string name;
    RTT::ConnPolicy conn_policy;
    conn_policy.transport = ORO_ROS_PROTOCOL_ID;

    if (!port->HasAttribute("name")) continue;
    name = port->GetAttribute("name")->GetAsString();
    if (port->HasAttribute("topic"))
      conn_policy.name_id = port->GetAttribute("topic")->GetAsString();
    else
      conn_policy.name_id = name;
    if (port->HasAttribute("queue_size")) conn_policy.size = boost::lexical_cast<int>(port->GetAttribute("queue_size")->GetAsString());
    conn_policy.type = conn_policy.size > 1 ? RTT::ConnPolicy::BUFFER : RTT::ConnPolicy::DATA;

    RTT::base::PortInterface *port_interface = taskContext->getPort(name);
    if (port_interface) {
      port_interface->createStream(conn_policy);

    } else {
      gzwarn << "Component '" << taskContext->getName() << "' has no port named '" << name << "'" << std::endl;
    }

    port = port->GetNextElement("port");
  }

  // Listen to the update event. This event is broadcast every
  // simulation iteration.
  this->updateConnection = event::Events::ConnectWorldUpdateStart(
      boost::bind(&RTTPlugin::OnUpdate, this));
}

void RTTPlugin::Init()
{
  // do nothing for now
}

// Called by the world update start event
void RTTPlugin::OnUpdate()
{
  if (!taskContext) return;

  // start TaskContext
  if (!taskContext->isRunning()) {
    if (!taskContext->start()) {
      gzerr << "Failed to start TaskContext " << taskContext->getName() << std::endl;
      return;
    }
  }

  taskContext->update();
}

void RTTPlugin::Reset()
{
  if (!taskContext) return;
  taskContext->stop();
}

// Register this plugin with the simulator
GZ_REGISTER_MODEL_PLUGIN(RTTPlugin)

} // namespace gazebo
