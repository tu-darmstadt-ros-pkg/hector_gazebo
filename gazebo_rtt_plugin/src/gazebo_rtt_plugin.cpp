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

#include <rtt/internal/GlobalService.hpp>

#ifndef ORO_ROS_PROTOCOL_ID
  #define ORO_ROS_PROTOCOL_ID 3
#endif

#include <ros/names.h>

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
  if (_sdf->HasElement("import")) {
    std::string package = _sdf->GetElement("import")->Get<std::string>();

    bool success = false; /* RTT::ComponentLoader::Instance()->import(package, ""); */
    if (!success) {
      if (!RTT::internal::GlobalService::Instance()->hasService("ros")) {
        RTT::ComponentLoader::Instance()->import("rtt_ros", "");
      }
      RTT::ServicePtr ros_service = RTT::internal::GlobalService::Instance()->getService("ros");

      if (ros_service) {
        RTT::OperationCaller<bool(std::string)> import = ros_service->getOperation("import");
        if (import.ready()) success = import(package);
      }
    }

    if (!success) {
      gzerr << "Could not import ROS package " << package << " using the ros.import service!" << std::endl;
    }
  }
  if (_sdf->HasElement("library")) {
    RTT::ComponentLoader::Instance()->loadLibrary(_sdf->GetElement("library")->Get<std::string>());
  }

  // Create an instance of the component
  std::string component;
  if (_sdf->HasElement("component")) component = _sdf->GetElement("component")->Get<std::string>();
  std::string name;
  if (_sdf->HasElement("name")) name = _sdf->GetElement("name")->Get<std::string>();
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
    boost::shared_ptr<RTT::Marshalling> marshalling = taskContext->getProvider<RTT::Marshalling>("marshalling");
    if (marshalling)
      marshalling->loadProperties(_sdf->GetElement("configuration")->Get<std::string>());
    else
      gzerr << "Could not load component configuration from " << _sdf->GetElement("configuration")->Get<std::string>() << std::endl;
  }
#endif

  // set TaskContext's properties
  if (_sdf->HasElement("property")) {
    sdf::ElementPtr property = _sdf->GetElement("property");
    while(property) {
      std::string name = property->Get<std::string>("name");
      RTT::base::PropertyBase *prop = taskContext->getProperty(name);
      if (!prop) {
        gzwarn << "Component '" << taskContext->getName() << "' has no property named '" << name << "'" << std::endl;
        property = property->GetNextElement("property");
        continue;
      }

      if (prop->getType() == "string")
        RTT::Property<std::string>(prop).set(property->Get<std::string>());
      else if (prop->getType() == "double")
        RTT::Property<double>(prop).set(property->Get<double>());
      else if (prop->getType() == "float")
        RTT::Property<float>(prop).set(property->Get<float>());
      else if (prop->getType() == "int")
        RTT::Property<int>(prop).set(property->Get<int>());
      else if (prop->getType() == "uint")
        RTT::Property<uint>(prop).set(property->Get<uint>());
      else if (prop->getType() == "char")
        RTT::Property<char>(prop).set(property->Get<char>());
      else if (prop->getType() == "bool")
        RTT::Property<bool>(prop).set(property->Get<bool>());
      else
        gzerr << "Property " << name << " has an unknown type. Will not use it." << std::endl;

      property = property->GetNextElement("property");
    }
  }

  // configure the TaskContext
  if (!taskContext->configure()) {
    gzerr << "Failed to configure TaskContext " << taskContext->getName() << std::endl;
    return;
  }

  // get robot namespace
  std::string robotNamespace;
  if (_sdf->HasElement("robotNamespace"))
    robotNamespace = _sdf->GetElement("robotNamespace")->Get<std::string>();

  // create Streams
  sdf::ElementPtr port = _sdf->GetElement("port");
  while(port) {
    std::string name = port->Get<std::string>("name");
    RTT::base::PortInterface *port_interface = taskContext->getPort(name);
    if (!port_interface) {
      gzwarn << "Component '" << taskContext->getName() << "' has no port named '" << name << "'" << std::endl;
      port = port->GetNextElement("port");
      continue;
    }

    RTT::ConnPolicy conn_policy;
    conn_policy.transport = ORO_ROS_PROTOCOL_ID;

    if (port->HasAttribute("topic") || port->HasElement("topic"))
      conn_policy.name_id = port->Get<std::string>("topic");
    else
      conn_policy.name_id = name;
    if (port->HasAttribute("queue_size") || port->HasElement("queue_size"))
      conn_policy.size = port->Get<int>("queue_size");
    conn_policy.type = conn_policy.size > 1 ? RTT::ConnPolicy::buffer(0).type : RTT::ConnPolicy::data().type;

    conn_policy.name_id = ros::names::resolve(robotNamespace, conn_policy.name_id, false);
    port_interface->createStream(conn_policy);
    port = port->GetNextElement("port");
  }
}

void RTTPlugin::Init()
{
  if (!taskContext) return;
  taskContext->start();
}

void RTTPlugin::Reset()
{
  if (!taskContext) return;
  taskContext->stop();
  Init();
}

// Register this plugin with the simulator
GZ_REGISTER_MODEL_PLUGIN(RTTPlugin)

} // namespace gazebo
