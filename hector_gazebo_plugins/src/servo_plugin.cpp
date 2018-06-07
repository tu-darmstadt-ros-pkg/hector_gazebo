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

#include <hector_gazebo_plugins/servo_plugin.h>
#include <gazebo/common/Events.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/gazebo_config.h>

#include <sensor_msgs/JointState.h>

#if (GAZEBO_MAJOR_VERSION > 1) || (GAZEBO_MINOR_VERSION >= 2)
  #define RADIAN Radian
#else
  #define RADIAN GetAsRadian
#endif

namespace gazebo {

GZ_REGISTER_MODEL_PLUGIN(ServoPlugin)

enum
{
  FIRST = 0, SECOND = 1, THIRD = 2
};

enum
{
  xyz, zyx
};

// Constructor
ServoPlugin::ServoPlugin()
{
  rosnode_ = 0;
  transform_listener_ = 0;
}

// Destructor
ServoPlugin::~ServoPlugin()
{
#if (GAZEBO_MAJOR_VERSION >= 8)
  updateConnection.reset();
#else
  event::Events::DisconnectWorldUpdateBegin(updateConnection);
#endif
  delete transform_listener_;
  rosnode_->shutdown();
  delete rosnode_;
}

// Load the controller
void ServoPlugin::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
{
  // Get the world name.
  world = _model->GetWorld();

  // default parameters
  topicName = "drive";
  jointStateName = "joint_states";
  robotNamespace.clear();
  controlPeriod = 0;
  proportionalControllerGain = 8.0;
  derivativeControllerGain = 0.0;
  maximumVelocity = 0.0;
  maximumTorque = 0.0;

  // load parameters
  if (_sdf->HasElement("robotNamespace")) robotNamespace = _sdf->Get<std::string>("robotNamespace");
  if (_sdf->HasElement("topicName")) topicName = _sdf->Get<std::string>("topicName");
  if (_sdf->HasElement("jointStateName")) jointStateName = _sdf->Get<std::string>("jointStateName");
  if (_sdf->HasElement("firstServoName")) servo[FIRST].name = _sdf->Get<std::string>("firstServoName");
#if (GAZEBO_MAJOR_VERSION >= 8)
  if (_sdf->HasElement("firstServoAxis")) servo[FIRST].axis = _sdf->Get<ignition::math::Vector3d>("firstServoAxis");
#else
  if (_sdf->HasElement("firstServoAxis")) servo[FIRST].axis = _sdf->Get<math::Vector3>("firstServoAxis");
#endif
  if (_sdf->HasElement("secondServoName")) servo[SECOND].name = _sdf->Get<std::string>("secondServoName");
#if (GAZEBO_MAJOR_VERSION >= 8)
  if (_sdf->HasElement("secondServoAxis")) servo[SECOND].axis = _sdf->Get<ignition::math::Vector3d>("secondServoAxis");
#else
  if (_sdf->HasElement("secondServoAxis")) servo[SECOND].axis = _sdf->Get<math::Vector3>("secondServoAxis");
#endif
  if (_sdf->HasElement("thirdServoName")) servo[THIRD].name = _sdf->Get<std::string>("thirdServoName");
#if (GAZEBO_MAJOR_VERSION >= 8)
  if (_sdf->HasElement("thirdServoAxis")) servo[THIRD].axis = _sdf->Get<ignition::math::Vector3d>("thirdServoAxis");
#else
  if (_sdf->HasElement("thirdServoAxis")) servo[THIRD].axis = _sdf->Get<math::Vector3>("thirdServoAxis");
#endif
  if (_sdf->HasElement("proportionalControllerGain")) proportionalControllerGain = _sdf->Get<double>("proportionalControllerGain");
  if (_sdf->HasElement("derivativeControllerGain")) derivativeControllerGain = _sdf->Get<double>("derivativeControllerGain");
  if (_sdf->HasElement("maxVelocity")) maximumVelocity = _sdf->Get<double>("maxVelocity");
  if (_sdf->HasElement("torque")) maximumTorque = _sdf->Get<double>("torque");

  double controlRate = 0.0;
  if (_sdf->HasElement("controlRate")) controlRate = _sdf->Get<double>("controlRate");
  controlPeriod = controlRate > 0.0 ? 1.0/controlRate : 0.0;

  servo[FIRST].joint  = _model->GetJoint(servo[FIRST].name);
  servo[SECOND].joint = _model->GetJoint(servo[SECOND].name);
  servo[THIRD].joint  = _model->GetJoint(servo[THIRD].name);

  if (!servo[FIRST].joint)
    gzthrow("The controller couldn't get first joint");

  countOfServos = 1;
  if (servo[SECOND].joint) {
    countOfServos = 2;
    if (servo[THIRD].joint) {
      countOfServos = 3;
    }
  }
  else {
    if (servo[THIRD].joint) {
      gzthrow("The controller couldn't get second joint, but third joint was loaded");
    }
  }

  if (!ros::isInitialized()) {
    int argc = 0;
    char** argv = NULL;
    ros::init(argc, argv, "gazebo", ros::init_options::NoSigintHandler | ros::init_options::AnonymousName);
  }

  rosnode_ = new ros::NodeHandle(robotNamespace);

  transform_listener_ = new tf::TransformListener();
  transform_listener_->setExtrapolationLimit(ros::Duration(1.0));

  if (!topicName.empty()) {
    ros::SubscribeOptions so = ros::SubscribeOptions::create<geometry_msgs::QuaternionStamped>(topicName, 1,
                                                                                               boost::bind(&ServoPlugin::cmdCallback, this, _1),
                                                                                               ros::VoidPtr(), &queue_);
    sub_ = rosnode_->subscribe(so);
  }

  if (!jointStateName.empty()) {
    jointStatePub_ = rosnode_->advertise<sensor_msgs::JointState>(jointStateName, 10);
  }

  joint_state.header.frame_id = transform_listener_->resolve(_model->GetLink()->GetName());

  // New Mechanism for Updating every World Cycle
  // Listen to the update event. This event is broadcast every
  // simulation iteration.
  updateConnection = event::Events::ConnectWorldUpdateBegin(
      boost::bind(&ServoPlugin::Update, this));
}

// Initialize the controller
void ServoPlugin::Init()
{
  Reset();
}

// Reset
void ServoPlugin::Reset()
{
  // Reset orientation
  current_cmd.reset();

  enableMotors = true;

  servo[FIRST].velocity = 0;
  servo[SECOND].velocity = 0;
  servo[THIRD].velocity = 0;

#if (GAZEBO_MAJOR_VERSION >= 8)
  prevUpdateTime = world->SimTime();
#else
  prevUpdateTime = world->GetSimTime();
#endif
}

// Update the controller
void ServoPlugin::Update()
{
  // handle callbacks
  queue_.callAvailable();

  common::Time stepTime;
#if (GAZEBO_MAJOR_VERSION >= 8)
  stepTime = world->SimTime() - prevUpdateTime;
#else
  stepTime = world->GetSimTime() - prevUpdateTime;
#endif

  if (controlPeriod == 0.0 || stepTime > controlPeriod) {
    CalculateVelocities();
    publish_joint_states();
#if (GAZEBO_MAJOR_VERSION >= 8)
    prevUpdateTime = world->SimTime();
#else
    prevUpdateTime = world->GetSimTime();
#endif
  }

  if (enableMotors)
  {
    servo[FIRST].joint->SetVelocity(0, servo[FIRST].velocity);
    if (countOfServos > 1) {
      servo[SECOND].joint->SetVelocity(0, servo[SECOND].velocity);
      if (countOfServos > 2) {
        servo[THIRD].joint->SetVelocity(0, servo[THIRD].velocity);
      }
    }

#if (GAZEBO_MAJOR_VERSION > 4)
    servo[FIRST].joint->SetEffortLimit(0, maximumTorque);
    if (countOfServos > 1) {
      servo[SECOND].joint->SetEffortLimit(0, maximumTorque);
      if (countOfServos > 2) {
        servo[THIRD].joint->SetEffortLimit(0, maximumTorque);
      }
    }
#else
    servo[FIRST].joint->SetMaxForce(0, maximumTorque);
    if (countOfServos > 1) {
      servo[SECOND].joint->SetMaxForce(0, maximumTorque);
      if (countOfServos > 2) {
        servo[THIRD].joint->SetMaxForce(0, maximumTorque);
      }
    }
#endif
  } else {
#if (GAZEBO_MAJOR_VERSION > 4)
    servo[FIRST].joint->SetEffortLimit(0, 0.0);
    if (countOfServos > 1) {
      servo[SECOND].joint->SetEffortLimit(0, 0.0);
      if (countOfServos > 2) {
        servo[THIRD].joint->SetEffortLimit(0, 0.0);
      }
    }
#else
    servo[FIRST].joint->SetMaxForce(0, 0.0);
    if (countOfServos > 1) {
      servo[SECOND].joint->SetMaxForce(0, 0.0);
      if (countOfServos > 2) {
        servo[THIRD].joint->SetMaxForce(0, 0.0);
      }
    }
#endif
  }
}

void ServoPlugin::CalculateVelocities()
{
  tf::StampedTransform transform;
  boost::mutex::scoped_lock lock(mutex);

  if(!current_cmd){
    geometry_msgs::QuaternionStamped *default_cmd = new geometry_msgs::QuaternionStamped;
    default_cmd->header.frame_id = "base_stabilized";
    default_cmd->quaternion.w = 1;
    current_cmd.reset(default_cmd);
  }

  try{
    // ros::Time simTime(world->GetSimTime().sec, world->GetSimTime().nsec);
    transform_listener_->lookupTransform("base_link", current_cmd->header.frame_id, ros::Time(0), transform);
  }
  catch (tf::TransformException ex){
    ROS_DEBUG("%s",ex.what());
    servo[FIRST].velocity = 0.0;
    servo[SECOND].velocity = 0.0;
    servo[THIRD].velocity = 0.0;
    return;
  }

  rotation_.Set(current_cmd->quaternion.w, current_cmd->quaternion.x, current_cmd->quaternion.y, current_cmd->quaternion.z);

#if (GAZEBO_MAJOR_VERSION >= 8)
  ignition::math::Quaterniond quat(transform.getRotation().getW(),transform.getRotation().getX(),transform.getRotation().getY(),transform.getRotation().getZ());
#else
  math::Quaternion quat(transform.getRotation().getW(),transform.getRotation().getX(),transform.getRotation().getY(),transform.getRotation().getZ());
#endif

  rotation_ = quat * rotation_;

  double temp[5];
  double desAngle[3];
  double actualAngle[3] = {0.0, 0.0, 0.0};
  double actualVel[3] = {0.0, 0.0, 0.0};

  //TODO use countOfServos for calculation
  rotationConv = 99;
  orderOfAxes[0] = 99;
  orderOfAxes[1] = 99;
  orderOfAxes[2] = 99;

  switch(countOfServos) {
    case 2:
#if (GAZEBO_MAJOR_VERSION >= 8)
      if ((servo[FIRST].axis.Z() == 1) && (servo[SECOND].axis.Y() == 1)) {
#else
      if ((servo[FIRST].axis.z == 1) && (servo[SECOND].axis.y == 1)) {
#endif
        rotationConv = zyx;
        orderOfAxes[0] = 0;
        orderOfAxes[1] = 1;
      }
      else {
#if (GAZEBO_MAJOR_VERSION >= 8)
        if ((servo[FIRST].axis.X() == 1) && (servo[SECOND].axis.Y() == 1)) {
#else
        if ((servo[FIRST].axis.x == 1) && (servo[SECOND].axis.y == 1)) {
#endif
          rotationConv = xyz;
          orderOfAxes[0] = 0;
          orderOfAxes[1] = 1;
        }
      }
      break;

    case 3:
#if (GAZEBO_MAJOR_VERSION >= 8)
      if ((servo[FIRST].axis.Z() == 1) && (servo[SECOND].axis.Y() == 1) && (servo[THIRD].axis.X() == 1)) {
#else
      if ((servo[FIRST].axis.z == 1) && (servo[SECOND].axis.y == 1) && (servo[THIRD].axis.x == 1)) {
#endif
        rotationConv = zyx;
        orderOfAxes[0] = 0;
        orderOfAxes[1] = 1;
        orderOfAxes[2] = 2;
      }
#if (GAZEBO_MAJOR_VERSION >= 8)
      else if ((servo[FIRST].axis.X() == 1) && (servo[SECOND].axis.Y() == 1) && (servo[THIRD].axis.Z() == 1)) {
#else
      else if ((servo[FIRST].axis.x == 1) && (servo[SECOND].axis.y == 1) && (servo[THIRD].axis.z == 1)) {
#endif
        rotationConv = xyz;
        orderOfAxes[0] = 0;
        orderOfAxes[1] = 1;
        orderOfAxes[2] = 2;
      }
      break;

    case 1:
#if (GAZEBO_MAJOR_VERSION >= 8)
      if (servo[FIRST].axis.Y() == 1) {
#else
      if (servo[FIRST].axis.y == 1) {
#endif
         rotationConv = xyz;
         orderOfAxes[0] = 1;
      }
      break;

    default:
      gzthrow("Something went wrong. The count of servos is greater than 3");
      break;
  }

  switch(rotationConv)  {
    case zyx:
#if (GAZEBO_MAJOR_VERSION >= 8)
      temp[0] =  2*(rotation_.X()*rotation_.Y() + rotation_.W()*rotation_.Z());
      temp[1] =     rotation_.W()*rotation_.W() + rotation_.X()*rotation_.X() - rotation_.Y()*rotation_.Y() - rotation_.Z()*rotation_.Z();
      temp[2] = -2*(rotation_.X()*rotation_.Z() - rotation_.W()*rotation_.Y());
      temp[3] =  2*(rotation_.Y()*rotation_.Z() + rotation_.W()*rotation_.X());
      temp[4] =     rotation_.W()*rotation_.W() - rotation_.X()*rotation_.X() - rotation_.Y()*rotation_.Y() + rotation_.Z()*rotation_.Z();
#else
      temp[0] =  2*(rotation_.x*rotation_.y + rotation_.w*rotation_.z);
      temp[1] =     rotation_.w*rotation_.w + rotation_.x*rotation_.x - rotation_.y*rotation_.y - rotation_.z*rotation_.z;
      temp[2] = -2*(rotation_.x*rotation_.z - rotation_.w*rotation_.y);
      temp[3] =  2*(rotation_.y*rotation_.z + rotation_.w*rotation_.x);
      temp[4] =     rotation_.w*rotation_.w - rotation_.x*rotation_.x - rotation_.y*rotation_.y + rotation_.z*rotation_.z;
#endif
      break;

    case xyz:
#if (GAZEBO_MAJOR_VERSION >= 8)
      temp[0] =  -2*(rotation_.Y()*rotation_.Z() - rotation_.W()*rotation_.X());
      temp[1] =      rotation_.W()*rotation_.W() - rotation_.X()*rotation_.X() - rotation_.Y()*rotation_.Y() + rotation_.Z()*rotation_.Z();
      temp[2] =   2*(rotation_.X()*rotation_.Z() + rotation_.W()*rotation_.Y());
      temp[3] =  -2*(rotation_.X()*rotation_.Y() - rotation_.W()*rotation_.Z());
      temp[4] =      rotation_.W()*rotation_.W() + rotation_.X()*rotation_.X() - rotation_.Y()*rotation_.Y() - rotation_.Z()*rotation_.Z();
#else
      temp[0] =  -2*(rotation_.y*rotation_.z - rotation_.w*rotation_.x);
      temp[1] =      rotation_.w*rotation_.w - rotation_.x*rotation_.x - rotation_.y*rotation_.y + rotation_.z*rotation_.z;
      temp[2] =   2*(rotation_.x*rotation_.z + rotation_.w*rotation_.y);
      temp[3] =  -2*(rotation_.x*rotation_.y - rotation_.w*rotation_.z);
      temp[4] =      rotation_.w*rotation_.w + rotation_.x*rotation_.x - rotation_.y*rotation_.y - rotation_.z*rotation_.z;
#endif
      break;

    default:
      gzthrow("joint order " << rotationConv << " not supported");
      break;
  }

  desAngle[0] = atan2(temp[0], temp[1]);
  desAngle[1] = asin(temp[2]);
  desAngle[2] = atan2(temp[3], temp[4]);

#if (GAZEBO_MAJOR_VERSION >= 8)
  actualAngle[FIRST] = servo[FIRST].joint->Position(0);
#else
  actualAngle[FIRST] = servo[FIRST].joint->GetAngle(0).RADIAN();
#endif
  actualVel[FIRST] = servo[FIRST].joint->GetVelocity(0);
  ROS_DEBUG_NAMED("servo_plugin", "%s servo angle: %f - %f", servo[FIRST].name.c_str(), desAngle[orderOfAxes[FIRST]], actualAngle[FIRST]);
  servo[FIRST].velocity = ( proportionalControllerGain*(desAngle[orderOfAxes[FIRST]] - actualAngle[FIRST]) - derivativeControllerGain*actualVel[FIRST]);
  if (maximumVelocity > 0.0 && fabs(servo[FIRST].velocity) > maximumVelocity) servo[FIRST].velocity = (servo[FIRST].velocity > 0 ? maximumVelocity : -maximumVelocity);

  if (countOfServos > 1) {
#if (GAZEBO_MAJOR_VERSION >= 8)
    actualAngle[SECOND] = servo[SECOND].joint->Position(0);
#else
    actualAngle[SECOND] = servo[SECOND].joint->GetAngle(0).RADIAN();
#endif
    actualVel[SECOND] = servo[SECOND].joint->GetVelocity(0);
    ROS_DEBUG_NAMED("servo_plugin", "%s servo angle: %f - %f", servo[SECOND].name.c_str(), desAngle[orderOfAxes[SECOND]], actualAngle[SECOND]);
    servo[SECOND].velocity = ( proportionalControllerGain*(desAngle[orderOfAxes[SECOND]] - actualAngle[SECOND]) - derivativeControllerGain*actualVel[SECOND]);
    if (maximumVelocity > 0.0 && fabs(servo[SECOND].velocity) > maximumVelocity) servo[SECOND].velocity = (servo[SECOND].velocity > 0 ? maximumVelocity : -maximumVelocity);

    if (countOfServos == 3) {
#if (GAZEBO_MAJOR_VERSION >= 8)
      actualAngle[THIRD] = servo[THIRD].joint->Position(0);
#else
      actualAngle[THIRD] = servo[THIRD].joint->GetAngle(0).RADIAN();
#endif
      actualVel[THIRD] = servo[THIRD].joint->GetVelocity(0);
      ROS_DEBUG_NAMED("servo_plugin", "%s servo angle: %f - %f", servo[THIRD].name.c_str(), desAngle[orderOfAxes[THIRD]], actualAngle[THIRD]);
      servo[THIRD].velocity = ( proportionalControllerGain*(desAngle[orderOfAxes[THIRD]] - actualAngle[THIRD]) - derivativeControllerGain*actualVel[THIRD]);
      if (maximumVelocity > 0.0 && fabs(servo[THIRD].velocity) > maximumVelocity) servo[THIRD].velocity = (servo[THIRD].velocity > 0 ? maximumVelocity : -maximumVelocity);
    }
  }

  // Changed motors to be always on, which is probably what we want anyway
  enableMotors = true; //myIface->data->cmdEnableMotors > 0;
}

// NEW: Store the velocities from the ROS message
void ServoPlugin::cmdCallback(const geometry_msgs::QuaternionStamped::ConstPtr& cmd_msg)
{
  boost::mutex::scoped_lock lock(mutex);
  current_cmd = cmd_msg;
}

void ServoPlugin::publish_joint_states()
{
  if (!jointStatePub_) return;

#if (GAZEBO_MAJOR_VERSION >= 8)
  joint_state.header.stamp.sec = (world->SimTime()).sec;
  joint_state.header.stamp.nsec = (world->SimTime()).nsec;
#else
  joint_state.header.stamp.sec = (world->GetSimTime()).sec;
  joint_state.header.stamp.nsec = (world->GetSimTime()).nsec;
#endif

  joint_state.name.resize(countOfServos);
  joint_state.position.resize(countOfServos);
  joint_state.velocity.resize(countOfServos);
  joint_state.effort.resize(countOfServos);

  for (unsigned int i = 0; i < countOfServos; i++) {
    joint_state.name[i] = servo[i].joint->GetName();
#if (GAZEBO_MAJOR_VERSION >= 8)
    joint_state.position[i] = servo[i].joint->Position(0);
#else
    joint_state.position[i] = servo[i].joint->GetAngle(0).RADIAN();
#endif
    joint_state.velocity[i] = servo[i].joint->GetVelocity(0);
    joint_state.effort[i] = servo[i].joint->GetForce(0u);
  }

  jointStatePub_.publish(joint_state);
}

} // namespace gazebo
