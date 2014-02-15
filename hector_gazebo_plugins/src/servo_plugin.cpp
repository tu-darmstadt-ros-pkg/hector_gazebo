/*
 *  Gazebo - Outdoor Multi-Robot Simulator
 *  Copyright (C) 2003
 *     Nate Koenig & Andrew Howard
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program; if not, write to the Free Software
 *  Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 */

#include <hector_gazebo_plugins/servo_plugin.h>
#include <gazebo/common/Events.hh>
#include <gazebo/physics/physics.hh>

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
  event::Events::DisconnectWorldUpdateBegin(updateConnection);
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
  if (_sdf->HasElement("firstServoAxis")) servo[FIRST].axis = _sdf->Get<math::Vector3>("firstServoAxis");
  if (_sdf->HasElement("secondServoName")) servo[SECOND].name = _sdf->Get<std::string>("secondServoName");
  if (_sdf->HasElement("secondServoAxis")) servo[SECOND].axis = _sdf->Get<math::Vector3>("secondServoAxis");
  if (_sdf->HasElement("thirdServoName")) servo[THIRD].name = _sdf->Get<std::string>("thirdServoName");
  if (_sdf->HasElement("thirdServoAxis")) servo[THIRD].axis = _sdf->Get<math::Vector3>("thirdServoAxis");
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

  prevUpdateTime = world->GetSimTime();
}

// Update the controller
void ServoPlugin::Update()
{
  // handle callbacks
  queue_.callAvailable();

  common::Time stepTime;
  stepTime = world->GetSimTime() - prevUpdateTime;

  if (controlPeriod == 0.0 || stepTime > controlPeriod) {
    CalculateVelocities();
    publish_joint_states();
    prevUpdateTime = world->GetSimTime();
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

    servo[FIRST].joint->SetMaxForce(0, maximumTorque);
    if (countOfServos > 1) {
      servo[SECOND].joint->SetMaxForce(0, maximumTorque);
      if (countOfServos > 2) {
        servo[THIRD].joint->SetMaxForce(0, maximumTorque);
      }
    }
  } else {
    servo[FIRST].joint->SetMaxForce(0, 0.0);
    if (countOfServos > 1) {
      servo[SECOND].joint->SetMaxForce(0, 0.0);
      if (countOfServos > 2) {
        servo[THIRD].joint->SetMaxForce(0, 0.0);
      }
    }
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

  math::Quaternion quat(transform.getRotation().getW(),transform.getRotation().getX(),transform.getRotation().getY(),transform.getRotation().getZ());

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
      if ((servo[FIRST].axis.z == 1) && (servo[SECOND].axis.y == 1)) {
        rotationConv = zyx;
        orderOfAxes[0] = 0;
        orderOfAxes[1] = 1;
      }
      else {
        if ((servo[FIRST].axis.x == 1) && (servo[SECOND].axis.y == 1)) {
          rotationConv = xyz;
          orderOfAxes[0] = 0;
          orderOfAxes[1] = 1;
        }
      }
      break;

    case 3:
      if ((servo[FIRST].axis.z == 1) && (servo[SECOND].axis.y == 1) && (servo[THIRD].axis.x == 1)) {
        rotationConv = zyx;
        orderOfAxes[0] = 0;
        orderOfAxes[1] = 1;
        orderOfAxes[2] = 2;
      }
      else if ((servo[FIRST].axis.x == 1) && (servo[SECOND].axis.y == 1) && (servo[THIRD].axis.z == 1)) {
        rotationConv = xyz;
        orderOfAxes[0] = 0;
        orderOfAxes[1] = 1;
        orderOfAxes[2] = 2;
      }
      break;

    case 1:
      if (servo[FIRST].axis.y == 1) {
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
      temp[0] =  2*(rotation_.x*rotation_.y + rotation_.w*rotation_.z);
      temp[1] =     rotation_.w*rotation_.w + rotation_.x*rotation_.x - rotation_.y*rotation_.y - rotation_.z*rotation_.z;
      temp[2] = -2*(rotation_.x*rotation_.z - rotation_.w*rotation_.y);
      temp[3] =  2*(rotation_.y*rotation_.z + rotation_.w*rotation_.x);
      temp[4] =     rotation_.w*rotation_.w - rotation_.x*rotation_.x - rotation_.y*rotation_.y + rotation_.z*rotation_.z;
      break;

    case xyz:
      temp[0] =  -2*(rotation_.y*rotation_.z - rotation_.w*rotation_.x);
      temp[1] =      rotation_.w*rotation_.w - rotation_.x*rotation_.x - rotation_.y*rotation_.y + rotation_.z*rotation_.z;
      temp[2] =   2*(rotation_.x*rotation_.z + rotation_.w*rotation_.y);
      temp[3] =  -2*(rotation_.x*rotation_.y - rotation_.w*rotation_.z);
      temp[4] =      rotation_.w*rotation_.w + rotation_.x*rotation_.x - rotation_.y*rotation_.y - rotation_.z*rotation_.z;
      break;

    default:
      gzthrow("joint order " << rotationConv << " not supported");
      break;
  }

  desAngle[0] = atan2(temp[0], temp[1]);
  desAngle[1] = asin(temp[2]);
  desAngle[2] = atan2(temp[3], temp[4]);

  actualAngle[FIRST] = servo[FIRST].joint->GetAngle(0).RADIAN();
  actualVel[FIRST] = servo[FIRST].joint->GetVelocity(0);
  ROS_DEBUG_NAMED("servo_plugin", "%s servo angle: %f - %f", servo[FIRST].name.c_str(), desAngle[orderOfAxes[FIRST]], actualAngle[FIRST]);
  servo[FIRST].velocity = ( proportionalControllerGain*(desAngle[orderOfAxes[FIRST]] - actualAngle[FIRST]) - derivativeControllerGain*actualVel[FIRST]);
  if (maximumVelocity > 0.0 && fabs(servo[FIRST].velocity) > maximumVelocity) servo[FIRST].velocity = (servo[FIRST].velocity > 0 ? maximumVelocity : -maximumVelocity);

  if (countOfServos > 1) {
    actualAngle[SECOND] = servo[SECOND].joint->GetAngle(0).RADIAN();
    actualVel[SECOND] = servo[SECOND].joint->GetVelocity(0);
    ROS_DEBUG_NAMED("servo_plugin", "%s servo angle: %f - %f", servo[SECOND].name.c_str(), desAngle[orderOfAxes[SECOND]], actualAngle[SECOND]);
    servo[SECOND].velocity = ( proportionalControllerGain*(desAngle[orderOfAxes[SECOND]] - actualAngle[SECOND]) - derivativeControllerGain*actualVel[SECOND]);
    if (maximumVelocity > 0.0 && fabs(servo[SECOND].velocity) > maximumVelocity) servo[SECOND].velocity = (servo[SECOND].velocity > 0 ? maximumVelocity : -maximumVelocity);

    if (countOfServos == 3) {
      actualAngle[THIRD] = servo[THIRD].joint->GetAngle(0).RADIAN();
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

  joint_state.header.stamp.sec = (world->GetSimTime()).sec;
  joint_state.header.stamp.nsec = (world->GetSimTime()).nsec;

  joint_state.name.resize(countOfServos);
  joint_state.position.resize(countOfServos);
  joint_state.velocity.resize(countOfServos);
  joint_state.effort.resize(countOfServos);

  for (unsigned int i = 0; i < countOfServos; i++) {
    joint_state.name[i] = servo[i].joint->GetName();
    joint_state.position[i] = servo[i].joint->GetAngle(0).RADIAN();
    joint_state.velocity[i] = servo[i].joint->GetVelocity(0);
    joint_state.effort[i] = servo[i].joint->GetForce(0u);
  }

  jointStatePub_.publish(joint_state);
}

} // namespace gazebo
