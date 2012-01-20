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
// This code is based on the original gazebo_ros_imu plugin by Sachin Chitta and John Hsu:
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
/*
 * Desc: 3D position interface.
 * Author: Sachin Chitta and John Hsu
 * Date: 10 June 2008
 * SVN: $Id$
 */
//=================================================================================================

#include <hector_gazebo_plugins/gazebo_ros_imu.h>

#include <gazebo/Global.hh>
#include <gazebo/XMLConfig.hh>
#include <gazebo/Simulator.hh>
#include <gazebo/gazebo.h>
#include <gazebo/GazeboError.hh>
#include <gazebo/ControllerFactory.hh>

#include <gazebo/World.hh>
#include <gazebo/PhysicsEngine.hh>

using namespace gazebo;

GZ_REGISTER_DYNAMIC_CONTROLLER("hector_gazebo_ros_imu", GazeboRosIMU);

#define DEBUG_OUTPUT
#ifdef DEBUG_OUTPUT
  #include <geometry_msgs/PoseStamped.h>
  static ros::Publisher debugPublisher;
#endif // DEBUG_OUTPUT

////////////////////////////////////////////////////////////////////////////////
// Constructor
GazeboRosIMU::GazeboRosIMU(Entity *parent)
   : Controller(parent)
   , accelModel(this->parameters, "accel")
   , rateModel(this->parameters, "rate")
   , headingModel(this->parameters, "heading")
{
   this->myParent = dynamic_cast<Model*>(this->parent);

   if (!this->myParent)
      gzthrow("GazeboRosIMU controller requires a Model as its parent");

  Param::Begin(&this->parameters);
  this->robotNamespaceP = new ParamT<std::string>("robotNamespace", "/", 0);
  this->bodyNameP = new ParamT<std::string>("bodyName", "", 0);
  this->topicNameP = new ParamT<std::string>("topicName", "", 1);
  this->rpyOffsetsP    = new ParamT<Vector3>("rpyOffsets", Vector3(0,0,0),0);
  this->gaussianNoiseP = new ParamT<double>("gaussianNoise",0.0,0);
  this->serviceNameP = new ParamT<std::string>("serviceName", "imu/calibrate", 0);
  Param::End();
}

////////////////////////////////////////////////////////////////////////////////
// Destructor
GazeboRosIMU::~GazeboRosIMU()
{
  delete this->robotNamespaceP;
  delete this->bodyNameP;
  delete this->topicNameP;
  delete this->rpyOffsetsP;
  delete this->gaussianNoiseP;
  delete this->serviceNameP;
  delete this->rosnode_;
}

////////////////////////////////////////////////////////////////////////////////
// Load the controller
void GazeboRosIMU::LoadChild(XMLConfigNode *node)
{
  this->robotNamespaceP->Load(node);
  this->robotNamespace = this->robotNamespaceP->GetValue();
  if (!ros::isInitialized())
  {
    int argc = 0;
    char** argv = NULL;
    ros::init(argc,argv, "gazebo", ros::init_options::NoSigintHandler|ros::init_options::AnonymousName);
  }

  this->rosnode_ = new ros::NodeHandle(this->robotNamespace);

  this->bodyNameP->Load(node);
  this->bodyName = this->bodyNameP->GetValue();

  // assert that the body by bodyName exists
  if (dynamic_cast<Body*>(this->myParent->GetBody(this->bodyName)) == NULL)
    ROS_FATAL("gazebo_ros_imu plugin error: bodyName: %s does not exist\n",this->bodyName.c_str());

  this->myBody = dynamic_cast<Body*>(this->myParent->GetBody(this->bodyName));

  this->topicNameP->Load(node);
  this->topicName = this->topicNameP->GetValue();

  accelModel.Load(node);
  rateModel.Load(node);
  headingModel.Load(node);

  // also use old configuration variables from gazebo_ros_imu
  this->rpyOffsetsP->Load(node);
  Vector3 rpyOffsets = this->rpyOffsetsP->GetValue();
  if (this->accelModel.offset.y == 0.0 && rpyOffsets.x != 0.0) this->accelModel.offset.y = -rpyOffsets.x * 9.8065;
  if (this->accelModel.offset.x == 0.0 && rpyOffsets.y != 0.0) this->accelModel.offset.x =  rpyOffsets.y * 9.8065;
  if (this->headingModel.offset == 0.0 && rpyOffsets.z != 0.0) this->headingModel.offset =  rpyOffsets.z;
  this->gaussianNoiseP->Load(node);
  double gaussianNoise = this->gaussianNoiseP->GetValue();
  if (gaussianNoise != 0.0) {
    this->accelModel.gaussian_noise = gaussianNoise;
    this->rateModel.gaussian_noise  = gaussianNoise;
  }

  // fill in constant covariance matrix
  this->imuMsg.angular_velocity_covariance[0] = this->rateModel.gaussian_noise.z*this->rateModel.gaussian_noise.z;
  this->imuMsg.angular_velocity_covariance[4] = this->rateModel.gaussian_noise.y*this->rateModel.gaussian_noise.y;
  this->imuMsg.angular_velocity_covariance[8] = this->rateModel.gaussian_noise.x*this->rateModel.gaussian_noise.x;
  this->imuMsg.linear_acceleration_covariance[0] = this->accelModel.gaussian_noise.z*this->accelModel.gaussian_noise.z;
  this->imuMsg.linear_acceleration_covariance[4] = this->accelModel.gaussian_noise.y*this->accelModel.gaussian_noise.y;
  this->imuMsg.linear_acceleration_covariance[8] = this->accelModel.gaussian_noise.x*this->accelModel.gaussian_noise.x;

  if (this->topicName != "")
  {
#ifdef USE_CBQ
    ros::AdvertiseOptions ao = ros::AdvertiseOptions::create<sensor_msgs::Imu>(
      this->topicName, 1,
      ros::SubscriberStatusCallback(), ros::SubscriberStatusCallback(),
      ros::VoidPtr(), &this->callback_queue_);
    this->pub_ = this->rosnode_->advertise(ao);
#else
    this->pub_ = this->rosnode_->advertise<sensor_msgs::Imu>(this->topicName,10);
#endif
  }

#ifdef DEBUG_OUTPUT
  debugPublisher = this->rosnode_->advertise<geometry_msgs::PoseStamped>(this->topicName+"/pose", 10);
#endif // DEBUG_OUTPUT

  // add service call version for position change
  this->serviceNameP->Load(node);
  this->serviceName = this->serviceNameP->GetValue();
  // advertise services on the custom queue
  ros::AdvertiseServiceOptions aso = ros::AdvertiseServiceOptions::create<std_srvs::Empty>(
      this->serviceName,boost::bind( &GazeboRosIMU::ServiceCallback, this, _1, _2 ), ros::VoidPtr(), &this->callback_queue_);
  this->srv_ = this->rosnode_->advertiseService(aso);

  aso = ros::AdvertiseServiceOptions::create<hector_gazebo_plugins::SetBias>(
        this->topicName+"/set_accel_bias", boost::bind( &GazeboRosIMU::SetAccelBiasCallback, this, _1, _2 ), ros::VoidPtr(), &this->callback_queue_);
  this->accelBiasService = this->rosnode_->advertiseService(aso);
  aso = ros::AdvertiseServiceOptions::create<hector_gazebo_plugins::SetBias>(
        this->topicName+"/set_rate_bias", boost::bind( &GazeboRosIMU::SetRateBiasCallback, this, _1, _2 ), ros::VoidPtr(), &this->callback_queue_);
  this->rateBiasService  = this->rosnode_->advertiseService(aso);
}

////////////////////////////////////////////////////////////////////////////////
// returns true always, imu is always calibrated in sim
bool GazeboRosIMU::ServiceCallback(std_srvs::Empty::Request &req,
                                        std_srvs::Empty::Response &res)
{
  this->lock.lock();
  this->rateModel.reset();
  this->lock.unlock();
  return true;
}

bool GazeboRosIMU::SetAccelBiasCallback(hector_gazebo_plugins::SetBias::Request &req, hector_gazebo_plugins::SetBias::Response &res)
{
  this->lock.lock();
  this->accelModel.reset(Vector3(req.bias.x, req.bias.y, req.bias.z));
  this->lock.unlock();
  return true;
}

bool GazeboRosIMU::SetRateBiasCallback(hector_gazebo_plugins::SetBias::Request &req, hector_gazebo_plugins::SetBias::Response &res)
{
  this->lock.lock();
  this->rateModel.reset(Vector3(req.bias.x, req.bias.y, req.bias.z));
  this->lock.unlock();
  return true;
}

////////////////////////////////////////////////////////////////////////////////
// Initialize the controller
void GazeboRosIMU::InitChild()
{
  this->last_time = Simulator::Instance()->GetSimTime();
#ifdef USE_CBQ
  // start custom queue for imu
  this->callback_queue_thread_ = boost::thread( boost::bind( &GazeboRosIMU::CallbackQueueThread,this ) );
#endif
}

////////////////////////////////////////////////////////////////////////////////
// Update the controller
void GazeboRosIMU::UpdateChild()
{
  Pose3d pose;
  Quatern rot;
  Vector3 pos;

  // Get Pose/Orientation ///@todo: verify correctness
  pose = this->myBody->GetWorldPose();
  // apply xyz offsets and get position and rotation components
  pos = pose.pos;
  rot = pose.rot;
  // std::cout << " --------- GazeboRosIMU rot " << rot.x << ", " << rot.y << ", " << rot.z << ", " << rot.u << std::endl;

  gazebo::Time cur_time = Simulator::Instance()->GetSimTime();
  double dt = cur_time - this->last_time;

  this->lock.lock();

  // get Acceleration, Rates and Gravity
  this->accel = this->myBody->GetRelativeLinearAccel(); // get acceleration in body frame
  this->rate  = this->myBody->GetRelativeAngularVel(); // get angular rate in body frame
  this->gravity       = World::Instance()->GetPhysicsEngine()->GetGravity();
  this->gravity_body  = rot.RotateVector(this->gravity);
  double gravity_length = this->gravity.GetLength();
  ROS_DEBUG_NAMED("hector_gazebo_ros_imu", "gravity_world = [%g %g %g]", this->gravity.x, this->gravity.y, this->gravity.z);

  // add gravity vector to body acceleration
  this->accel = this->accel - this->gravity_body;

  // update sensor models
  this->accel = this->accel + accelModel.update(dt);
  this->rate  = this->rate  + rateModel.update(dt);
  headingModel.update(dt);
  ROS_DEBUG_NAMED("hector_gazebo_ros_imu", "Current errors: accel = [%g %g %g], rate = [%g %g %g], heading = %g",
                 this->accelModel.getCurrentError().x, this->accelModel.getCurrentError().y, this->accelModel.getCurrentError().z,
                 this->rateModel.getCurrentError().x, this->rateModel.getCurrentError().y, this->rateModel.getCurrentError().z,
                 this->headingModel.getCurrentError());

  // apply offset error to orientation (pseudo AHRS)
  double normalization_constant = (this->gravity_body + this->accelModel.getCurrentError()).GetLength() * this->gravity_body.GetLength();
  double cos_alpha = (this->gravity_body + this->accelModel.getCurrentError()).GetDotProd(this->gravity_body)/normalization_constant;
  Vector3 normal_vector(this->gravity_body.GetCrossProd(this->accelModel.getCurrentError()));
  normal_vector *= sqrt((1 - cos_alpha)/2)/normalization_constant;
  Quatern attitudeError(sqrt((1 + cos_alpha)/2), normal_vector.x, normal_vector.y, normal_vector.z);
  Quatern headingError(cos(this->headingModel.getCurrentError()/2),0,0,sin(this->headingModel.getCurrentError()/2));
  rot = attitudeError * rot * headingError;

  // copy data into pose message
  this->imuMsg.header.frame_id = this->bodyName;
  this->imuMsg.header.stamp.sec = cur_time.sec;
  this->imuMsg.header.stamp.nsec = cur_time.nsec;

  // orientation quaternion
  this->imuMsg.orientation.x = rot.x;
  this->imuMsg.orientation.y = rot.y;
  this->imuMsg.orientation.z = rot.z;
  this->imuMsg.orientation.w = rot.u;

  // pass angular rates
  this->imuMsg.angular_velocity.x    = this->rate.x;
  this->imuMsg.angular_velocity.y    = this->rate.y;
  this->imuMsg.angular_velocity.z    = this->rate.z;

  // pass accelerations
  this->imuMsg.linear_acceleration.x    = this->accel.x;
  this->imuMsg.linear_acceleration.y    = this->accel.y;
  this->imuMsg.linear_acceleration.z    = this->accel.z;

  // fill in covariance matrix
  this->imuMsg.orientation_covariance[0] = this->headingModel.gaussian_noise*this->headingModel.gaussian_noise;
  if (gravity_length > 0.0) {
    this->imuMsg.orientation_covariance[4] = this->accelModel.gaussian_noise.y*this->accelModel.gaussian_noise.y/(gravity_length*gravity_length);
    this->imuMsg.orientation_covariance[8] = this->accelModel.gaussian_noise.x*this->accelModel.gaussian_noise.x/(gravity_length*gravity_length);
  } else {
    this->imuMsg.orientation_covariance[4] = -1;
    this->imuMsg.orientation_covariance[8] = -1;
  }

  // publish to ros
  this->pub_.publish(this->imuMsg);

  // debug output
#ifdef DEBUG_OUTPUT
  if (debugPublisher) {
    geometry_msgs::PoseStamped debugPose;
    debugPose.header = this->imuMsg.header;
    debugPose.header.frame_id = "/map";
    debugPose.pose.orientation.w = this->imuMsg.orientation.w;
    debugPose.pose.orientation.x = this->imuMsg.orientation.x;
    debugPose.pose.orientation.y = this->imuMsg.orientation.y;
    debugPose.pose.orientation.z = this->imuMsg.orientation.z;
    Pose3d pose = this->myBody->GetWorldPose();
    debugPose.pose.position.x = pose.pos.x;
    debugPose.pose.position.y = pose.pos.y;
    debugPose.pose.position.z = pose.pos.z;
    debugPublisher.publish(debugPose);
  }
#endif // DEBUG_OUTPUT

  this->lock.unlock();

  // save last time stamp
  this->last_time = cur_time;
}

////////////////////////////////////////////////////////////////////////////////
// Finalize the controller
void GazeboRosIMU::FiniChild()
{
  this->rosnode_->shutdown();
  this->callback_queue_thread_.join();
}

#ifdef USE_CBQ
void GazeboRosIMU::CallbackQueueThread()
{
  static const double timeout = 0.01;

  while (this->rosnode_->ok())
  {
    this->callback_queue_.callAvailable(ros::WallDuration(timeout));
  }
}
#endif

