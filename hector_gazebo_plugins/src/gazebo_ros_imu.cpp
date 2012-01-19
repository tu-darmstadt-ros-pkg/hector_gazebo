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
{
   this->myParent = dynamic_cast<Model*>(this->parent);

   if (!this->myParent)
      gzthrow("GazeboRosIMU controller requires a Model as its parent");

  Param::Begin(&this->parameters);
  this->robotNamespaceP = new ParamT<std::string>("robotNamespace", "/", 0);
  this->bodyNameP = new ParamT<std::string>("bodyName", "", 0);
  this->topicNameP = new ParamT<std::string>("topicName", "", 1);
  this->accelOffsetP = new ParamT<Vector3>("accelOffset", Vector3(0,0,0), 0);
  this->accelDriftP = new ParamT<Vector3>("accelDrift", Vector3(0,0,0), 0);
  this->accelDriftFrequencyP = new ParamT<Vector3>("accelDriftFrequency", Vector3(0.0003,0.0003,0.0003), 0);
  this->accelGaussianNoiseP = new ParamT<Vector3>("accelGaussianNoise", Vector3(0.0,0.0,0.0), 0);
  this->rateOffsetP = new ParamT<Vector3>("rateOffset", Vector3(0,0,0), 0);
  this->rateDriftP = new ParamT<Vector3>("rateDriftP", Vector3(0,0,0), 0);
  this->rateDriftFrequencyP = new ParamT<Vector3>("rateDriftFrequency", Vector3(0.0003,0.0003,0.0003), 0);
  this->rateGaussianNoiseP = new ParamT<Vector3>("rateGaussianNoise", Vector3(0.0,0.0,0.0), 0);
  this->headingOffsetP = new ParamT<double>("headingOffset", 0.0, 0);
  this->headingDriftP = new ParamT<double>("headingDrift", 0.0, 0);
  this->headingDriftFrequencyP = new ParamT<double>("headingDriftFrequency", 0.0003, 0);
  this->headingGaussianNoiseP = new ParamT<double>("headingGaussianNoise", 0.0, 0);
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
  delete this->accelOffsetP;
  delete this->accelDriftP;
  delete this->accelDriftFrequencyP;
  delete this->accelGaussianNoiseP;
  delete this->rateOffsetP;
  delete this->rateDriftP;
  delete this->rateDriftFrequencyP;
  delete this->rateGaussianNoiseP;
  delete this->headingOffsetP;
  delete this->headingDriftP;
  delete this->headingDriftFrequencyP;
  delete this->headingGaussianNoiseP;
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

  this->accelOffsetP->Load(node);
  this->accelOffset = this->accelOffsetP->GetValue();
  this->accelDriftP->Load(node);
  this->accelDrift = this->accelDriftP->GetValue();
  this->accelDriftFrequencyP->Load(node);
  this->accelDriftFrequency = this->accelDriftFrequencyP->GetValue();
  this->accelGaussianNoiseP->Load(node);
  this->accelGaussianNoise = this->accelGaussianNoiseP->GetValue();
  this->rateOffsetP->Load(node);
  this->rateOffset = this->rateOffsetP->GetValue();
  this->rateDriftP->Load(node);
  this->rateDrift = this->rateDriftP->GetValue();
  this->rateDriftFrequencyP->Load(node);
  this->rateDriftFrequency = this->rateDriftFrequencyP->GetValue();
  this->rateGaussianNoiseP->Load(node);
  this->rateGaussianNoise = this->rateGaussianNoiseP->GetValue();
  this->headingOffsetP->Load(node);
  this->headingOffset = this->headingOffsetP->GetValue();
  this->headingDriftP->Load(node);
  this->headingDrift = this->headingDriftP->GetValue();
  this->headingDriftFrequencyP->Load(node);
  this->headingDriftFrequency = this->headingDriftFrequencyP->GetValue();
  this->headingGaussianNoiseP->Load(node);
  this->headingGaussianNoise = this->headingGaussianNoiseP->GetValue();

  // also use old configuration variables from gazebo_ros_imu
  this->rpyOffsetsP->Load(node);
  Vector3 rpyOffsets = this->rpyOffsetsP->GetValue();
  if (this->accelOffset.y == 0.0 && rpyOffsets.x != 0.0) this->accelOffset.y = -rpyOffsets.x * 9.8065;
  if (this->accelOffset.x == 0.0 && rpyOffsets.y != 0.0) this->accelOffset.x =  rpyOffsets.y * 9.8065;
  if (this->headingOffset == 0.0 && rpyOffsets.z != 0.0) this->headingOffset =  rpyOffsets.z;
  this->gaussianNoiseP->Load(node);
  double gaussianNoise = this->gaussianNoiseP->GetValue();
  if (gaussianNoise != 0.0) {
    this->accelGaussianNoise = gaussianNoise;
    this->rateGaussianNoise  = gaussianNoise;
  }

  this->accelCurrentDrift = Vector3(0.0, 0.0, 0.0);
  this->rateCurrentDrift = Vector3(0.0, 0.0, 0.0);
  this->headingCurrentDrift = 0.0;

  // fill in constant covariance matrix
  this->imuMsg.angular_velocity_covariance[0] = this->rateGaussianNoise.z*this->rateGaussianNoise.z;
  this->imuMsg.angular_velocity_covariance[4] = this->rateGaussianNoise.y*this->rateGaussianNoise.y;
  this->imuMsg.angular_velocity_covariance[8] = this->rateGaussianNoise.x*this->rateGaussianNoise.x;
  this->imuMsg.linear_acceleration_covariance[0] = this->accelGaussianNoise.z*this->accelGaussianNoise.z;
  this->imuMsg.linear_acceleration_covariance[4] = this->accelGaussianNoise.y*this->accelGaussianNoise.y;
  this->imuMsg.linear_acceleration_covariance[8] = this->accelGaussianNoise.x*this->accelGaussianNoise.x;

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
  this->rateOffset = Vector3(0.0, 0.0, 0.0);
  this->rateCurrentDrift = Vector3(0.0, 0.0, 0.0);
  this->lock.unlock();
  return true;
}

bool GazeboRosIMU::SetAccelBiasCallback(hector_gazebo_plugins::SetBias::Request &req, hector_gazebo_plugins::SetBias::Response &res)
{
  this->lock.lock();
  this->accelOffset = Vector3(req.bias.x, req.bias.y, req.bias.z);
  this->accelCurrentDrift = Vector3(0.0, 0.0, 0.0);
  this->lock.unlock();
  return true;
}

bool GazeboRosIMU::SetRateBiasCallback(hector_gazebo_plugins::SetBias::Request &req, hector_gazebo_plugins::SetBias::Response &res)
{
  this->lock.lock();
  this->rateOffset = Vector3(req.bias.x, req.bias.y, req.bias.z);
  this->rateCurrentDrift = Vector3(0.0, 0.0, 0.0);
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

  // get Acceleration, Rates and Gravity
  this->accel = this->myBody->GetRelativeLinearAccel(); // get acceleration in body frame
  this->rate  = this->myBody->GetRelativeAngularVel(); // get angular rate in body frame
  this->gravity       = World::Instance()->GetPhysicsEngine()->GetGravity();
  this->gravity_body  = rot.RotateVector(this->gravity);
  double temp = 1.0/std::max(this->gravity.GetLength(), 1.0);
  double gravity_correction = (this->gravity.z > 0.0) ? -temp : temp;

  // add gravity vector
  ROS_DEBUG_NAMED("hector_gazebo_ros_imu", "gravity_world = [%g %g %g]", this->gravity.x, this->gravity.y, this->gravity.z);
  this->accel = this->accel - this->gravity_body;

  this->lock.lock();

  // calculate offsets and drift
  this->accelCurrentError.x = updateCurrentError(this->accelCurrentDrift.x, dt, this->accelDriftFrequency.x, this->accelDrift.x, this->accelOffset.x, this->accelGaussianNoise.x);
  this->accelCurrentError.y = updateCurrentError(this->accelCurrentDrift.y, dt, this->accelDriftFrequency.y, this->accelDrift.y, this->accelOffset.y, this->accelGaussianNoise.y);
  this->accelCurrentError.z = updateCurrentError(this->accelCurrentDrift.z, dt, this->accelDriftFrequency.z, this->accelDrift.z, this->accelOffset.z, this->accelGaussianNoise.z);
  this->rateCurrentError.x  = updateCurrentError(this->rateCurrentDrift.x,  dt, this->rateDriftFrequency.x,  this->rateDrift.x,  this->rateOffset.x,  this->rateGaussianNoise.x);
  this->rateCurrentError.y  = updateCurrentError(this->rateCurrentDrift.y,  dt, this->rateDriftFrequency.y,  this->rateDrift.y,  this->rateOffset.y,  this->rateGaussianNoise.y);
  this->rateCurrentError.z  = updateCurrentError(this->rateCurrentDrift.z,  dt, this->rateDriftFrequency.z,  this->rateDrift.z,  this->rateOffset.z,  this->rateGaussianNoise.z);
  this->headingCurrentError = updateCurrentError(this->headingCurrentDrift, dt, this->headingDriftFrequency, this->headingDrift, this->headingOffset, this->headingGaussianNoise);
  ROS_DEBUG_NAMED("hector_gazebo_ros_imu", "Current errors: accel = [%g %g %g], rate = [%g %g %g], heading = %g, gravity_correction = %g",
                 this->accelCurrentError.x, this->accelCurrentError.y, this->accelCurrentError.z,
                 this->rateCurrentError.x, this->rateCurrentError.y, this->rateCurrentError.z,
                 this->headingCurrentError, 1.0/gravity_correction);

  // apply offset error
  this->accel = this->accel + this->accelCurrentError;
  this->rate  = this->rate  + this->rateCurrentError;
  double normalization_constant = (this->gravity_body + this->accelCurrentError).GetLength() * this->gravity_body.GetLength();
  double cos_alpha = (this->gravity_body + this->accelCurrentError).GetDotProd(this->gravity_body)/normalization_constant;
  Vector3 normal_vector(this->gravity_body.GetCrossProd(this->accelCurrentError));
  normal_vector *= sqrt((1 - cos_alpha)/2)/normalization_constant;
  Quatern attitudeError(sqrt((1 + cos_alpha)/2), normal_vector.x, normal_vector.y, normal_vector.z);
  Quatern headingError(cos(this->headingCurrentError/2),0,0,sin(this->headingCurrentError/2));
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
  this->imuMsg.orientation_covariance[0] = this->headingGaussianNoise*this->headingGaussianNoise;
  this->imuMsg.orientation_covariance[4] = this->accelGaussianNoise.y*this->accelGaussianNoise.y*(gravity_correction*gravity_correction);
  this->imuMsg.orientation_covariance[8] = this->accelGaussianNoise.x*this->accelGaussianNoise.x*(gravity_correction*gravity_correction);

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

//////////////////////////////////////////////////////////////////////////////
// Utility for adding noise
double GazeboRosIMU::GaussianKernel(double mu, double sigma)
{
  // using Box-Muller transform to generate two independent standard normally disbributed normal variables
  // see wikipedia
  double U = (double)rand()/(double)RAND_MAX; // normalized uniform random variable
  double V = (double)rand()/(double)RAND_MAX; // normalized uniform random variable
  double X = sqrt(-2.0 * ::log(U)) * cos( 2.0*M_PI * V);
  //double Y = sqrt(-2.0 * ::log(U)) * sin( 2.0*M_PI * V); // the other indep. normal variable
  // we'll just use X
  // scale to our mu and sigma
  X = sigma * X + mu;
  return X;
}

//////////////////////////////////////////////////////////////////////////////
// Drift model
double GazeboRosIMU::updateCurrentError(double &currentDrift, double dt, double driftFrequency,  double drift,  double offset, double noise)
{
  currentDrift += dt * (-currentDrift * driftFrequency + this->GaussianKernel(0, sqrt(2*driftFrequency)*drift));
  return offset + currentDrift +  this->GaussianKernel(0, noise);
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

