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
   , accelModel(parameters, "accel")
   , rateModel(parameters, "rate")
   , headingModel(parameters, "heading")
{
   myParent = dynamic_cast<Model*>(parent);

   if (!myParent)
      gzthrow("GazeboRosIMU controller requires a Model as its parent");

  Param::Begin(&parameters);
  robotNamespaceP = new ParamT<std::string>("robotNamespace", "/", 0);
  bodyNameP = new ParamT<std::string>("bodyName", "", 0);
  topicNameP = new ParamT<std::string>("topicName", "", 1);
  rpyOffsetsP    = new ParamT<Vector3>("rpyOffsets", Vector3(0,0,0),0);
  gaussianNoiseP = new ParamT<double>("gaussianNoise",0.0,0);
  serviceNameP = new ParamT<std::string>("serviceName", "imu/calibrate", 0);
  Param::End();
}

////////////////////////////////////////////////////////////////////////////////
// Destructor
GazeboRosIMU::~GazeboRosIMU()
{
  delete robotNamespaceP;
  delete bodyNameP;
  delete topicNameP;
  delete rpyOffsetsP;
  delete gaussianNoiseP;
  delete serviceNameP;
  delete rosnode_;
}

////////////////////////////////////////////////////////////////////////////////
// Load the controller
void GazeboRosIMU::LoadChild(XMLConfigNode *node)
{
  robotNamespaceP->Load(node);
  robotNamespace = robotNamespaceP->GetValue();
  if (!ros::isInitialized())
  {
    int argc = 0;
    char** argv = NULL;
    ros::init(argc,argv, "gazebo", ros::init_options::NoSigintHandler|ros::init_options::AnonymousName);
  }

  rosnode_ = new ros::NodeHandle(robotNamespace);
#ifdef USE_CBQ
  rosnode_->setCallbackQueue(&callback_queue_);
#endif

  bodyNameP->Load(node);
  bodyName = bodyNameP->GetValue();

  // assert that the body by bodyName exists
  if (dynamic_cast<Body*>(myParent->GetBody(bodyName)) == NULL)
    ROS_FATAL("gazebo_ros_imu plugin error: bodyName: %s does not exist\n",bodyName.c_str());

  myBody = dynamic_cast<Body*>(myParent->GetBody(bodyName));

  topicNameP->Load(node);
  topicName = topicNameP->GetValue();

  accelModel.Load(node);
  rateModel.Load(node);
  headingModel.Load(node);

  // also use old configuration variables from gazebo_ros_imu
  rpyOffsetsP->Load(node);
  Vector3 rpyOffsets = rpyOffsetsP->GetValue();
  if (accelModel.offset.y == 0.0 && rpyOffsets.x != 0.0) accelModel.offset.y = -rpyOffsets.x * 9.8065;
  if (accelModel.offset.x == 0.0 && rpyOffsets.y != 0.0) accelModel.offset.x =  rpyOffsets.y * 9.8065;
  if (headingModel.offset == 0.0 && rpyOffsets.z != 0.0) headingModel.offset =  rpyOffsets.z;
  gaussianNoiseP->Load(node);
  double gaussianNoise = gaussianNoiseP->GetValue();
  if (gaussianNoise != 0.0) {
    accelModel.gaussian_noise = gaussianNoise;
    rateModel.gaussian_noise  = gaussianNoise;
  }

  // fill in constant covariance matrix
  imuMsg.angular_velocity_covariance[0] = rateModel.gaussian_noise.z*rateModel.gaussian_noise.z;
  imuMsg.angular_velocity_covariance[4] = rateModel.gaussian_noise.y*rateModel.gaussian_noise.y;
  imuMsg.angular_velocity_covariance[8] = rateModel.gaussian_noise.x*rateModel.gaussian_noise.x;
  imuMsg.linear_acceleration_covariance[0] = accelModel.gaussian_noise.z*accelModel.gaussian_noise.z;
  imuMsg.linear_acceleration_covariance[4] = accelModel.gaussian_noise.y*accelModel.gaussian_noise.y;
  imuMsg.linear_acceleration_covariance[8] = accelModel.gaussian_noise.x*accelModel.gaussian_noise.x;

  if (topicName != "")
  {
    pub_ = rosnode_->advertise<sensor_msgs::Imu>(topicName,10);
  }

#ifdef DEBUG_OUTPUT
  debugPublisher = rosnode_->advertise<geometry_msgs::PoseStamped>(topicName+"/pose", 10);
#endif // DEBUG_OUTPUT

  // add service call version for position change
  serviceNameP->Load(node);
  serviceName = serviceNameP->GetValue();
  // advertise services on the custom queue
  srv_ = rosnode_->advertiseService(serviceName, &GazeboRosIMU::ServiceCallback, this);
  accelBiasService = rosnode_->advertiseService(topicName+"/set_accel_bias", &GazeboRosIMU::SetAccelBiasCallback, this);
  rateBiasService  = rosnode_->advertiseService(topicName+"/set_rate_bias", &GazeboRosIMU::SetRateBiasCallback, this);
}

////////////////////////////////////////////////////////////////////////////////
// returns true always, imu is always calibrated in sim
bool GazeboRosIMU::ServiceCallback(std_srvs::Empty::Request &req,
                                        std_srvs::Empty::Response &res)
{
  lock.lock();
  rateModel.reset();
  lock.unlock();
  return true;
}

bool GazeboRosIMU::SetAccelBiasCallback(hector_gazebo_plugins::SetBias::Request &req, hector_gazebo_plugins::SetBias::Response &res)
{
  lock.lock();
  accelModel.reset(Vector3(req.bias.x, req.bias.y, req.bias.z));
  lock.unlock();
  return true;
}

bool GazeboRosIMU::SetRateBiasCallback(hector_gazebo_plugins::SetBias::Request &req, hector_gazebo_plugins::SetBias::Response &res)
{
  lock.lock();
  rateModel.reset(Vector3(req.bias.x, req.bias.y, req.bias.z));
  lock.unlock();
  return true;
}

////////////////////////////////////////////////////////////////////////////////
// Initialize the controller
void GazeboRosIMU::InitChild()
{
  orientation = Quatern();
  velocity = 0.0;
  accel = 0.0;

#ifdef USE_CBQ
  // start custom queue for imu
  callback_queue_thread_ = boost::thread( boost::bind( &GazeboRosIMU::CallbackQueueThread,this ) );
#endif
}

////////////////////////////////////////////////////////////////////////////////
// Update the controller
void GazeboRosIMU::UpdateChild()
{
  // Get Pose/Orientation
  Pose3d pose = myBody->GetWorldPose();

  // Get Time Difference dt
  gazebo::Time cur_time = Simulator::Instance()->GetSimTime();
  double dt = cur_time - lastUpdate;

  lock.lock();

  // get Acceleration and Angular Rates
  // the result of GetRelativeLinearAccel() seems to be unreliable (sum of forces added during the current simulation step)?
  //accel = myBody->GetRelativeLinearAccel(); // get acceleration in body frame
  Vector3 temp = myBody->GetRelativeLinearVel(); // get velocity in body frame
  accel = (temp - velocity) / dt;
  velocity = temp;

  // GetRelativeAngularVel() sometimes return nan?
  //rate  = myBody->GetRelativeAngularVel(); // get angular rate in body frame
  Quatern delta = pose.rot - orientation;
  orientation = pose.rot;
  rate.x = 2.0 * (-orientation.x * delta.u + orientation.u * delta.x + orientation.z * delta.y - orientation.y * delta.z) / dt;
  rate.y = 2.0 * (-orientation.y * delta.u - orientation.z * delta.x + orientation.u * delta.y + orientation.x * delta.z) / dt;
  rate.z = 2.0 * (-orientation.z * delta.u + orientation.y * delta.x - orientation.x * delta.y + orientation.u * delta.z) / dt;

  // get Gravity
  gravity       = World::Instance()->GetPhysicsEngine()->GetGravity();
  gravity_body  = orientation.RotateVectorReverse(gravity);
  double gravity_length = gravity.GetLength();
  ROS_DEBUG_NAMED("hector_gazebo_ros_imu", "gravity_world = [%g %g %g]", gravity.x, gravity.y, gravity.z);

  // add gravity vector to body acceleration
  accel = accel - gravity_body;

  // update sensor models
  accel = accel + accelModel.update(dt);
  rate  = rate  + rateModel.update(dt);
  headingModel.update(dt);
  ROS_DEBUG_NAMED("hector_gazebo_ros_imu", "Current errors: accel = [%g %g %g], rate = [%g %g %g], heading = %g",
                 accelModel.getCurrentError().x, accelModel.getCurrentError().y, accelModel.getCurrentError().z,
                 rateModel.getCurrentError().x, rateModel.getCurrentError().y, rateModel.getCurrentError().z,
                 headingModel.getCurrentError());

  // apply offset error to orientation (pseudo AHRS)
  double normalization_constant = (gravity_body + accelModel.getCurrentError()).GetLength() * gravity_body.GetLength();
  double cos_alpha = (gravity_body + accelModel.getCurrentError()).GetDotProd(gravity_body)/normalization_constant;
  Vector3 normal_vector(gravity_body.GetCrossProd(accelModel.getCurrentError()));
  normal_vector *= sqrt((1 - cos_alpha)/2)/normalization_constant;
  Quatern attitudeError(sqrt((1 + cos_alpha)/2), normal_vector.x, normal_vector.y, normal_vector.z);
  Quatern headingError(cos(headingModel.getCurrentError()/2),0,0,sin(headingModel.getCurrentError()/2));
  pose.rot = attitudeError * pose.rot * headingError;

  // copy data into pose message
  imuMsg.header.frame_id = bodyName;
  imuMsg.header.stamp.sec = cur_time.sec;
  imuMsg.header.stamp.nsec = cur_time.nsec;

  // orientation quaternion
  imuMsg.orientation.x = pose.rot.x;
  imuMsg.orientation.y = pose.rot.y;
  imuMsg.orientation.z = pose.rot.z;
  imuMsg.orientation.w = pose.rot.u;

  // pass angular rates
  imuMsg.angular_velocity.x    = rate.x;
  imuMsg.angular_velocity.y    = rate.y;
  imuMsg.angular_velocity.z    = rate.z;

  // pass accelerations
  imuMsg.linear_acceleration.x    = accel.x;
  imuMsg.linear_acceleration.y    = accel.y;
  imuMsg.linear_acceleration.z    = accel.z;

  // fill in covariance matrix
  imuMsg.orientation_covariance[0] = headingModel.gaussian_noise*headingModel.gaussian_noise;
  if (gravity_length > 0.0) {
    imuMsg.orientation_covariance[4] = accelModel.gaussian_noise.y*accelModel.gaussian_noise.y/(gravity_length*gravity_length);
    imuMsg.orientation_covariance[8] = accelModel.gaussian_noise.x*accelModel.gaussian_noise.x/(gravity_length*gravity_length);
  } else {
    imuMsg.orientation_covariance[4] = -1;
    imuMsg.orientation_covariance[8] = -1;
  }

  // publish to ros
  pub_.publish(imuMsg);

  // debug output
#ifdef DEBUG_OUTPUT
  if (debugPublisher) {
    geometry_msgs::PoseStamped debugPose;
    debugPose.header = imuMsg.header;
    debugPose.header.frame_id = "/map";
    debugPose.pose.orientation.w = imuMsg.orientation.w;
    debugPose.pose.orientation.x = imuMsg.orientation.x;
    debugPose.pose.orientation.y = imuMsg.orientation.y;
    debugPose.pose.orientation.z = imuMsg.orientation.z;
    Pose3d pose = myBody->GetWorldPose();
    debugPose.pose.position.x = pose.pos.x;
    debugPose.pose.position.y = pose.pos.y;
    debugPose.pose.position.z = pose.pos.z;
    debugPublisher.publish(debugPose);
  }
#endif // DEBUG_OUTPUT

  lock.unlock();
}

////////////////////////////////////////////////////////////////////////////////
// Finalize the controller
void GazeboRosIMU::FiniChild()
{
  rosnode_->shutdown();
  callback_queue_thread_.join();
}

#ifdef USE_CBQ
void GazeboRosIMU::CallbackQueueThread()
{
  static const double timeout = 0.01;

  while (rosnode_->ok())
  {
    callback_queue_.callAvailable(ros::WallDuration(timeout));
  }
}
#endif

