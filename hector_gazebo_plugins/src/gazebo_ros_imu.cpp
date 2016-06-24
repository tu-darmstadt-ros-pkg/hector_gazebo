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

#include <hector_gazebo_plugins/gazebo_ros_imu.h>
#include <gazebo/common/Events.hh>
#include <gazebo/physics/physics.hh>

#include <ros/console.h>

namespace gazebo
{

// #define DEBUG_OUTPUT
#ifdef DEBUG_OUTPUT
  #include <geometry_msgs/PoseStamped.h>
  static ros::Publisher debugPublisher;
#endif // DEBUG_OUTPUT

////////////////////////////////////////////////////////////////////////////////
// Constructor
GazeboRosIMU::GazeboRosIMU()
{
}

////////////////////////////////////////////////////////////////////////////////
// Destructor
GazeboRosIMU::~GazeboRosIMU()
{
  updateTimer.Disconnect(updateConnection);

  dynamic_reconfigure_server_accel_.reset();
  dynamic_reconfigure_server_rate_.reset();
  dynamic_reconfigure_server_yaw_.reset();

  node_handle_->shutdown();
#ifdef USE_CBQ
  callback_queue_thread_.join();
#endif
  delete node_handle_;
}

////////////////////////////////////////////////////////////////////////////////
// Load the controller
void GazeboRosIMU::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
{
  // Get the world name.
  world = _model->GetWorld();

  // load parameters
  if (_sdf->HasElement("robotNamespace"))
    namespace_ = _sdf->GetElement("robotNamespace")->GetValue()->GetAsString();
  else
    namespace_.clear();

  if (_sdf->HasElement("bodyName"))
  {
    link_name_ = _sdf->GetElement("bodyName")->GetValue()->GetAsString();
    link = _model->GetLink(link_name_);
  }
  else
  {
    link = _model->GetLink();
    link_name_ = link->GetName();
  }

  // assert that the body by link_name_ exists
  if (!link)
  {
    ROS_FATAL("GazeboRosIMU plugin error: bodyName: %s does not exist\n", link_name_.c_str());
    return;
  }

  // default parameters
  frame_id_ = link_name_;
  topic_ = "imu";

  if (_sdf->HasElement("frameId"))
    frame_id_ = _sdf->GetElement("frameId")->GetValue()->GetAsString();

  if (_sdf->HasElement("topicName"))
    topic_ = _sdf->GetElement("topicName")->GetValue()->GetAsString();

  if (_sdf->HasElement("biasTopicName"))
    bias_topic_ = _sdf->GetElement("biasTopicName")->GetValue()->GetAsString();
  else
    bias_topic_ = (!topic_.empty() ? topic_ + "/bias" : "");

  if (_sdf->HasElement("serviceName"))
    serviceName = _sdf->GetElement("serviceName")->GetValue()->GetAsString();
  else
    serviceName = topic_ + "/calibrate";

  accelModel.Load(_sdf, "accel");
  rateModel.Load(_sdf, "rate");
  yawModel.Load(_sdf, "yaw");

  // also use old configuration variables from gazebo_ros_imu
  if (_sdf->HasElement("gaussianNoise")) {
    double gaussianNoise;
    if (_sdf->GetElement("gaussianNoise")->GetValue()->Get(gaussianNoise) && gaussianNoise != 0.0) {
      accelModel.gaussian_noise = gaussianNoise;
      rateModel.gaussian_noise  = gaussianNoise;
    }
  }

  if (_sdf->HasElement("xyzOffset")) {
    this->offset_.pos = _sdf->Get<math::Vector3>("xyzOffset");
  } else {
    ROS_INFO("imu plugin missing <xyzOffset>, defaults to 0s");
    this->offset_.pos = math::Vector3(0, 0, 0);
  }

  if (_sdf->HasElement("rpyOffset")) {
    this->offset_.rot = _sdf->Get<math::Vector3>("rpyOffset");
  } else {
    ROS_INFO("imu plugin missing <rpyOffset>, defaults to 0s");
    this->offset_.rot = math::Vector3(0, 0, 0);
  }

  // fill in constant covariance matrix
  imuMsg.angular_velocity_covariance[0] = rateModel.gaussian_noise.x*rateModel.gaussian_noise.x;
  imuMsg.angular_velocity_covariance[4] = rateModel.gaussian_noise.y*rateModel.gaussian_noise.y;
  imuMsg.angular_velocity_covariance[8] = rateModel.gaussian_noise.z*rateModel.gaussian_noise.z;
  imuMsg.linear_acceleration_covariance[0] = accelModel.gaussian_noise.x*accelModel.gaussian_noise.x;
  imuMsg.linear_acceleration_covariance[4] = accelModel.gaussian_noise.y*accelModel.gaussian_noise.y;
  imuMsg.linear_acceleration_covariance[8] = accelModel.gaussian_noise.z*accelModel.gaussian_noise.z;

  // Make sure the ROS node for Gazebo has already been initialized
  if (!ros::isInitialized())
  {
    ROS_FATAL_STREAM("A ROS node for Gazebo has not been initialized, unable to load plugin. "
      << "Load the Gazebo system plugin 'libgazebo_ros_api_plugin.so' in the gazebo_ros package.");
    return;
  }

  node_handle_ = new ros::NodeHandle(namespace_);

  // if topic name specified as empty, do not publish (then what is this plugin good for?)
  if (!topic_.empty())
    pub_ = node_handle_->advertise<sensor_msgs::Imu>(topic_, 10);
  if (!bias_topic_.empty())
    bias_pub_ = node_handle_->advertise<sensor_msgs::Imu>(bias_topic_, 10);

#ifdef DEBUG_OUTPUT
  debugPublisher = rosnode_->advertise<geometry_msgs::PoseStamped>(topic_ + "/pose", 10);
#endif // DEBUG_OUTPUT

  // advertise services for calibration and bias setting
  if (!serviceName.empty())
    srv_ = node_handle_->advertiseService(serviceName, &GazeboRosIMU::ServiceCallback, this);

  accelBiasService = node_handle_->advertiseService(topic_ + "/set_accel_bias", &GazeboRosIMU::SetAccelBiasCallback, this);
  rateBiasService  = node_handle_->advertiseService(topic_ + "/set_rate_bias", &GazeboRosIMU::SetRateBiasCallback, this);

  // setup dynamic_reconfigure servers
  if (!topic_.empty()) {
    dynamic_reconfigure_server_accel_.reset(new dynamic_reconfigure::Server<SensorModelConfig>(ros::NodeHandle(*node_handle_, topic_ + "/accel")));
    dynamic_reconfigure_server_rate_.reset(new dynamic_reconfigure::Server<SensorModelConfig>(ros::NodeHandle(*node_handle_, topic_ + "/rate")));
    dynamic_reconfigure_server_yaw_.reset(new dynamic_reconfigure::Server<SensorModelConfig>(ros::NodeHandle(*node_handle_, topic_ + "/yaw")));
    dynamic_reconfigure_server_accel_->setCallback(boost::bind(&SensorModel3::dynamicReconfigureCallback, &accelModel, _1, _2));
    dynamic_reconfigure_server_rate_->setCallback(boost::bind(&SensorModel3::dynamicReconfigureCallback, &rateModel, _1, _2));
    dynamic_reconfigure_server_yaw_->setCallback(boost::bind(&SensorModel::dynamicReconfigureCallback, &yawModel, _1, _2));
  }

#ifdef USE_CBQ
  // start custom queue for imu
  callback_queue_thread_ = boost::thread( boost::bind( &GazeboRosIMU::CallbackQueueThread,this ) );
#endif

  Reset();

  // connect Update function
  updateTimer.Load(world, _sdf);
  updateConnection = updateTimer.Connect(boost::bind(&GazeboRosIMU::Update, this));
}

void GazeboRosIMU::Reset()
{
  updateTimer.Reset();

  orientation = math::Quaternion();
  velocity = 0.0;
  accel = 0.0;

  accelModel.reset();
  rateModel.reset();
  yawModel.reset();
}

////////////////////////////////////////////////////////////////////////////////
// returns true always, imu is always calibrated in sim
bool GazeboRosIMU::ServiceCallback(std_srvs::Empty::Request &req,
                                        std_srvs::Empty::Response &res)
{
  boost::mutex::scoped_lock scoped_lock(lock);
  rateModel.reset(math::Vector3(0.0, 0.0, 0.0));
  return true;
}

bool GazeboRosIMU::SetAccelBiasCallback(hector_gazebo_plugins::SetBias::Request &req, hector_gazebo_plugins::SetBias::Response &res)
{
  boost::mutex::scoped_lock scoped_lock(lock);
  accelModel.reset(math::Vector3(req.bias.x, req.bias.y, req.bias.z));
  return true;
}

bool GazeboRosIMU::SetRateBiasCallback(hector_gazebo_plugins::SetBias::Request &req, hector_gazebo_plugins::SetBias::Response &res)
{
  boost::mutex::scoped_lock scoped_lock(lock);
  rateModel.reset(math::Vector3(req.bias.x, req.bias.y, req.bias.z));
  return true;
}

////////////////////////////////////////////////////////////////////////////////
// Update the controller
void GazeboRosIMU::Update()
{
  // Get Time Difference dt
  common::Time cur_time = world->GetSimTime();
  double dt = updateTimer.getTimeSinceLastUpdate().Double();
  boost::mutex::scoped_lock scoped_lock(lock);

  // Get Pose/Orientation
  math::Pose pose = link->GetWorldPose();
  // math::Vector3 pos = pose.pos + this->offset_.pos;
  math::Quaternion rot = this->offset_.rot * pose.rot;
  rot.Normalize();

  // get Gravity
  gravity = world->GetPhysicsEngine()->GetGravity();
  double gravity_length = gravity.GetLength();
  ROS_DEBUG_NAMED("gazebo_ros_imu", "gravity_world = [%g %g %g]", gravity.x, gravity.y, gravity.z);

  // get Acceleration and Angular Rates
  // the result of GetRelativeLinearAccel() seems to be unreliable (sum of forces added during the current simulation step)?
  //accel = myBody->GetRelativeLinearAccel(); // get acceleration in body frame
  math::Vector3 temp = link->GetWorldLinearVel(); // get velocity in world frame
  if (dt > 0.0) accel = rot.RotateVectorReverse((temp - velocity) / dt - gravity);
  velocity = temp;

  // calculate angular velocity from delta quaternion
  // note: link->GetRelativeAngularVel() sometimes return nan?
  // rate  = link->GetRelativeAngularVel(); // get angular rate in body frame
  math::Quaternion delta = this->orientation.GetInverse() * rot;
  this->orientation = rot;
  if (dt > 0.0) {
    rate = this->offset_.rot.GetInverse()
           * (2.0 * acos(std::max(std::min(delta.w, 1.0), -1.0)) * math::Vector3(delta.x, delta.y, delta.z).Normalize() / dt);
  }

  // update sensor models
  accel = accelModel(accel, dt);
  rate  = rateModel(rate, dt);
  yawModel.update(dt);
  ROS_DEBUG_NAMED("gazebo_ros_imu", "Current bias errors: accel = [%g %g %g], rate = [%g %g %g], yaw = %g",
                 accelModel.getCurrentBias().x, accelModel.getCurrentBias().y, accelModel.getCurrentBias().z,
                 rateModel.getCurrentBias().x, rateModel.getCurrentBias().y, rateModel.getCurrentBias().z,
                 yawModel.getCurrentBias());
  ROS_DEBUG_NAMED("gazebo_ros_imu", "Scale errors: accel = [%g %g %g], rate = [%g %g %g], yaw = %g",
                 accelModel.getScaleError().x, accelModel.getScaleError().y, accelModel.getScaleError().z,
                 rateModel.getScaleError().x, rateModel.getScaleError().y, rateModel.getScaleError().z,
                 yawModel.getScaleError());


  // apply accelerometer and yaw drift error to orientation (pseudo AHRS)
  math::Vector3 accelDrift = pose.rot.RotateVector(accelModel.getCurrentBias());
  double yawError = yawModel.getCurrentBias();
  math::Quaternion orientationError(
    math::Quaternion(cos(yawError/2), 0.0, 0.0, sin(yawError/2)) *                                         // yaw error
    math::Quaternion(1.0, 0.5 * accelDrift.y / gravity_length, 0.5 * -accelDrift.x / gravity_length, 0.0)  // roll and pitch error
  );
  orientationError.Normalize();
  rot = orientationError * rot;

  // copy data into pose message
  imuMsg.header.frame_id = frame_id_;
  imuMsg.header.stamp.sec = cur_time.sec;
  imuMsg.header.stamp.nsec = cur_time.nsec;

  // orientation quaternion
  imuMsg.orientation.x = rot.x;
  imuMsg.orientation.y = rot.y;
  imuMsg.orientation.z = rot.z;
  imuMsg.orientation.w = rot.w;

  // pass angular rates
  imuMsg.angular_velocity.x    = rate.x;
  imuMsg.angular_velocity.y    = rate.y;
  imuMsg.angular_velocity.z    = rate.z;

  // pass accelerations
  imuMsg.linear_acceleration.x    = accel.x;
  imuMsg.linear_acceleration.y    = accel.y;
  imuMsg.linear_acceleration.z    = accel.z;

  // fill in covariance matrix
  imuMsg.orientation_covariance[8] = yawModel.gaussian_noise*yawModel.gaussian_noise;
  if (gravity_length > 0.0) {
    imuMsg.orientation_covariance[0] = accelModel.gaussian_noise.x*accelModel.gaussian_noise.x/(gravity_length*gravity_length);
    imuMsg.orientation_covariance[4] = accelModel.gaussian_noise.y*accelModel.gaussian_noise.y/(gravity_length*gravity_length);
  } else {
    imuMsg.orientation_covariance[0] = -1;
    imuMsg.orientation_covariance[4] = -1;
  }

  // publish to ros
  pub_.publish(imuMsg);
  ROS_DEBUG_NAMED("gazebo_ros_imu", "Publishing IMU data at t = %f", cur_time.Double());

  // publish bias
  if (bias_pub_) {
    biasMsg.header = imuMsg.header;
    biasMsg.orientation.x = orientationError.x;
    biasMsg.orientation.y = orientationError.y;
    biasMsg.orientation.z = orientationError.z;
    biasMsg.orientation.w = orientationError.w;
    biasMsg.angular_velocity.x = rateModel.getCurrentBias().x;
    biasMsg.angular_velocity.y = rateModel.getCurrentBias().y;
    biasMsg.angular_velocity.z = rateModel.getCurrentBias().z;
    biasMsg.linear_acceleration.x = accelModel.getCurrentBias().x;
    biasMsg.linear_acceleration.y = accelModel.getCurrentBias().y;
    biasMsg.linear_acceleration.z = accelModel.getCurrentBias().z;
    bias_pub_.publish(biasMsg);
  }

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
    math::Pose pose = link->GetWorldPose();
    debugPose.pose.position.x = pose.pos.x;
    debugPose.pose.position.y = pose.pos.y;
    debugPose.pose.position.z = pose.pos.z;
    debugPublisher.publish(debugPose);
  }
#endif // DEBUG_OUTPUT
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

// Register this plugin with the simulator
GZ_REGISTER_MODEL_PLUGIN(GazeboRosIMU)

} // namespace gazebo
