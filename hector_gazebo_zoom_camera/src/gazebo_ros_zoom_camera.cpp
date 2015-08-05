/*
 * Copyright 2013 Open Source Robotics Foundation
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
*/

/*
 @mainpage
   Desc: GazeboRosZoomCamera plugin for simulating cameras in Gazebo
   Author: John Hsu
   Date: 24 Sept 2008
*/

#include "hector_gazebo_zoom_camera/gazebo_ros_zoom_camera.h"

#include <string>

#include <gazebo/sensors/Sensor.hh>
#include <gazebo/sensors/CameraSensor.hh>
#include <gazebo/sensors/SensorTypes.hh>

namespace gazebo
{
// Register this plugin with the simulator
GZ_REGISTER_SENSOR_PLUGIN(GazeboRosZoomCamera)

////////////////////////////////////////////////////////////////////////////////
// Constructor
GazeboRosZoomCamera::GazeboRosZoomCamera()
{
}

////////////////////////////////////////////////////////////////////////////////
// Destructor
GazeboRosZoomCamera::~GazeboRosZoomCamera()
{
  rosnode_->shutdown();
  callback_queue_thread_.join();
  delete rosnode_;
  
  ROS_DEBUG_STREAM_NAMED("camera","Unloaded");
}

void GazeboRosZoomCamera::Load(sensors::SensorPtr _parent, sdf::ElementPtr _sdf)
{
  // Make sure the ROS node for Gazebo has already been initialized
  if (!ros::isInitialized())
  {
    ROS_FATAL_STREAM("A ROS node for Gazebo has not been initialized, unable to load plugin. "
      << "Load the Gazebo system plugin 'libgazebo_ros_api_plugin.so' in the gazebo_ros package)");
    return;
  }
  
  if (_sdf->HasElement("zoomFactorTopicName"))
    fov_topic_ = _sdf->GetElement("zoomFactorTopicName")->GetValue()->GetAsString();

  CameraPlugin::Load(_parent, _sdf);
  // copying from CameraPlugin into GazeboRosCameraUtils
  this->parentSensor_ = this->parentSensor;
  this->width_ = this->width;
  this->height_ = this->height;
  this->depth_ = this->depth;
  this->format_ = this->format;
  this->camera_ = this->camera;

  GazeboRosCameraUtils::Load(_parent, _sdf);

  default_fov_ = camera_->GetHFOV().Radian();
  
  rosnode_ = new ros::NodeHandle("");
  
  // ROS: Subscribe to the velocity command topic (usually "cmd_vel")
  ros::SubscribeOptions so =
      ros::SubscribeOptions::create<std_msgs::Float64>(fov_topic_, 1,
                                                          boost::bind(&GazeboRosZoomCamera::fovCallback, this, _1),
                                                          ros::VoidPtr(), &queue_);
  fov_sub_ = rosnode_->subscribe(so);
  
  callback_queue_thread_ = boost::thread(boost::bind(&GazeboRosZoomCamera::QueueThread, this));
}

////////////////////////////////////////////////////////////////////////////////
// Update the controller
void GazeboRosZoomCamera::OnNewFrame(const unsigned char *_image,
    unsigned int _width, unsigned int _height, unsigned int _depth,
    const std::string &_format)
{
  this->sensor_update_time_ = this->parentSensor_->GetLastUpdateTime();

  if (!this->parentSensor->IsActive())
  {
    if ((*this->image_connect_count_) > 0)
      // do this first so there's chance for sensor to run once after activated
      this->parentSensor->SetActive(true);
  }
  else
  {
    if ((*this->image_connect_count_) > 0)
    {
      common::Time cur_time = this->world_->GetSimTime();
      if (cur_time - this->last_update_time_ >= this->update_period_)
      {
        this->PutCameraData(_image);
        this->PublishCameraInfo();
        this->last_update_time_ = cur_time;
      }
    }
  }
}

// NEW: custom callback queue thread
void GazeboRosZoomCamera::QueueThread()
{
  static const double timeout = 0.05;

  while (rosnode_->ok())
  {
    //    std::cout << "CALLING STUFF\n";
    queue_.callAvailable(ros::WallDuration(timeout));
  }
}

void GazeboRosZoomCamera::fovCallback(const std_msgs::Float64::ConstPtr& msg)
{
  double new_fov = default_fov_ * msg->data;
  std::cout << "new_fov: " << new_fov << "\n";
  this->camera_->SetHFOV(new_fov);
}


}
