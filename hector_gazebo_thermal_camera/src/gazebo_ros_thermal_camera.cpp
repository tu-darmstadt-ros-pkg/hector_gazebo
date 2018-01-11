//=================================================================================================
// Copyright (c) 2012, Stefan Kohlbrecher, TU Darmstadt
// All rights reserved.

// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//     * Redistributions of source code must retain the above copyright
//       notice, this list of conditions and the following disclaimer.
//     * Redistributions in binary form must reproduce the above copyright
//       notice, this list of conditions and the following disclaimer in the
//       documentation and/or other materials provided with the distribution.
//     * Neither the name of the Simulation, Systems Optimization and Robotics
//       group, TU Darmstadt nor the names of its contributors may be used to
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
/**
 * Copy of the CameraSensor/DepthCameraSensor plugin with minor changes
 */

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

#include <hector_gazebo_thermal_camera/gazebo_ros_thermal_camera.h>

#include <gazebo/sensors/Sensor.hh>
#include <gazebo/sensors/SensorTypes.hh>

#include <sensor_msgs/image_encodings.h>

#include <gazebo/gazebo_config.h>

namespace gazebo
{

////////////////////////////////////////////////////////////////////////////////
// Constructor
template <class Base>
GazeboRosThermalCamera_<Base>::GazeboRosThermalCamera_()
{
}

////////////////////////////////////////////////////////////////////////////////
// Destructor
template <class Base>
GazeboRosThermalCamera_<Base>::~GazeboRosThermalCamera_()
{
}

template <class Base>
void GazeboRosThermalCamera_<Base>::Load(sensors::SensorPtr _parent, sdf::ElementPtr _sdf)
{
  Base::Load(_parent, _sdf);
  // copying from CameraPlugin into GazeboRosCameraUtils
  this->parentSensor_ = this->parentSensor;
  this->width_ = this->width;
  this->height_ = this->height;
  this->depth_ = this->depth;
  this->format_ = this->format;

  this->image_connect_count_ = boost::shared_ptr<int>(new int);
  *this->image_connect_count_ = 0;
  this->image_connect_count_lock_ = boost::shared_ptr<boost::mutex>(new boost::mutex);
  this->was_active_ = boost::shared_ptr<bool>(new bool);
  *this->was_active_ = false;

  LoadImpl(_parent, _sdf);
  GazeboRosCameraUtils::Load(_parent, _sdf);
}

////////////////////////////////////////////////////////////////////////////////
// Update the controller
template <class Base>
void GazeboRosThermalCamera_<Base>::OnNewFrame(const unsigned char *_image,
    unsigned int _width, unsigned int _height, unsigned int _depth,
    const std::string &_format)
{
  if (!this->initialized_ || this->height_ <=0 || this->width_ <=0)
    return;

#if (GAZEBO_MAJOR_VERSION > 6)
  this->sensor_update_time_ = this->parentSensor_->LastUpdateTime();
#else
  this->sensor_update_time_ = this->parentSensor_->GetLastUpdateTime();
#endif

  if (!this->parentSensor->IsActive())
  {
    if ((*this->image_connect_count_) > 0)
      // do this first so there's chance for sensor to run 1 frame after activate
      this->parentSensor->SetActive(true);
  }
  else
  {
    if ((*this->image_connect_count_) > 0)
    {
#if (GAZEBO_MAJOR_VERSION >= 8)
      common::Time cur_time = this->world_->SimTime();
#else
      common::Time cur_time = this->world_->GetSimTime();
#endif
      if (cur_time - this->last_update_time_ >= this->update_period_)
      {
        this->PutCameraData(_image);
        this->PublishCameraInfo();
        this->last_update_time_ = cur_time;
      }
    }
  }
}

template <class Base>
void GazeboRosThermalCamera_<Base>::OnNewImageFrame(const unsigned char *_image,
    unsigned int _width, unsigned int _height, unsigned int _depth,
    const std::string &_format)
{
  OnNewFrame(_image, _width, _height, _depth, _format);
}

////////////////////////////////////////////////////////////////////////////////
// Put camera_ data to the interface
template <class Base>
void GazeboRosThermalCamera_<Base>::PutCameraData(const unsigned char *_src, common::Time &last_update_time)
{
  this->sensor_update_time_ = last_update_time;
  this->PutCameraData(_src);
}

template <class Base>
void GazeboRosThermalCamera_<Base>::PutCameraData(const unsigned char *_src)
{
  if (!this->initialized_ || this->height_ <=0 || this->width_ <=0)
    return;

  this->lock_.lock();

  // copy data into image
  this->image_msg_.header.frame_id = this->frame_name_;
  this->image_msg_.header.stamp.sec = this->sensor_update_time_.sec;
  this->image_msg_.header.stamp.nsec = this->sensor_update_time_.nsec;

  /// don't bother if there are no subscribers
  if ((*this->image_connect_count_) > 0)
  {
    this->image_msg_.width = this->width_;
    this->image_msg_.height = this->height_;
    this->image_msg_.encoding = sensor_msgs::image_encodings::MONO8;
    this->image_msg_.step = this->image_msg_.width;

    size_t size = this->width_ * this->height_;

    std::vector<uint8_t>& data (this->image_msg_.data);
    data.resize(size);

    size_t img_index = 0;

    for (size_t i = 0; i < size; ++i){
      if ((_src[img_index] >254) && (_src[img_index+1] < 1) && (_src[img_index+2] < 1)){
        //RGB [255,0,0] translates to white (white hot)
        data[i]= 255;
      }else{
        //Everything else is written to the MONO8 output image much darker
        data[i]= (_src[img_index] + _src[img_index+1] + _src[img_index+2]) /8 ;
      }
      img_index += 3;
    }

    // publish to ros
    this->image_pub_.publish(this->image_msg_);
  }

  this->lock_.unlock();
}

}
