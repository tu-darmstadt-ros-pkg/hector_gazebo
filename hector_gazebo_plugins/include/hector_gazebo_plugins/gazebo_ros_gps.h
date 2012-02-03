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

#ifndef HECTOR_GAZEBO_PLUGINS_GAZEBO_ROS_GPS_H
#define HECTOR_GAZEBO_PLUGINS_GAZEBO_ROS_GPS_H

#include <gazebo/Controller.hh>
#include <gazebo/Entity.hh>
#include <gazebo/Model.hh>
#include <gazebo/Body.hh>
#include <gazebo/Param.hh>
#include <gazebo/Time.hh>

#include <ros/ros.h>
#include <sensor_msgs/NavSatFix.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <hector_gazebo_plugins/sensor_model.h>

namespace gazebo
{

class GazeboRosGps : public Controller
{
public:
  GazeboRosGps(Entity *parent);
  virtual ~GazeboRosGps();

protected:
  virtual void LoadChild(XMLConfigNode *node);
  virtual void InitChild();
  virtual void UpdateChild();
  virtual void FiniChild();

private:
  Model *parent_;
  Body *body_;

  ros::NodeHandle* node_handle_;
  ros::Publisher fix_publisher_;
  ros::Publisher velocity_publisher_;

  sensor_msgs::NavSatFix fix_;
  geometry_msgs::Vector3Stamped velocity_;

  ParamT<std::string> *body_name_;
  ParamT<std::string> *namespace_;
  ParamT<std::string> *frame_id_;
  ParamT<std::string> *fix_topic_;
  ParamT<std::string> *velocity_topic_;

  ParamT<double> *reference_latitude_;
  ParamT<double> *reference_longitude_;
  ParamT<double> *reference_heading_;
  ParamT<double> *reference_altitude_;
  ParamT<sensor_msgs::NavSatStatus::_status_type> *status_;
  ParamT<sensor_msgs::NavSatStatus::_service_type> *service_;

  SensorModel3 position_error_model_;
  SensorModel3 velocity_error_model_;
};

} // namespace gazebo

#endif // HECTOR_GAZEBO_PLUGINS_GAZEBO_ROS_GPS_H
