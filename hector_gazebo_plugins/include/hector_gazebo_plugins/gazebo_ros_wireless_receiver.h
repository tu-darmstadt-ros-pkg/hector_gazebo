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

#ifndef HECTOR_GAZEBO_PLUGINS_GAZEBO_ROS_WIRELESS_RECEIVER_H
#define HECTOR_GAZEBO_PLUGINS_GAZEBO_ROS_WIRELESS_RECEIVER_H

#include <gazebo/common/Plugin.hh>

#include <ros/callback_queue.h>
#include <ros/ros.h>

#include <sensor_msgs/PointCloud.h>
#include <geometry_msgs/PoseStamped.h>
#include <hector_gazebo_plugins/sensor_model.h>
#include <hector_gazebo_plugins/update_timer.h>


#include <dynamic_reconfigure/server.h>
#include <gazebo/sensors/Sensor.hh>

namespace gazebo
{

class GazeboRosWirelessReceiver : public SensorPlugin
{
public:
  GazeboRosWirelessReceiver();
  virtual ~GazeboRosWirelessReceiver();

protected:
  virtual void Load(sensors::SensorPtr _sensor, sdf::ElementPtr _sdf);
  virtual void Reset();
  virtual void Update();

private:
  /// \brief The parent World
  physics::WorldPtr world;

  sensors::SensorPtr sensor_;

  ros::NodeHandle* node_handle_;

  ros::Publisher transmitter_pub_;
  ros::Publisher rss_pub_;
  ros::Publisher AoA_pub_;
  ros::Publisher receiver_pub_;
  int transmitter_count_;


  /// \brief Parent entity of this sensor
  physics::EntityPtr parentEntity;

  std::string namespace_;
  std::string topic_;
  std::string frame_id_;

  SensorModel rss_sensor_model_;
  SensorModel3 AoA_sensor_model_;

  UpdateTimer updateTimer;
  event::ConnectionPtr updateConnection;

  boost::shared_ptr<dynamic_reconfigure::Server<SensorModelConfig> > dynamic_reconfigure_server_;
};

} // namespace gazebo

#endif // HECTOR_GAZEBO_PLUGINS_GAZEBO_ROS_SONAR_H
