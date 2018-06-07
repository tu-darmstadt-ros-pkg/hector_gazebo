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

#ifndef HECTOR_GAZEBO_PLUGINS_GAZEBO_ROS_IMU_H
#define HECTOR_GAZEBO_PLUGINS_GAZEBO_ROS_IMU_H

// #define USE_CBQ
#ifdef USE_CBQ
#include <ros/callback_queue.h>
#include <ros/advertise_options.h>
#endif

#include <gazebo/common/Plugin.hh>

#include <ros/ros.h>
#include <boost/thread/mutex.hpp>
#include <sensor_msgs/Imu.h>
#include <std_srvs/Empty.h>
#include <hector_gazebo_plugins/SetBias.h>
#include <hector_gazebo_plugins/sensor_model.h>
#include <hector_gazebo_plugins/update_timer.h>

#include <dynamic_reconfigure/server.h>

namespace gazebo
{
   class GazeboRosIMU : public ModelPlugin
   {
   public:
      /// \brief Constructor
      GazeboRosIMU();

      /// \brief Destructor
      virtual ~GazeboRosIMU();

   protected:
      virtual void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf);
      virtual void Reset();
      virtual void Update();

   private:
      /// \brief The parent World
      physics::WorldPtr world;

      /// \brief The link referred to by this plugin
      physics::LinkPtr link;

      /// \brief pointer to ros node
      ros::NodeHandle* node_handle_;
      ros::Publisher pub_;
      ros::Publisher bias_pub_;

      /// \brief ros message
      sensor_msgs::Imu imuMsg;
      sensor_msgs::Imu biasMsg;

      /// \brief store link name
      std::string link_name_;

      /// \brief frame id
      std::string frame_id_;

      /// \brief topic name
      std::string topic_;
      std::string bias_topic_;

      /// \brief allow specifying constant xyz and rpy offsets
#if (GAZEBO_MAJOR_VERSION >= 8)
      ignition::math::Pose3d offset_;
#else
      math::Pose offset_;
#endif

      /// \brief Sensor models
      SensorModel3 accelModel;
      SensorModel3 rateModel;
      SensorModel yawModel;

      /// \brief A mutex to lock access to fields that are used in message callbacks
      boost::mutex lock;

      /// \brief save current body/physics state
#if (GAZEBO_MAJOR_VERSION >= 8)
      ignition::math::Quaterniond orientation;
      ignition::math::Vector3d velocity;
      ignition::math::Vector3d accel;
      ignition::math::Vector3d rate;
      ignition::math::Vector3d gravity;
#else
      math::Quaternion orientation;
      math::Vector3 velocity;
      math::Vector3 accel;
      math::Vector3 rate;
      math::Vector3 gravity;
#endif

      /// \brief Gaussian noise generator
      double GaussianKernel(double mu,double sigma);

      /// \brief for setting ROS name space
      std::string namespace_;

      /// \brief call back when using service
      bool ServiceCallback(std_srvs::Empty::Request &req,
                                    std_srvs::Empty::Response &res);
      ros::ServiceServer srv_;
      std::string serviceName;

      /// \brief Bias service callbacks
      bool SetAccelBiasCallback(hector_gazebo_plugins::SetBias::Request &req, hector_gazebo_plugins::SetBias::Response &res);
      bool SetRateBiasCallback(hector_gazebo_plugins::SetBias::Request &req, hector_gazebo_plugins::SetBias::Response &res);
      ros::ServiceServer accelBiasService;
      ros::ServiceServer rateBiasService;

#ifdef USE_CBQ
      ros::CallbackQueue callback_queue_;
      void CallbackQueueThread();
      boost::thread callback_queue_thread_;
#endif

      UpdateTimer updateTimer;
      event::ConnectionPtr updateConnection;

      boost::shared_ptr<dynamic_reconfigure::Server<SensorModelConfig> > dynamic_reconfigure_server_accel_, dynamic_reconfigure_server_rate_, dynamic_reconfigure_server_yaw_;
   };
}

#endif // HECTOR_GAZEBO_PLUGINS_GAZEBO_ROS_IMU_H
