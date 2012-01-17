//=================================================================================================
// Copyright (c) 2011, Johannes Meyer, TU Darmstadt
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


#ifndef HECTOR_GAZEBO_PLUGINS_GAZEBO_ROS_IMU_H
#define HECTOR_GAZEBO_PLUGINS_GAZEBO_ROS_IMU_H

#define USE_CBQ
#ifdef USE_CBQ
#include <ros/callback_queue.h>
#include <ros/advertise_options.h>
#endif

#include <gazebo/Controller.hh>
#include <gazebo/Entity.hh>
#include <gazebo/Model.hh>
#include <gazebo/Body.hh>
#include <gazebo/Param.hh>
#include <gazebo/Time.hh>

#include <ros/ros.h>
#include "boost/thread/mutex.hpp"
#include <sensor_msgs/Imu.h>
#include <std_srvs/Empty.h>
#include <hector_gazebo_plugins/SetBias.h>


namespace gazebo
{
   class GazeboRosIMU : public Controller
   {
      /// \brief Constructor
      public: GazeboRosIMU(Entity *parent );

      /// \brief Destructor
      public: virtual ~GazeboRosIMU();

      /// \brief Load the controller
      /// \param node XML config node
      protected: virtual void LoadChild(XMLConfigNode *node);

      /// \brief Init the controller
      protected: virtual void InitChild();

      /// \brief Update the controller
      protected: virtual void UpdateChild();

      /// \brief Finalize the controller
      protected: virtual void FiniChild();

      /// \brief The parent Model
      private: Model *myParent;

      /// \brief The parent Model
      private: Body *myBody; //Gazebo/ODE body

      /// \brief pointer to ros node
      private: ros::NodeHandle* rosnode_;
      private: ros::Publisher pub_;

      /// \brief ros message
      private: sensor_msgs::Imu imuMsg;

      /// \brief store bodyname
      private: ParamT<std::string> *bodyNameP;
      private: std::string bodyName;

      /// \brief topic name
      private: ParamT<std::string> *topicNameP;
      private: std::string topicName;

      /// \brief allow specifying constant offsets, drift and noise
      private: ParamT<Vector3> *accelOffsetP;
      private: Vector3 accelOffset;
      private: ParamT<Vector3> *accelDriftP;
      private: Vector3 accelDrift;
      private: ParamT<Vector3> *accelDriftFrequencyP;
      private: Vector3 accelDriftFrequency;
      private: ParamT<Vector3> *accelGaussianNoiseP;
      private: Vector3 accelGaussianNoise;
      private: ParamT<Vector3> *rateOffsetP;
      private: Vector3 rateOffset;
      private: ParamT<Vector3> *rateDriftP;
      private: Vector3 rateDrift;
      private: ParamT<Vector3> *rateDriftFrequencyP;
      private: Vector3 rateDriftFrequency;
      private: ParamT<Vector3> *rateGaussianNoiseP;
      private: Vector3 rateGaussianNoise;
      private: ParamT<double> *headingOffsetP;
      private: double headingOffset;
      private: ParamT<double> *headingDriftP;
      private: double headingDrift;
      private: ParamT<double> *headingDriftFrequencyP;
      private: double headingDriftFrequency;
      private: ParamT<double> *headingGaussianNoiseP;
      private: double headingGaussianNoise;
      private: ParamT<Vector3> *rpyOffsetsP;
      private: ParamT<double> *gaussianNoiseP;

      /// \brief Some working variables
      private: Vector3 accelCurrentDrift;
      private: Vector3 accelCurrentError;
      private: Vector3 rateCurrentDrift;
      private: Vector3 rateCurrentError;
      private: double headingCurrentDrift;
      private: double headingCurrentError;

      /// \brief Update drift model
      private: double updateCurrentError(double &currentDrift, double dt, double driftFrequency,  double drift,  double offset, double noise);

      /// \brief A mutex to lock access to fields that are used in message callbacks
      private: boost::mutex lock;

      /// \brief save last_time
      private: Time last_time;
      private: Vector3 accel;
      private: Vector3 rate;
      private: Vector3 gravity;
      private: Vector3 gravity_body;

      /// \brief Gaussian noise generator
      private: double GaussianKernel(double mu,double sigma);

      /// \brief for setting ROS name space
      private: ParamT<std::string> *robotNamespaceP;
      private: std::string robotNamespace;

      /// \brief call back when using service
      private: bool ServiceCallback(std_srvs::Empty::Request &req,
                                    std_srvs::Empty::Response &res);
      private: ros::ServiceServer srv_;
      private: ParamT<std::string> *serviceNameP;
      private: std::string serviceName;

      /// \brief Bias service callbacks
      private: bool SetAccelBiasCallback(hector_gazebo_plugins::SetBias::Request &req, hector_gazebo_plugins::SetBias::Response &res);
      private: bool SetRateBiasCallback(hector_gazebo_plugins::SetBias::Request &req, hector_gazebo_plugins::SetBias::Response &res);
      private: ros::ServiceServer accelBiasService;
      private: ros::ServiceServer rateBiasService;

#ifdef USE_CBQ
      private: ros::CallbackQueue imu_queue_;
      private: void IMUQueueThread();
      private: boost::thread callback_queue_thread_;
#endif
   };

/** \} */
/// @}


}

#endif // HECTOR_GAZEBO_PLUGINS_GAZEBO_ROS_IMU_H
