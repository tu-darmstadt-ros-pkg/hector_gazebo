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

#include <hector_gazebo_plugins/gazebo_ros_velodyne.h>
#include <gazebo/common/Events.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/sensors/RaySensor.hh>
#include <sensor_msgs/PointCloud2.h>

#include <limits>

namespace gazebo {

GazeboRosVelodyne::GazeboRosVelodyne()
{
}

////////////////////////////////////////////////////////////////////////////////
// Destructor
GazeboRosVelodyne::~GazeboRosVelodyne()
{
  updateTimer.Disconnect(updateConnection);
  sensor_->SetActive(false);

  dynamic_reconfigure_server_.reset();

  node_handle_->shutdown();
  delete node_handle_;
}

////////////////////////////////////////////////////////////////////////////////
// Load the controller
void GazeboRosVelodyne::Load(sensors::SensorPtr _sensor, sdf::ElementPtr _sdf)
{
  // Get then name of the parent sensor
  sensor_ = boost::dynamic_pointer_cast<sensors::RaySensor>(_sensor);
  if (!sensor_)
  {
    gzthrow("GazeboRosVelodyne requires a Ray Sensor as its parent");
    return;
  }

  // Get the world name.
  std::string worldName = sensor_->GetWorldName();
  world = physics::get_world(worldName);
  

  // default parameters
  namespace_.clear();
  topic_ = "sonar";
  frame_id_ = "/velodyne_link";
  updateRate_ = 10;

  // load parameters
  if (_sdf->HasElement("robotNamespace"))
    namespace_ = _sdf->GetElement("robotNamespace")->GetValue()->GetAsString();

  if (_sdf->HasElement("frameName"))
    frame_id_ = _sdf->GetElement("frameName")->GetValue()->GetAsString();

  if (_sdf->HasElement("topicName"))
    topic_ = _sdf->GetElement("topicName")->GetValue()->GetAsString();

  sensor_model_.Load(_sdf);

  point_step = 16;
  cloud_msg_.header.frame_id = frame_id_;
  cloud_msg_.fields.resize(4);
  cloud_msg_.fields[0].name = "x";
  cloud_msg_.fields[0].offset = 0;
  cloud_msg_.fields[0].datatype = sensor_msgs::PointField::FLOAT32;
  cloud_msg_.fields[0].count = 1;
  cloud_msg_.fields[1].name = "y";
  cloud_msg_.fields[1].offset = 4;
  cloud_msg_.fields[1].datatype = sensor_msgs::PointField::FLOAT32;
  cloud_msg_.fields[1].count = 1;
  cloud_msg_.fields[2].name = "z";
  cloud_msg_.fields[2].offset = 8;
  cloud_msg_.fields[2].datatype = sensor_msgs::PointField::FLOAT32;
  cloud_msg_.fields[2].count = 1;
  cloud_msg_.fields[3].name = "intensity";
  cloud_msg_.fields[3].offset = 12;
  cloud_msg_.fields[3].datatype = sensor_msgs::PointField::FLOAT32;
  cloud_msg_.fields[3].count = 1;
  // cloud_msg_.fields[4].name = "ring";
  // cloud_msg_.fields[4].offset = 20;
  // cloud_msg_.fields[4].datatype = sensor_msgs::PointField::UINT16;
  // cloud_msg_.fields[4].count = 1;

  cloud_msg_.point_step = point_step;

  // cloud_msg_.data.resize(verticalRangeCount * rangeCount * POINT_STEP);

  // range_.header.frame_id = frame_id_;
  // range_.radiation_type = sensor_msgs::Range::ULTRASOUND;
  // range_.field_of_view = std::min(fabs((sensor_->GetAngleMax() - sensor_->GetAngleMin()).Radian()), fabs((sensor_->GetVerticalAngleMax() - sensor_->GetVerticalAngleMin()).Radian()));
  // range_.max_range = sensor_->GetRangeMax();
  // range_.min_range = sensor_->GetRangeMin();

  // Make sure the ROS node for Gazebo has already been initialized
  if (!ros::isInitialized())
  {
    ROS_FATAL_STREAM("A ROS node for Gazebo has not been initialized, unable to load plugin. "
      << "Load the Gazebo system plugin 'libgazebo_ros_api_plugin.so' in the gazebo_ros package)");
    return;
  }

  node_handle_ = new ros::NodeHandle(namespace_);
  publisher_ = node_handle_->advertise<sensor_msgs::PointCloud2>(topic_, 1);

  // setup dynamic_reconfigure server
  dynamic_reconfigure_server_.reset(new dynamic_reconfigure::Server<SensorModelConfig>(ros::NodeHandle(*node_handle_, topic_)));
  dynamic_reconfigure_server_->setCallback(boost::bind(&SensorModel::dynamicReconfigureCallback, &sensor_model_, _1, _2));

  Reset();

  // connect Update function
  sensor_->SetUpdateRate(10);//todo
  updateTimer.setUpdateRate(10);
  updateTimer.Load(world, _sdf);
  updateConnection = updateTimer.Connect(boost::bind(&GazeboRosVelodyne::Update, this));

  // activate RaySensor
  sensor_->SetActive(true);
}

void GazeboRosVelodyne::Reset()
{
  updateTimer.Reset();
  sensor_model_.reset();
}

////////////////////////////////////////////////////////////////////////////////
// Update the controller
void GazeboRosVelodyne::Update()
{
  
  // clock_t start = clock();

  common::Time sim_time = world->GetSimTime();
  double dt = updateTimer.getTimeSinceLastUpdate().Double();

  // activate RaySensor if it is not yet active
  // if (!sensor_->IsActive()) sensor_->SetActive(true);
  sensor_->SetActive(false);

  int vertialRangeCnt = sensor_->GetLaserShape()->GetVerticalSampleCount();
  int rangeCnt = sensor_->GetLaserShape()->GetSampleCount();
  if(vertialRangeCnt*rangeCnt == 0) return;

  
  double maxAngle = sensor_->GetAngleMax().Radian();
  double minAngle = sensor_->GetAngleMin().Radian();
  
  double vertialMaxAngle = sensor_->GetVerticalAngleMax().Radian();
  double vertialMinAngle = sensor_->GetVerticalAngleMin().Radian();
  

  // std::cout << "count: " << vertialRangeCnt << "  " 
  //      << rangeCnt << "  " 
  //      << vertialRangeCnt*rangeCnt << std::endl;
  double delta_angle = (maxAngle - minAngle )/rangeCnt;
  double delta_vertial_angle = (vertialMaxAngle - vertialMinAngle)/vertialRangeCnt;
  
  double maxRange = sensor_->GetRangeMax();
  double minRange = sensor_->GetRangeMin();
  
  cloud_msg_.header.stamp.sec  = (world->GetSimTime()).sec;
  cloud_msg_.header.stamp.nsec = (world->GetSimTime()).nsec;
  
  cloud_msg_.data.resize(vertialRangeCnt*rangeCnt*point_step);

  cloud_msg_.width = rangeCnt;
  cloud_msg_.height = vertialRangeCnt;

  cloud_msg_.is_bigendian = false;
  cloud_msg_.is_dense = true;

  std::vector<double> ranges;
  sensor_->GetRanges(ranges);

  // std::cout << "ranges size: " << ranges.size() << std::endl;

  float x, y, z, intensity;
  double ray;
  float * ptr = (float*)( cloud_msg_.data.data() );
  
  for(int i = 0; i < vertialRangeCnt; ++i) 
  {
    double vertialAngle = vertialMinAngle + delta_vertial_angle*i; 
    double angle = minAngle;
    double cos_va = cos(vertialAngle);
    double sin_va = sin(vertialAngle);
    int range_id = i*rangeCnt; 
    for(int j = 0; j < rangeCnt; j++)
    { 
      // ray = sensor_->GetLaserShape()->GetRange(range_id);
      ray = ranges[range_id];
      //intensity = sensor_->GetLaserShape()->GetRetro(i*rangeCnt + j);
      ray = sensor_model_(ray, dt); //add noise

       x = ray*cos_va*cos(angle);
       y = ray*cos_va*sin(angle);
       z = -ray*sin_va;

       *(ptr + 0)  = x;
       *(ptr + 1)  = y;
       *(ptr + 2)  = z;
       *(ptr + 3) = intensity;
       ptr += cloud_msg_.fields.size();
       angle += delta_angle;
       range_id++;
    }

  }
  // std::cout << "cloud_msg_ t1: " << (double)(clock() - start)/CLOCKS_PER_SEC << std::endl;
  // std::cout << "publish cloud_msg_" << std::endl;
  publisher_.publish(cloud_msg_);
  // std::cout << "cloud_msg_ duration: " << (double)(clock() - start)/CLOCKS_PER_SEC << std::endl;
  sensor_->SetActive(true);
}

// Register this plugin with the simulator
GZ_REGISTER_SENSOR_PLUGIN(GazeboRosVelodyne)

} // namespace gazebo
