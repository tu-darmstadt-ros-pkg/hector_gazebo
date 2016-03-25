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

  if (_sdf->HasElement("frameId"))
    frame_id_ = _sdf->GetElement("frameId")->GetValue()->GetAsString();

  if (_sdf->HasElement("topicName"))
    topic_ = _sdf->GetElement("topicName")->GetValue()->GetAsString();

  sensor_model_.Load(_sdf);

  point_step = 16;
  pointcloud2.header.frame_id = frame_id_;
  pointcloud2.fields.resize(4);
  pointcloud2.fields[0].name = "x";
  pointcloud2.fields[0].offset = 0;
  pointcloud2.fields[0].datatype = sensor_msgs::PointField::FLOAT32;
  pointcloud2.fields[0].count = 1;
  pointcloud2.fields[1].name = "y";
  pointcloud2.fields[1].offset = 4;
  pointcloud2.fields[1].datatype = sensor_msgs::PointField::FLOAT32;
  pointcloud2.fields[1].count = 1;
  pointcloud2.fields[2].name = "z";
  pointcloud2.fields[2].offset = 8;
  pointcloud2.fields[2].datatype = sensor_msgs::PointField::FLOAT32;
  pointcloud2.fields[2].count = 1;
  pointcloud2.fields[3].name = "intensity";
  pointcloud2.fields[3].offset = 12;
  pointcloud2.fields[3].datatype = sensor_msgs::PointField::FLOAT32;
  pointcloud2.fields[3].count = 1;
  // pointcloud2.fields[4].name = "ring";
  // pointcloud2.fields[4].offset = 20;
  // pointcloud2.fields[4].datatype = sensor_msgs::PointField::UINT16;
  // pointcloud2.fields[4].count = 1;

  pointcloud2.point_step = point_step;

  // pointcloud2.data.resize(verticalRangeCount * rangeCount * POINT_STEP);

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
  updateTimer.setUpdateRate(10.0);
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
  common::Time sim_time = world->GetSimTime();
  double dt = updateTimer.getTimeSinceLastUpdate().Double();

  // activate RaySensor if it is not yet active
  if (!sensor_->IsActive()) sensor_->SetActive(true);

  //std::cout << "line: " << __LINE__ << std::endl;
  float maxAngle = sensor_->GetAngleMax().Radian();
  float minAngle = sensor_->GetAngleMin().Radian();
  //std::cout << "line: " << __LINE__ << std::endl;
  float vertialMaxAngle = sensor_->GetVerticalAngleMax().Radian();
  float vertialMinAngle = sensor_->GetVerticalAngleMin().Radian();
  //std::cout << "line: " << __LINE__ << std::endl;
  int vertialRangeCnt = sensor_->GetLaserShape()->GetVerticalSampleCount();
  int rangeCnt = sensor_->GetLaserShape()->GetSampleCount();
  // std::cout << "count: " << vertialRangeCnt << "  " 
  //      << rangeCnt << "  " 
  //      << vertialRangeCnt*rangeCnt << std::endl;
  float delta_angle = (maxAngle - minAngle )/rangeCnt;
  float delta_vertial_angle = (vertialMaxAngle - vertialMinAngle)/vertialRangeCnt;
  //std::cout << "line: " << __LINE__ << std::endl;
  float maxRange = sensor_->GetRangeMax();
  float minRange = sensor_->GetRangeMin();
  //std::cout << "line: " << __LINE__ << std::endl;
  pointcloud2.header.stamp.sec  = (world->GetSimTime()).sec;
  pointcloud2.header.stamp.nsec = (world->GetSimTime()).nsec;
  //std::cout << "line: " << __LINE__ << std::endl;
  pointcloud2.data.resize(vertialRangeCnt*rangeCnt*point_step);
  pointcloud2.width = rangeCnt;
  pointcloud2.height = vertialRangeCnt;
  pointcloud2.is_bigendian = false;
  pointcloud2.is_dense = true;

  float x, y, z;
  float ray, intensity;
  uint8_t * ptr = pointcloud2.data.data();
  //std::cout << "line: " << __LINE__ << std::endl;
  for(int i = 0; i < vertialRangeCnt; ++i) 
  {
    float vertialAngle = vertialMinAngle + delta_vertial_angle*i; 
    //std::cout << "line: " << __LINE__ << std::endl;
    for(int j = 0; j < rangeCnt; j++)
    {
      //std::cout << "line: " << __LINE__ << std::endl;
      float angle = minAngle + delta_angle*j;
      ray = sensor_->GetLaserShape()->GetRange(i*rangeCnt + j);
      intensity = sensor_->GetLaserShape()->GetRetro(i*rangeCnt + j);
      ray = sensor_model_(ray, dt); //add noise

       x = ray*cos(vertialAngle)*cos(angle);
       y = ray*cos(vertialAngle)*sin(angle);
       z = ray*sin(vertialAngle);

       *((float*) (ptr + 0) ) = x;
       *((float*) (ptr + 4) ) = y;
       *((float*) (ptr + 8) ) = z;
       *((float*) (ptr + 12) ) = intensity;
       ptr += point_step;
    }
    
    //x y z

  }
  //std::cout << "line: " << __LINE__ << std::endl;
  publisher_.publish(pointcloud2);
  // find ray with minimal range
  // range_.range = std::numeric_limits<sensor_msgs::Range::_range_type>::max();
  // int num_ranges = sensor_->GetLaserShape()->GetSampleCount() * sensor_->GetLaserShape()->GetVerticalSampleCount();
  // for(int i = 0; i < num_ranges; ++i) {
  //   double ray = sensor_->GetLaserShape()->GetRange(i);
  //   if (ray < range_.range) range_.range = ray;
  // }

  // add Gaussian noise (and limit to min/max range)
  // if (range_.range < range_.max_range) {
  //   range_.range = sensor_model_(range_.range, dt);
  //   if (range_.range < range_.min_range) range_.range = range_.min_range;
  //   if (range_.range > range_.max_range) range_.range = range_.max_range;
  // }


}

// Register this plugin with the simulator
GZ_REGISTER_SENSOR_PLUGIN(GazeboRosVelodyne)

} // namespace gazebo
