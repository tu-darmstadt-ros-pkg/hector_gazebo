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

#include <hector_gazebo_plugins/gazebo_ros_gps.h>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/SphericalCoordinates.hh>

#include <tf2/LinearMath/Transform.h>

#include <iostream>

// WGS84 constants
static const double equatorial_radius = 6378137.0;
static const double flattening = 1.0/298.257223563;
static const double excentrity2 = 2*flattening - flattening*flattening;

// default reference position
static const double DEFAULT_REFERENCE_LATITUDE  = 49.9;
static const double DEFAULT_REFERENCE_LONGITUDE = 8.9;
static const double DEFAULT_REFERENCE_HEADING   = 0.0;
static const double DEFAULT_REFERENCE_ALTITUDE  = 0.0;
namespace gazebo {

GazeboRosGps::GazeboRosGps()
{
}

////////////////////////////////////////////////////////////////////////////////
// Destructor
GazeboRosGps::~GazeboRosGps()
{
  updateTimer.Disconnect(updateConnection);
}

////////////////////////////////////////////////////////////////////////////////
// Load the controller
void GazeboRosGps::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
{
  world = _model->GetWorld();

  node_ = gazebo_ros::Node::Get(_sdf);

  // load parameters
  if (!_sdf->HasElement("robotNamespace"))
    namespace_.clear();
  else
    namespace_ = _sdf->GetElement("robotNamespace")->GetValue()->GetAsString();

  if (!_sdf->HasElement("bodyName"))
  {
    link = _model->GetLink();
    link_name_ = link->GetName();
  }
  else {
    link_name_ = _sdf->GetElement("bodyName")->GetValue()->GetAsString();
    link = _model->GetLink(link_name_);
  }

  if (!link)
  {
    RCLCPP_FATAL(this->node_->get_logger(), "GazeboRosGps plugin error: bodyName: %s does not exist\n", link_name_.c_str());
    return;
  }

  // default parameters
  frame_id_ = "/world";
  fix_topic_ = "fix";
  velocity_topic_ = "fix_velocity";

  reference_latitude_  = DEFAULT_REFERENCE_LATITUDE;
  reference_longitude_ = DEFAULT_REFERENCE_LONGITUDE;
  reference_heading_   = DEFAULT_REFERENCE_HEADING * M_PI/180.0;
  reference_altitude_  = DEFAULT_REFERENCE_ALTITUDE;

  if (_sdf->HasElement("useWorldSphericalCoordinates"))
  {
    bool use_world_coords = false;
    if (_sdf->GetElement("useWorldSphericalCoordinates")->GetValue()->Get(use_world_coords) && use_world_coords)
    {
      common::SphericalCoordinatesPtr spherical_coords = world->SphericalCoords();
      reference_latitude_ = spherical_coords->LatitudeReference().Degree();
      reference_longitude_ = spherical_coords->LongitudeReference().Degree();
      // SDF specifies heading counter-clockwise from east, but here it's measured clockwise from north
      reference_heading_ = (M_PI / 2.0) - spherical_coords->HeadingOffset().Radian();
      reference_altitude_ = spherical_coords->GetElevationReference();
    }
  }

  fix_.status.status  = sensor_msgs::msg::NavSatStatus::STATUS_FIX;
  fix_.status.service = 0;

  if (_sdf->HasElement("frameId"))
    frame_id_ = _sdf->GetElement("frameId")->GetValue()->GetAsString();

  if (_sdf->HasElement("topicName"))
    fix_topic_ = _sdf->GetElement("topicName")->GetValue()->GetAsString();

  if (_sdf->HasElement("velocityTopicName"))
    velocity_topic_ = _sdf->GetElement("velocityTopicName")->GetValue()->GetAsString();

  if (_sdf->HasElement("referenceLatitude"))
    _sdf->GetElement("referenceLatitude")->GetValue()->Get(reference_latitude_);

  if (_sdf->HasElement("referenceLongitude"))
    _sdf->GetElement("referenceLongitude")->GetValue()->Get(reference_longitude_);

  if (_sdf->HasElement("referenceHeading"))
    if (_sdf->GetElement("referenceHeading")->GetValue()->Get(reference_heading_))
      reference_heading_ *= M_PI/180.0;

  if (_sdf->HasElement("referenceAltitude"))
    _sdf->GetElement("referenceAltitude")->GetValue()->Get(reference_altitude_);

  if (_sdf->HasElement("status")) {
    int status = fix_.status.status;
    if (_sdf->GetElement("status")->GetValue()->Get(status))
      fix_.status.status = static_cast<sensor_msgs::msg::NavSatStatus::_status_type>(status);
  }

  if (_sdf->HasElement("service")) {
    unsigned int service = fix_.status.service;
    if (_sdf->GetElement("service")->GetValue()->Get(service))
      fix_.status.service = static_cast<sensor_msgs::msg::NavSatStatus::_service_type>(service);
  }


  fix_.header.frame_id = frame_id_;
  velocity_.header.frame_id = frame_id_;

  position_error_model_.Load(node_, _sdf);
  velocity_error_model_.Load(node_, _sdf, "velocity");

  // calculate earth radii
  double temp = 1.0 / (1.0 - excentrity2 * sin(reference_latitude_ * M_PI/180.0) * sin(reference_latitude_ * M_PI/180.0));
  double prime_vertical_radius = equatorial_radius * sqrt(temp);
  radius_north_ = prime_vertical_radius * (1 - excentrity2) * temp;
  radius_east_  = prime_vertical_radius * cos(reference_latitude_ * M_PI/180.0);

  // Make sure the ROS node for Gazebo has already been initialized
  if (!rclcpp::ok())
  {
    RCLCPP_FATAL_STREAM(node_->get_logger(), 
        "A ROS node for Gazebo has not been initialized, unable to load plugin. "
        << "Load the Gazebo system plugin 'libgazebo_ros_api_plugin.so' in the gazebo_ros package)");
    return;
  }

  fix_publisher_ = node_->create_publisher<sensor_msgs::msg::NavSatFix>(fix_topic_, 10);
  velocity_publisher_ = node_->create_publisher<geometry_msgs::msg::Vector3Stamped>(velocity_topic_, 10);

  set_geopose_srv_ = node_->create_service<hector_gazebo_plugins::srv::SetReferenceGeoPose>(fix_topic_ + "/set_reference_geopose",
        std::bind(&GazeboRosGps::setGeoposeCb, this, std::placeholders::_1, std::placeholders::_2));

  // Setup the GNSS parameters
  gnss_config = std::make_shared<GNSSConfig>(node_);

  // Setup the parameters handler
  callback_handle_ = node_->add_on_set_parameters_callback(std::bind(&GazeboRosGps::parametersChangedCallback, this, std::placeholders::_1));

  Reset();

  // connect Update function
  updateTimer.setUpdateRate(4.0);
  updateTimer.Load(world, _sdf);
  updateConnection = updateTimer.Connect(boost::bind(&GazeboRosGps::Update, this));
}

void GazeboRosGps::setGeoposeCb(hector_gazebo_plugins::srv::SetReferenceGeoPose::Request::SharedPtr request,
                                hector_gazebo_plugins::srv::SetReferenceGeoPose::Response::SharedPtr)
{
  reference_latitude_ = request->geo_pose.position.latitude;
  reference_longitude_ = request->geo_pose.position.longitude;
  tf2::Quaternion q(request->geo_pose.orientation.x,
                    request->geo_pose.orientation.y,
                    request->geo_pose.orientation.z,
                    request->geo_pose.orientation.w);
  tf2::Matrix3x3 m(q);
  tf2Scalar yaw, pitch, roll;
  m.getEulerYPR(yaw, pitch, roll);
  reference_heading_ = (M_PI / 2.0) - yaw;
  reference_altitude_ = request->geo_pose.position.altitude;

  Reset();
}

void GazeboRosGps::Reset()
{
  updateTimer.Reset();
  position_error_model_.reset();
  velocity_error_model_.reset();
}

rcl_interfaces::msg::SetParametersResult GazeboRosGps::checkStatusParameters(const std::vector<rclcpp::Parameter> & parameters)
{
  using sensor_msgs::msg::NavSatStatus;

  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;
  result.reason = "";

  // result.successful = false;
  // result.reason = "Parameter not found";

  for (const auto & parameter : parameters) {
    std::string name = parameter.get_name();

    if (name == "status_fix" ||
        name == "status_sbas_fix" ||
        name == "status_gbas_fix" ||
        name == "service_gps" ||
        name == "service_glonass" ||
        name == "service_compass" ||
        name == "service_galileo")
    {
      result.successful = true;
      result.reason = "";

      bool status_fix(false);
      bool status_sbas_fix(false);
      bool status_gbas_fix(false);
      bool service_gps(false);
      bool service_glonass(false);
      bool service_compass(false);
      bool service_galileo(false);

      node_->get_parameter("status_fix", status_fix);
      node_->get_parameter("status_sbas_fix", status_sbas_fix);
      node_->get_parameter("status_gbas_fix", status_gbas_fix);
      node_->get_parameter("service_gps", service_gps);
      node_->get_parameter("service_glonass", service_glonass);
      node_->get_parameter("service_compass", service_compass);
      node_->get_parameter("service_galileo", service_galileo);

      RCLCPP_INFO(node_->get_logger(), "Fix status paramter changed");
    
      if (!status_fix)
      {
        fix_.status.status = NavSatStatus::STATUS_NO_FIX;
      }
      else
      {
        fix_.status.status = (status_sbas_fix ? NavSatStatus::STATUS_SBAS_FIX : 0) |
                            (status_gbas_fix ? NavSatStatus::STATUS_GBAS_FIX : 0);
      }

      fix_.status.service = (service_gps ? NavSatStatus::SERVICE_GPS : 0) |
                            (service_glonass ? NavSatStatus::SERVICE_GLONASS : 0) |
                            (service_compass ? NavSatStatus::SERVICE_COMPASS : 0) |
                            (service_galileo ? NavSatStatus::SERVICE_GALILEO : 0);
    }
  }

  return result;
}

rcl_interfaces::msg::SetParametersResult GazeboRosGps::parametersChangedCallback(const std::vector<rclcpp::Parameter> & parameters)
{
  rcl_interfaces::msg::SetParametersResult result_status, result_pos, result_vel;

  result_status = checkStatusParameters(parameters);
  result_pos = position_error_model_.parametersChangedCallback(parameters);
  result_vel = velocity_error_model_.parametersChangedCallback(parameters);

  // Return the final result
  if (result_status.successful && result_pos.successful && result_vel.successful) {
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;
    result.reason = "";
    return result;
  } else {
    if (!result_status.successful) {
      return result_status;
    } else if (!result_pos.successful) {
      return result_pos;
    } else {
      return result_vel;
    }
  }
}

////////////////////////////////////////////////////////////////////////////////
// Update the controller
void GazeboRosGps::Update()
{
#if (GAZEBO_MAJOR_VERSION >= 8)
  common::Time sim_time = world->SimTime();
  double dt = updateTimer.getTimeSinceLastUpdate().Double();

  ignition::math::Pose3d pose = link->WorldPose();

  ignition::math::Vector3d velocity = velocity_error_model_(link->WorldLinearVel(), dt);
  ignition::math::Vector3d position = position_error_model_(pose.Pos(), dt);
#else
  common::Time sim_time = world->GetSimTime();
  double dt = updateTimer.getTimeSinceLastUpdate().Double();

  math::Pose pose = link->GetWorldPose();

  gazebo::math::Vector3 velocity = velocity_error_model_(link->GetWorldLinearVel(), dt);
  gazebo::math::Vector3 position = position_error_model_(pose.pos, dt);
#endif

  // An offset error in the velocity is integrated into the position error for the next timestep.
  // Note: Usually GNSS receivers have almost no drift in the velocity signal.
  position_error_model_.setCurrentDrift(position_error_model_.getCurrentDrift() + dt * velocity_error_model_.getCurrentDrift());

  fix_.header.stamp = rclcpp::Time(sim_time.sec, sim_time.nsec);
  velocity_.header.stamp = fix_.header.stamp;

#if (GAZEBO_MAJOR_VERSION >= 8)
  fix_.latitude  = reference_latitude_  + ( cos(reference_heading_) * position.X() + sin(reference_heading_) * position.Y()) / radius_north_ * 180.0/M_PI;
  fix_.longitude = reference_longitude_ - (-sin(reference_heading_) * position.X() + cos(reference_heading_) * position.Y()) / radius_east_  * 180.0/M_PI;
  fix_.altitude  = reference_altitude_  + position.Z();
  velocity_.vector.x =  cos(reference_heading_) * velocity.X() + sin(reference_heading_) * velocity.Y();
  velocity_.vector.y = -sin(reference_heading_) * velocity.X() + cos(reference_heading_) * velocity.Y();
  velocity_.vector.z = velocity.Z();

  fix_.position_covariance_type = sensor_msgs::msg::NavSatFix::COVARIANCE_TYPE_DIAGONAL_KNOWN;
  fix_.position_covariance[0] = position_error_model_.drift.X()*position_error_model_.drift.X() + position_error_model_.gaussian_noise.X()*position_error_model_.gaussian_noise.X();
  fix_.position_covariance[4] = position_error_model_.drift.Y()*position_error_model_.drift.Y() + position_error_model_.gaussian_noise.Y()*position_error_model_.gaussian_noise.Y();
  fix_.position_covariance[8] = position_error_model_.drift.Z()*position_error_model_.drift.Z() + position_error_model_.gaussian_noise.Z()*position_error_model_.gaussian_noise.Z();
#else
  fix_.latitude  = reference_latitude_  + ( cos(reference_heading_) * position.x + sin(reference_heading_) * position.y) / radius_north_ * 180.0/M_PI;
  fix_.longitude = reference_longitude_ - (-sin(reference_heading_) * position.x + cos(reference_heading_) * position.y) / radius_east_  * 180.0/M_PI;
  fix_.altitude  = reference_altitude_  + position.z;
  velocity_.vector.x =  cos(reference_heading_) * velocity.x + sin(reference_heading_) * velocity.y;
  velocity_.vector.y = -sin(reference_heading_) * velocity.x + cos(reference_heading_) * velocity.y;
  velocity_.vector.z = velocity.z;

  fix_.position_covariance_type = sensor_msgs::msg::NavSatFix::COVARIANCE_TYPE_DIAGONAL_KNOWN;
  fix_.position_covariance[0] = position_error_model_.drift.x*position_error_model_.drift.x + position_error_model_.gaussian_noise.x*position_error_model_.gaussian_noise.x;
  fix_.position_covariance[4] = position_error_model_.drift.y*position_error_model_.drift.y + position_error_model_.gaussian_noise.y*position_error_model_.gaussian_noise.y;
  fix_.position_covariance[8] = position_error_model_.drift.z*position_error_model_.drift.z + position_error_model_.gaussian_noise.z*position_error_model_.gaussian_noise.z;
#endif

  fix_publisher_->publish(fix_);
  velocity_publisher_->publish(velocity_);
}

// Register this plugin with the simulator
GZ_REGISTER_MODEL_PLUGIN(GazeboRosGps)

} // namespace gazebo
