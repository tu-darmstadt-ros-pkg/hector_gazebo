/*
 * Copyright 2015 Stefan Kohlbrecher, TU Darmstadt
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
 * Desc: Simple model controller that uses a twist message to exert
 *       forces on a robot, resulting in motion. Based on the
 *       planar_move plugin by Piyush Khandelwal.
 * Author: Stefan Kohlbrecher, Sammy Pfeiffer
 * Date: 06 August 2015, 21 December 2018
 */

#include <hector_gazebo_plugins/gazebo_ros_force_based_move.h>

namespace gazebo 
{

  GazeboRosForceBasedMove::GazeboRosForceBasedMove() {}

  GazeboRosForceBasedMove::~GazeboRosForceBasedMove() {
    rosnode_->shutdown();
  }

  // Load the controller
  void GazeboRosForceBasedMove::Load(physics::ModelPtr parent,
      sdf::ElementPtr sdf) 
  {

    parent_ = parent;

    /* Parse parameters */

    robot_namespace_ = "";
    if (!sdf->HasElement("robotNamespace")) 
    {
      ROS_INFO("PlanarMovePlugin missing <robotNamespace>, "
          "defaults to \"%s\"", robot_namespace_.c_str());
    }
    else 
    {
      robot_namespace_ = 
        sdf->GetElement("robotNamespace")->Get<std::string>();
    }

    command_topic_ = "cmd_vel";
    if (!sdf->HasElement("commandTopic")) 
    {
      ROS_WARN("PlanarMovePlugin (ns = %s) missing <commandTopic>, "
          "defaults to \"%s\"", 
          robot_namespace_.c_str(), command_topic_.c_str());
    } 
    else 
    {
      command_topic_ = sdf->GetElement("commandTopic")->Get<std::string>();
    }

    odometry_topic_ = "odom";
    if (!sdf->HasElement("odometryTopic")) 
    {
      ROS_WARN("PlanarMovePlugin (ns = %s) missing <odometryTopic>, "
          "defaults to \"%s\"", 
          robot_namespace_.c_str(), odometry_topic_.c_str());
    } 
    else 
    {
      odometry_topic_ = sdf->GetElement("odometryTopic")->Get<std::string>();
    }

    odometry_frame_ = "odom";
    if (!sdf->HasElement("odometryFrame")) 
    {
      ROS_WARN("PlanarMovePlugin (ns = %s) missing <odometryFrame>, "
          "defaults to \"%s\"",
          robot_namespace_.c_str(), odometry_frame_.c_str());
    }
    else 
    {
      odometry_frame_ = sdf->GetElement("odometryFrame")->Get<std::string>();
    }
    
    // All these are related somehow to the friction
    // coefficients of the link that we are pushing with the force
    // as in, mu, mu2, fdir
    torque_yaw_velocity_p_gain_ = 0.0;
    force_x_velocity_p_gain_ = 0.0;
    force_y_velocity_p_gain_ = 0.0;
    torque_yaw_velocity_i_gain_ = 0.0;
    force_x_velocity_i_gain_ = 0.0;
    force_y_velocity_i_gain_ = 0.0;
    torque_yaw_velocity_d_gain_ = 0.0;
    force_x_velocity_d_gain_ = 0.0;
    force_y_velocity_d_gain_ = 0.0;
    torque_yaw_velocity_i_clamp_ = 0.0;
    force_x_velocity_i_clamp_ = 0.0;
    force_y_velocity_i_clamp_ = 0.0;
    
    if (sdf->HasElement("yaw_velocity_p_gain"))
      (sdf->GetElement("yaw_velocity_p_gain")->GetValue()->Get(torque_yaw_velocity_p_gain_));

    if (sdf->HasElement("x_velocity_p_gain"))
      (sdf->GetElement("x_velocity_p_gain")->GetValue()->Get(force_x_velocity_p_gain_));

    if (sdf->HasElement("y_velocity_p_gain"))
      (sdf->GetElement("y_velocity_p_gain")->GetValue()->Get(force_y_velocity_p_gain_));

    if (sdf->HasElement("yaw_velocity_i_gain"))
      (sdf->GetElement("yaw_velocity_i_gain")->GetValue()->Get(torque_yaw_velocity_i_gain_));

    if (sdf->HasElement("x_velocity_i_gain"))
      (sdf->GetElement("x_velocity_i_gain")->GetValue()->Get(force_x_velocity_i_gain_));

    if (sdf->HasElement("y_velocity_i_gain"))
      (sdf->GetElement("y_velocity_i_gain")->GetValue()->Get(force_y_velocity_i_gain_));

    if (sdf->HasElement("yaw_velocity_d_gain"))
      (sdf->GetElement("yaw_velocity_d_gain")->GetValue()->Get(torque_yaw_velocity_d_gain_));

    if (sdf->HasElement("x_velocity_d_gain"))
      (sdf->GetElement("x_velocity_d_gain")->GetValue()->Get(force_x_velocity_d_gain_));

    if (sdf->HasElement("y_velocity_d_gain"))
      (sdf->GetElement("y_velocity_d_gain")->GetValue()->Get(force_y_velocity_d_gain_));

    if (sdf->HasElement("yaw_velocity_i_clamp"))
      (sdf->GetElement("yaw_velocity_i_clamp")->GetValue()->Get(torque_yaw_velocity_i_clamp_));

    if (sdf->HasElement("x_velocity_i_clamp"))
      (sdf->GetElement("x_velocity_i_clamp")->GetValue()->Get(force_x_velocity_i_clamp_));

    if (sdf->HasElement("y_velocity_i_clamp"))
      (sdf->GetElement("y_velocity_i_clamp")->GetValue()->Get(force_y_velocity_i_clamp_));

      
    ROS_INFO_STREAM("ForceBasedMove using gains:\np yaw: " << torque_yaw_velocity_p_gain_ <<
                                                 "\np x: " << force_x_velocity_p_gain_ <<
                                                 "\np y: " << force_y_velocity_p_gain_ <<
                                                 "\ni yaw: " << torque_yaw_velocity_i_gain_ <<
                                                 "\ni x: " << force_x_velocity_i_gain_ <<
                                                 "\ni y: " << force_y_velocity_i_gain_ <<
                                                 "\nd yaw: " << torque_yaw_velocity_d_gain_ <<
                                                 "\nd x: " << force_x_velocity_d_gain_ <<
                                                 "\nd y: " << force_y_velocity_d_gain_ <<
                                                 "\ni_clamp yaw: " << torque_yaw_velocity_i_clamp_ <<
                                                 "\ni_clamp x: " << force_x_velocity_i_clamp_ <<
                                                 "\ni_clamp y: " << force_y_velocity_i_clamp_);

    robot_base_frame_ = "base_footprint";
    if (!sdf->HasElement("robotBaseFrame")) 
    {
      ROS_WARN("PlanarMovePlugin (ns = %s) missing <robotBaseFrame>, "
          "defaults to \"%s\"",
          robot_namespace_.c_str(), robot_base_frame_.c_str());
    } 
    else 
    {
      robot_base_frame_ = sdf->GetElement("robotBaseFrame")->Get<std::string>();
    }

    ROS_INFO_STREAM("robotBaseFrame for force based move plugin: " << robot_base_frame_  << "\n");

    this->link_ = parent->GetLink(robot_base_frame_);

    odometry_rate_ = 20.0;
    if (!sdf->HasElement("odometryRate")) 
    {
      ROS_WARN("PlanarMovePlugin (ns = %s) missing <odometryRate>, "
          "defaults to %f",
          robot_namespace_.c_str(), odometry_rate_);
    } 
    else 
    {
      odometry_rate_ = sdf->GetElement("odometryRate")->Get<double>();
    } 

    this->publish_odometry_tf_ = true;
    if (!sdf->HasElement("publishOdometryTf")) {
      ROS_WARN("PlanarMovePlugin Plugin (ns = %s) missing <publishOdometryTf>, defaults to %s",
               this->robot_namespace_.c_str(), this->publish_odometry_tf_ ? "true" : "false");
    } else {
      this->publish_odometry_tf_ = sdf->GetElement("publishOdometryTf")->Get<bool>();
    }
 
#if (GAZEBO_MAJOR_VERSION >= 8)
    last_odom_publish_time_ = parent_->GetWorld()->SimTime();
    last_odom_pose_ = parent_->WorldPose();
#else
    last_odom_publish_time_ = parent_->GetWorld()->GetSimTime();
    last_odom_pose_ = parent_->GetWorldPose();
#endif

    x_ = 0.0;
    y_ = 0.0;
    yaw_ = 0.0;
    alive_ = true;

    odom_transform_.setIdentity();

    // Ensure that ROS has been initialized and subscribe to cmd_vel
    if (!ros::isInitialized()) 
    {
      ROS_FATAL_STREAM("PlanarMovePlugin (ns = " << robot_namespace_
        << "). A ROS node for Gazebo has not been initialized, "
        << "unable to load plugin. Load the Gazebo system plugin "
        << "'libgazebo_ros_api_plugin.so' in the gazebo_ros package)");
      return;
    }
    rosnode_.reset(new ros::NodeHandle(robot_namespace_));

    ROS_DEBUG("ForceBasedMove Plugin (%s) has started!", 
        robot_namespace_.c_str());

    tf_prefix_ = tf::getPrefixParam(*rosnode_);

    if (publish_odometry_tf_)
      transform_broadcaster_.reset(new tf::TransformBroadcaster());

    // subscribe to the odometry topic
    ros::SubscribeOptions so =
      ros::SubscribeOptions::create<geometry_msgs::Twist>(command_topic_, 1,
          boost::bind(&GazeboRosForceBasedMove::cmdVelCallback, this, _1),
          ros::VoidPtr(), &queue_);

    vel_sub_ = rosnode_->subscribe(so);
    odometry_pub_ = rosnode_->advertise<nav_msgs::Odometry>(odometry_topic_, 1);

    // start custom queue for diff drive
    callback_queue_thread_ = 
      boost::thread(boost::bind(&GazeboRosForceBasedMove::QueueThread, this));

    // listen to the update event (broadcast every simulation iteration)
    update_connection_ = 
      event::Events::ConnectWorldUpdateBegin(
          boost::bind(&GazeboRosForceBasedMove::UpdateChild, this));

    // Apparently the dynamic reconfigure server only instances if there is at least
    // a 'p' param for it
    rosnode_->setParam("force_based_move/yaw_pid/p", torque_yaw_velocity_p_gain_);
    rosnode_->setParam("force_based_move/yaw_pid/i", torque_yaw_velocity_i_gain_);
    rosnode_->setParam("force_based_move/yaw_pid/d", torque_yaw_velocity_d_gain_);
    rosnode_->setParam("force_based_move/yaw_pid/i_clamp_min", -torque_yaw_velocity_i_clamp_);
    rosnode_->setParam("force_based_move/yaw_pid/i_clamp_max", torque_yaw_velocity_i_clamp_);
    pid_yaw_velocity_.init(ros::NodeHandle(*rosnode_, "force_based_move/yaw_pid"), false);

    rosnode_->setParam("force_based_move/x_pid/p", force_x_velocity_p_gain_);
    rosnode_->setParam("force_based_move/x_pid/i", force_x_velocity_i_gain_);
    rosnode_->setParam("force_based_move/x_pid/d", force_x_velocity_d_gain_);
    rosnode_->setParam("force_based_move/x_pid/i_clamp_min", -force_x_velocity_i_clamp_);
    rosnode_->setParam("force_based_move/x_pid/i_clamp_max", force_x_velocity_i_clamp_);
    pid_x_velocity_.init(ros::NodeHandle(*rosnode_, "force_based_move/x_pid"), false);

    rosnode_->setParam("force_based_move/y_pid/p", force_y_velocity_p_gain_);
    rosnode_->setParam("force_based_move/y_pid/i", force_y_velocity_i_gain_);
    rosnode_->setParam("force_based_move/y_pid/d", force_y_velocity_d_gain_);
    rosnode_->setParam("force_based_move/y_pid/i_clamp_min", -force_y_velocity_i_clamp_);
    rosnode_->setParam("force_based_move/y_pid/i_clamp_max", force_y_velocity_i_clamp_);
    pid_y_velocity_.init(ros::NodeHandle(*rosnode_, "force_based_move/y_pid"), false);
  }


  // Update the controller
  void GazeboRosForceBasedMove::UpdateChild()
  {
    boost::mutex::scoped_lock scoped_lock(lock);
    ros::Time tnow = ros::Time::now();
    ros::Duration dt = tnow - last_pid_update_time_;
#if (GAZEBO_MAJOR_VERSION >= 8)
    ignition::math::Pose3d pose = parent_->WorldPose();

    ignition::math::Vector3d angular_vel = parent_->WorldAngularVel();

    double value_yaw = pid_yaw_velocity_.computeCommand(pid_yaw_velocity_.getCurrentCmd() - angular_vel.Z(), dt);

    link_->AddTorque(ignition::math::Vector3d(0.0, 0.0, value_yaw));


    ignition::math::Vector3d linear_vel = parent_->RelativeLinearVel();

    double value_x = pid_x_velocity_.computeCommand(pid_x_velocity_.getCurrentCmd() - linear_vel.X(), dt);
    double value_y = pid_y_velocity_.computeCommand(pid_y_velocity_.getCurrentCmd() - linear_vel.Y(), dt);

    link_->AddRelativeForce(ignition::math::Vector3d(value_x,
                                                     value_y,
                                                     0.0));
#else
    math::Pose pose = parent_->GetWorldPose();

    math::Vector3 angular_vel = parent_->GetWorldAngularVel();
    math::Vector3 linear_vel = parent_->GetRelativeLinearVel();

    double  value_yaw = pid_yaw_velocity_.computeCommand(yaw_ - angular_vel.z, dt);
    link_->AddTorque(math::Vector3(0.0, 0.0, value_yaw));

    double value_x = pid_x_velocity_.computeCommand(x_ - linear_vel.x, dt);
    double value_y = pid_y_velocity_.computeCommand(y_ - linear_vel.y, dt);

    link_->AddRelativeForce(math::Vector3(value_x,
                                          value_y,
                                            0.0));
#endif
    last_pid_update_time_ = tnow;

    if (odometry_rate_ > 0.0) {
#if (GAZEBO_MAJOR_VERSION >= 8)
      common::Time current_time = parent_->GetWorld()->SimTime();
#else
      common::Time current_time = parent_->GetWorld()->GetSimTime();
#endif
      double seconds_since_last_update = 
        (current_time - last_odom_publish_time_).Double();
      if (seconds_since_last_update > (1.0 / odometry_rate_)) {
        publishOdometry(seconds_since_last_update);
        last_odom_publish_time_ = current_time;
      }
    }
  }

  // Finalize the controller
  void GazeboRosForceBasedMove::FiniChild() {
    alive_ = false;
    queue_.clear();
    queue_.disable();
    rosnode_->shutdown();
    callback_queue_thread_.join();
  }

  void GazeboRosForceBasedMove::cmdVelCallback(
      const geometry_msgs::Twist::ConstPtr& cmd_msg) 
  {
    boost::mutex::scoped_lock scoped_lock(lock);
    x_ = cmd_msg->linear.x;
    y_ = cmd_msg->linear.y;
    yaw_ = cmd_msg->angular.z;
    ROS_DEBUG_STREAM("Updating command:\n" << 
      "x: " << cmd_msg->linear.x <<
      "\ny: " << cmd_msg->linear.y <<
      "\nyaw: " << cmd_msg->angular.z);
 
  }

  void GazeboRosForceBasedMove::QueueThread()
  {
    static const double timeout = 0.01;
    while (alive_ && rosnode_->ok()) 
    {
      queue_.callAvailable(ros::WallDuration(timeout));
    }
  }

  void GazeboRosForceBasedMove::publishOdometry(double step_time)
  {

    ros::Time current_time = ros::Time::now();
    std::string odom_frame = tf::resolve(tf_prefix_, odometry_frame_);
    std::string base_footprint_frame = 
      tf::resolve(tf_prefix_, robot_base_frame_);

#if (GAZEBO_MAJOR_VERSION >= 8)
    ignition::math::Vector3d angular_vel = parent_->RelativeAngularVel();
    ignition::math::Vector3d linear_vel = parent_->RelativeLinearVel();

    odom_transform_= odom_transform_ * this->getTransformForMotion(linear_vel.X(), linear_vel.Y(), angular_vel.Z(), step_time);

    tf::poseTFToMsg(odom_transform_, odom_.pose.pose);
    odom_.twist.twist.angular.z = angular_vel.Z();
    odom_.twist.twist.linear.x  = linear_vel.X();
    odom_.twist.twist.linear.y = linear_vel.Y();
#else
    math::Vector3 angular_vel = parent_->GetRelativeAngularVel();
    math::Vector3 linear_vel = parent_->GetRelativeLinearVel();

    odom_transform_= odom_transform_ * this->getTransformForMotion(linear_vel.x, linear_vel.y, angular_vel.z, step_time);

    tf::poseTFToMsg(odom_transform_, odom_.pose.pose);
    odom_.twist.twist.angular.z = angular_vel.z;
    odom_.twist.twist.linear.x  = linear_vel.x;
    odom_.twist.twist.linear.y = linear_vel.y;
#endif

    odom_.header.stamp = current_time;
    odom_.header.frame_id = odom_frame;
    odom_.child_frame_id = base_footprint_frame;

    if (transform_broadcaster_.get()){
      transform_broadcaster_->sendTransform(
          tf::StampedTransform(odom_transform_, current_time, odom_frame,
              base_footprint_frame));
    }
    
    odom_.pose.covariance[0] = 0.001;
    odom_.pose.covariance[7] = 0.001;
    odom_.pose.covariance[14] = 1000000000000.0;
    odom_.pose.covariance[21] = 1000000000000.0;
    odom_.pose.covariance[28] = 1000000000000.0;
    
#if (GAZEBO_MAJOR_VERSION >= 8)
    if (std::abs(angular_vel.Z()) < 0.0001) {
#else
    if (std::abs(angular_vel.z) < 0.0001) {
#endif
      odom_.pose.covariance[35] = 0.01;
    }else{
      odom_.pose.covariance[35] = 100.0;
    }

    odom_.twist.covariance[0] = 0.001;
    odom_.twist.covariance[7] = 0.001;
    odom_.twist.covariance[14] = 0.001;
    odom_.twist.covariance[21] = 1000000000000.0;
    odom_.twist.covariance[28] = 1000000000000.0;

#if (GAZEBO_MAJOR_VERSION >= 8)
    if (std::abs(angular_vel.Z()) < 0.0001) {
#else
    if (std::abs(angular_vel.z) < 0.0001) {
#endif
      odom_.twist.covariance[35] = 0.01;
    }else{
      odom_.twist.covariance[35] = 100.0;
    }



    odometry_pub_.publish(odom_);
  }


  tf::Transform GazeboRosForceBasedMove::getTransformForMotion(double linear_vel_x, double linear_vel_y, double angular_vel, double timeSeconds) const
  {
    tf::Transform tmp;
    tmp.setIdentity();


    if (std::abs(angular_vel) < 0.0001) {
      //Drive straight
      tmp.setOrigin(tf::Vector3(static_cast<double>(linear_vel_x*timeSeconds), static_cast<double>(linear_vel_y*timeSeconds), 0.0));
    } else {
      //Follow circular arc
      double distChange = linear_vel_x * timeSeconds + linear_vel_y * timeSeconds;
      double angleChange = angular_vel * timeSeconds;

      double arcRadius = distChange / angleChange;

      tmp.setOrigin(tf::Vector3(std::sin(angleChange) * arcRadius,
                                arcRadius - std::cos(angleChange) * arcRadius,
                                0.0));
      tmp.setRotation(tf::createQuaternionFromYaw(angleChange));
    }

    return tmp;
  }

  GZ_REGISTER_MODEL_PLUGIN(GazeboRosForceBasedMove)
}

