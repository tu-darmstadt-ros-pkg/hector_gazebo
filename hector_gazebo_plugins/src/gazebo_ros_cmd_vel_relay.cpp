#include <hector_gazebo_plugins/gazebo_ros_cmd_vel_relay.h>

namespace gazebo {

void GazeboRosCmdVelRelay::Load(physics::ModelPtr parent, sdf::ElementPtr sdf)
{
  gzmsg << "Loading plugin GazeboRosCmdVelRelay" << std::endl;
  // Load Parameters
  std::string ros_command_topic = "/cmd_vel_raw";
  if (sdf->HasElement("ros_command_topic"))
  {
    ros_command_topic = sdf->GetElement("ros_command_topic")->Get<std::string>();
  }
  else
  {
    ROS_WARN_STREAM("GazeboRosCmdVelRelay Plugin is missing <ros_command_topic>, defaulting to '/cmd_vel_raw'");
  }
  gzmsg << "Subscribing to ros topic '" << ros_command_topic << "'." << std::endl;

  std::string gz_command_topic = "/cmd_vel_raw";
  if (sdf->HasElement("gz_command_topic"))
  {
    gz_command_topic = sdf->GetElement("gz_command_topic")->Get<std::string>();
  }
  else
  {
    ROS_WARN_STREAM("GazeboRosCmdVelRelay plugin is missing <gz_command_topic>, defaulting to '/cmd_vel_raw'");
  }
  gzmsg << "Publishing to gz topic '" << gz_command_topic << "'." << std::endl;

  // Initialize transport nodes
  if (!ros::isInitialized()) {
    ROS_ERROR_STREAM("GazeboRosCmdVelRelay plugin: ROS has not been initialized yet.");
    return;
  }
  ros_node_ptr_ = boost::make_shared<ros::NodeHandle>();
  gz_node_ptr_ = boost::make_shared<transport::Node>();
  gz_node_ptr_->Init();

  // Start subscribers and publishers
  cmd_vel_pub_ = gz_node_ptr_->Advertise<msgs::Pose>(gz_command_topic, 1000);
  cmd_vel_sub_ = ros_node_ptr_->subscribe(ros_command_topic, 1000, &GazeboRosCmdVelRelay::cmdVelCb, this);
  gzmsg << "Successfully loaded plugin GazeboRosCmdVelRelay" << std::endl;
}

void GazeboRosCmdVelRelay::cmdVelCb(const geometry_msgs::TwistConstPtr& twist_ptr)
{
  ignition::math::Pose3d pose3d(twist_ptr->linear.x, twist_ptr->linear.y, twist_ptr->linear.z,
                                twist_ptr->angular.x, twist_ptr->angular.y, twist_ptr->angular.z);
  msgs::Pose gz_pose = msgs::Convert(pose3d);
  cmd_vel_pub_->Publish(gz_pose);

//  gz_pose.mutable_position()->set_x(twist_ptr->linear.x);
//  gz_pose.mutable_position()->set_y(twist_ptr->linear.y);
//  gz_pose.mutable_position()->set_z(twist_ptr->linear.z);
////  gz_pose.mutable_orientation()->
}

  GZ_REGISTER_MODEL_PLUGIN(GazeboRosCmdVelRelay)
}
