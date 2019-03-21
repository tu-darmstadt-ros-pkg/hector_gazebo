#ifndef HECTOR_GAZEBO_PLUGINS_GAZEBO_ROS_CMD_VEL_RELAY_H
#define HECTOR_GAZEBO_PLUGINS_GAZEBO_ROS_CMD_VEL_RELAY_H

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <gazebo/common/common.hh>
#include <gazebo/transport/transport.hh>
#include <gazebo/msgs/msgs.hh>

namespace gazebo {

class GazeboRosCmdVelRelay : public ModelPlugin {
public:
  void Load(physics::ModelPtr parent, sdf::ElementPtr sdf) override;
private:
  void cmdVelCb(const geometry_msgs::TwistConstPtr& twist_ptr);
  ros::NodeHandlePtr ros_node_ptr_;
  transport::NodePtr gz_node_ptr_;

  ros::Subscriber cmd_vel_sub_;
  transport::PublisherPtr cmd_vel_pub_;

};

}

#endif
