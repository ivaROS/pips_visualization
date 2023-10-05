#include <robot_state_publisher/robot_state_publisher.h>
#include <kdl_parser/kdl_parser.hpp>
#include <sensor_msgs/JointState.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <geometry_msgs/PoseArray.h>
#include <sstream>


#ifndef ROBOT_HALLUCINATION_H
#define ROBOT_HALLUCINATION_H

template <typename T>
  std::string NumberToString ( T Number )
  {
     std::ostringstream ss;
     ss << Number;
     return ss.str();
  }
  
class  RobotHallucination
{

typedef boost::shared_ptr<robot_state_publisher::RobotStatePublisher> RobotPub;

public:
  bool onEnable();
  bool onDisable();
  bool load(urdf::Model& model);
  bool clear();
  std::string genTfPrefix(int i);
  void posesCB(const geometry_msgs::PoseArray::ConstPtr& msg);

protected:
  void jointStatesCB(const sensor_msgs::JointState::ConstPtr& msg);
  
  tf2_ros::StaticTransformBroadcaster br_;
  ros::NodeHandle nh_;
  RobotPub robot_state_publisher_;
  std::vector<std::string> prefixes_;
  ros::Subscriber joint_states_sub_;
};

#endif /* ROBOT_HALLUCINATION_H */
