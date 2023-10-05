#include "robot_hallucination.h"
#include <robot_state_publisher/robot_state_publisher.h>
#include <kdl_parser/kdl_parser.hpp>
#include <geometry_msgs/PoseArray.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <boost/algorithm/string/join.hpp>


#include <string>

bool RobotHallucination::onEnable()
{
  ROS_DEBUG_STREAM("Enablng Robot Hallucination");
  joint_states_sub_ = nh_.subscribe("/joint_states", 10, &RobotHallucination::jointStatesCB, this);
  return true;
}

bool RobotHallucination::onDisable()
{
  ROS_DEBUG_STREAM("Disabling Robot Hallucination");
  joint_states_sub_.shutdown();
  return true;
}


bool RobotHallucination::clear()
{
  prefixes_.clear();
  robot_state_publisher_.reset();
  // TODO: Clear existing transformations?
}

bool RobotHallucination::load(urdf::Model& model)
{
  KDL::Tree robot_tree;
  if (!kdl_parser::treeFromUrdfModel(model, robot_tree)){
      ROS_ERROR("Failed to construct kdl tree");
      return false;
   }
  robot_state_publisher_ = boost::make_shared<robot_state_publisher::RobotStatePublisher>(robot_tree);
}

void RobotHallucination::posesCB(const geometry_msgs::PoseArray::ConstPtr& msg)
{
  prefixes_.clear();
  std::string base_frame_id = msg->header.frame_id;
  for(int i = 0; i < msg->poses.size(); ++i)
  {
    const geometry_msgs::Pose& pose = msg->poses[i];
    // tf::Transform transform;
    // tf::poseMsgToTF(pose, transform);
    
    // ROS_DEBUG_STREAM("pose = [" << pose.position.x << "," << pose.position.y << "," << pose.position.z << "](" << pose.orientation.x << "," << pose.orientation.y << "," << pose.orientation.z << "," << pose.orientation.w << ") transform = [" << transform.getOrigin().x() << "," << transform.getOrigin().y() << "," <<transform.getOrigin().z() << "](" << transform.getRotation().getX() << "," << transform.getRotation().getY() << "," << transform.getRotation().getZ() << "," << transform.getRotation().getW() << ")");
    
    // geometry_msgs::TransformStamped transformMsg;
    // tf::transformTFToMsg(transform, transformMsg.transform);
    geometry_msgs::TransformStamped transformMsg;
    transformMsg.transform.translation.x = pose.position.x; 
    transformMsg.transform.translation.y = pose.position.y; 
    transformMsg.transform.translation.z = pose.position.z; 
    transformMsg.transform.rotation.x = pose.orientation.x;
    transformMsg.transform.rotation.y = pose.orientation.y;
    transformMsg.transform.rotation.z = pose.orientation.z;
    transformMsg.transform.rotation.w = pose.orientation.w;

    std::string prefix = genTfPrefix(i);
    transformMsg.header.frame_id = base_frame_id;
    transformMsg.header.stamp = msg->header.stamp;
    transformMsg.child_frame_id = prefix + "/" + base_frame_id;
    
    prefixes_.push_back(prefix);
    br_.sendTransform(transformMsg);
    

  }
  
  ROS_DEBUG_STREAM("Prefixes: " << boost::algorithm::join(prefixes_, ",") );
  
}

std::string RobotHallucination::genTfPrefix(int i)
{
  return "robot_" + NumberToString(i);
}

void RobotHallucination::jointStatesCB(const sensor_msgs::JointState::ConstPtr& msg)
{
  ROS_DEBUG_STREAM_THROTTLE(2,"Joint States callback. Prefix size=" << prefixes_.size());
  std::map<std::string, double> map;
  ROS_ASSERT(msg->name.size() == msg->position.size());
  for (size_t i = 0; i < msg->name.size(); ++i)
  {
      map[msg->name[i]] = msg->position[i];
  }
  
  for(int i = 0; i < prefixes_.size(); ++i)
  {
    robot_state_publisher_->publishTransforms(map, msg->header.stamp, prefixes_[i]);
    robot_state_publisher_->publishFixedTransforms(prefixes_[i], false);
  }
}

  
