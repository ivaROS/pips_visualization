/*
 * Copyright (c) 2008, Willow Garage, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include "robot_model_array_display.h"
#include "robot_hallucination.h"

#include <OgreSceneNode.h>
#include <OgreSceneManager.h>

#include <urdf/model.h>

#include <tf/transform_listener.h>

#include "rviz/display_context.h"
#include "rviz/robot/robot.h"
#include "rviz/robot/tf_link_updater.h"
#include "rviz/properties/float_property.h"
#include "rviz/properties/property.h"
#include "rviz/properties/string_property.h"
#include "rviz/properties/ros_topic_property.h"

#include <geometry_msgs/PoseArray.h>

namespace rviz
{

void linkUpdaterStatusFunction( StatusProperty::Level level,
                                const std::string& link_name,
                                const std::string& text,
                                RobotModelArrayDisplay* display )
{
  display->setStatus( level, QString::fromStdString( link_name ), QString::fromStdString( text ));
}

RobotModelArrayDisplay::RobotModelArrayDisplay()
  : Display()
  , has_new_transforms_( false )
  , time_since_last_transform_( 0.0f )
{
  visual_enabled_property_ = new Property( "Visual Enabled", true,
                                           "Whether to display the visual representation of the robot.",
                                           this, SLOT( updateVisualVisible() ));

  collision_enabled_property_ = new Property( "Collision Enabled", false,
                                              "Whether to display the collision representation of the robot.",
                                              this, SLOT( updateCollisionVisible() ));

  update_rate_property_ = new FloatProperty( "Update Interval", 0,
                                             "Interval at which to update the links, in seconds. "
                                             " 0 means to update every update cycle.",
                                             this );
  update_rate_property_->setMin( 0 );

  alpha_property_ = new FloatProperty( "Alpha", .5,
                                       "Amount of transparency to apply to the links.",
                                       this, SLOT( updateAlpha() ));
  alpha_property_->setMin( 0.0 );
  alpha_property_->setMax( 1.0 );

  robot_description_property_ = new StringProperty( "Robot Description", "robot_description",
                                                    "Name of the parameter to search for to load the robot description.",
                                                    this, SLOT( updateRobotDescription() ));

  /* TODO: set a topic to subscribe to that will contain an array of the prefixes to use. 
           Another option would be to create robot_state_publishers as needed, such that could subscribe to an array of transforms and do the rest internally */
  poses_topic_property_ = new RosTopicProperty( "Pose Topic", "/poses", "geometry_msgs::PoseArray",
                                            "The topic to listen for hallucinated poses on",
                                            this, SLOT( updatePoseTopic() ));
                                            
}

RobotModelArrayDisplay::~RobotModelArrayDisplay()
{

}

void RobotModelArrayDisplay::onInitialize()
{

}

void RobotModelArrayDisplay::updatePoseTopic()
{
  ROS_DEBUG_STREAM("Updated pose topic: " << poses_topic_property_->getStdString());
  poses_sub_ = nh_.subscribe(poses_topic_property_->getStdString(), 10, &RobotModelArrayDisplay::posesCB, this);
  if(poses_sub_.getNumPublishers() == 0)
  {
    ROS_ERROR_STREAM("Topic (" << poses_sub_.getTopic() << ") has no publishers!");
  }

}

void RobotModelArrayDisplay::updateAlpha()
{
  for(int i = 0; i < robots_.size(); ++i)
  {
    robots_[i]->setAlpha( alpha_property_->getFloat() );
  }
  context_->queueRender();
}

void RobotModelArrayDisplay::updateRobotDescription()
{
  if( isEnabled() )
  {
    ROS_DEBUG_STREAM("Updated robot description");
    load();
    context_->queueRender();
  }
}

void RobotModelArrayDisplay::updateVisualVisible()
{
  ROS_DEBUG_STREAM("Updated visual visible");
  for(int i = 0; i < robots_.size(); ++i)
  {
    robots_[i]->setVisualVisible( visual_enabled_property_->getValue().toBool() );
  }
  context_->queueRender();
}

void RobotModelArrayDisplay::updateCollisionVisible()
{
  ROS_DEBUG_STREAM("Updated collision visible");
  for(int i = 0; i < robots_.size(); ++i)
  {
    robots_[i]->setCollisionVisible( collision_enabled_property_->getValue().toBool() );
  }
  context_->queueRender();
}

void RobotModelArrayDisplay::posesCB(const geometry_msgs::PoseArray::ConstPtr& msg)
{
  hallucinator_.posesCB(msg);
  
  int num_poses = msg->poses.size();
  int num_robots = robots_.size();
  
  if(num_robots > num_poses)
  {
    for(int i = num_poses; i < num_robots; ++i)
    {
      rviz::Property * linkTree = robots_[i]->getLinkTreeProperty();
      linkTree->hide();
    }
    robots_.resize(num_poses);
  }
  else if(num_robots < num_poses)
  {
    for(int i = num_robots; i < num_poses; ++i)
    {
      boost::shared_ptr<Robot> new_robot = boost::make_shared<Robot>( scene_node_, context_, "Robot: " + getName().toStdString(), this );
      new_robot->load( urdf_ );
      new_robot->setVisible( true );
      new_robot->getLinkTreeProperty()->setName(QString::fromStdString(genTfPrefix(i) + " Links"));
      robots_.push_back(new_robot);
    }
  }
  
  ROS_DEBUG_STREAM("Poses callback: " << num_poses << " poses");
  
  updateVisualVisible();
  updateCollisionVisible();
  updateAlpha();
  
  clearStatuses();
  context_->queueRender();
}

void RobotModelArrayDisplay::load()
{
  ROS_DEBUG("load()");
  std::string content;
  if( !update_nh_.getParam( robot_description_property_->getStdString(), content ))
  {
    std::string loc;
    if( update_nh_.searchParam( robot_description_property_->getStdString(), loc ))
    {
      update_nh_.getParam( loc, content );
    }
    else
    {
      clear();
      setStatus( StatusProperty::Error, "URDF",
                 "Parameter [" + robot_description_property_->getString()
                 + "] does not exist, and was not found by searchParam()" );
      return;
    }
  }

  if( content.empty() )
  {
    clear();
    setStatus( StatusProperty::Error, "URDF", "URDF is empty" );
    return;
  }

  if( content == robot_description_ )
  {
    return;
  }

  robot_description_ = content;

  TiXmlDocument doc;
  doc.Parse( robot_description_.c_str() );
  if( !doc.RootElement() )
  {
    clear();
    setStatus( StatusProperty::Error, "URDF", "URDF failed XML parse" );
    return;
  }

  urdf::Model descr;
  if( !descr.initXml( doc.RootElement() ))
  {
    clear();
    setStatus( StatusProperty::Error, "URDF", "URDF failed Model parse" );
    return;
  }
  
  hallucinator_.load(descr);
  
  urdf_ = descr;

  setStatus( StatusProperty::Ok, "URDF", "URDF parsed OK" );
  
  for(int i = 0; i < robots_.size(); ++i)
  {
    robots_[i]->load(urdf_);
    robots_[i]->update( TFLinkUpdater( context_->getFrameManager(),
                                 boost::bind( linkUpdaterStatusFunction, _1, _2, _3, this ),
                                 genTfPrefix(i) ));
  }
}


std::string RobotModelArrayDisplay::genTfPrefix(int i)
{
  std::string prefix = hallucinator_.genTfPrefix(i);
  ROS_DEBUG_STREAM("prefix: " << i << " => " <<prefix);
  return prefix;
}

void RobotModelArrayDisplay::onEnable()
{
  ROS_DEBUG("enabled");
  hallucinator_.onEnable();
  load();
  updatePoseTopic();

  for(int i = 0; i < robots_.size(); ++i)
  {
    robots_[i]->setVisible( true );
  }
}

void RobotModelArrayDisplay::onDisable()
{
  ROS_DEBUG("disabled");
  for(int i = 0; i < robots_.size(); ++i)
  {
    robots_[i]->setVisible( false );
  }
  poses_sub_.shutdown();
  hallucinator_.onDisable();
  
  clear();
}

void RobotModelArrayDisplay::update( float wall_dt, float ros_dt )
{
  time_since_last_transform_ += wall_dt;
  float rate = update_rate_property_->getFloat();
  bool update = rate < 0.0001f || time_since_last_transform_ >= rate;

  if( has_new_transforms_ || update ) 
  {
    for(int i = 0; i < robots_.size(); ++i)
    {
      robots_[i]->update( TFLinkUpdater( context_->getFrameManager(),
                                   boost::bind( linkUpdaterStatusFunction, _1, _2, _3, this ),
                                   genTfPrefix(i) ));                              
    }
    context_->queueRender();

    has_new_transforms_ = false;
    time_since_last_transform_ = 0.0f;
  }
  
  if(poses_sub_.getNumPublishers() == 0)
  {
    ROS_WARN_STREAM_ONCE("Topic (" << poses_sub_.getTopic() << ") has no publishers! This message will only print once");
  }

  
  
}

void RobotModelArrayDisplay::fixedFrameChanged()
{
  has_new_transforms_ = true;
}

void RobotModelArrayDisplay::clear()
{
  for(int i = 0; i < robots_.size(); ++i)
  {
    rviz::Property * linkTree = robots_[i]->getLinkTreeProperty();
    linkTree->hide();
  }
  robots_.clear();
  
  clearStatuses();
  robot_description_.clear();
  urdf_.clear();
  hallucinator_.clear();
}

void RobotModelArrayDisplay::reset()
{
  Display::reset();
  has_new_transforms_ = true;
}

} // namespace rviz

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS( rviz::RobotModelArrayDisplay, rviz::Display )
